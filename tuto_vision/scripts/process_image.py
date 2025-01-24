#!/usr/bin/env python3

import signal
import time
import numpy as np
import sys
import cv2
import pyrealsense2 as rs
import torch
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import rclpy.time
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from kobuki_ros_interfaces.msg import Sound
from geometry_msgs.msg import Pose
from kobuki_ros_interfaces.msg import Sound
from tf2_ros import *
from tf2_geometry_msgs import *
import math

# Charger le modèle YOLOv5

model = torch.hub.load(
    "/home/a2s4/yolov5/", 'custom', path='/home/a2s4/yolov5/weightsss/best20.pt', source='local'
)
model.conf=0.5
"""
model20 = torch.hub.load(
    "/home/a2s4/yolov5/", 'custom', path='/home/a2s4/yolov5/weightsss/best20.pt', source='local'
)
model20.conf=0.5
model20.iou = 0.99  # Réglez ce seuil (valeur typique : 0.45)
#model20.conf = 0.5  # Réglez le seuil de confiance (valeur typique : 0.5)
"""


# Définir le seuil de confiance
confidence_threshold = 0.5  # Ajustez selon vos besoins

def signal_interruption(signum, frame):
    global is_running
    print("\nCtrl-c pressed")
    is_running = False

def main(args=None):
    global is_running
    is_running = True

    # Initialisation de la caméra RealSense
    pipeline = rs.pipeline()
    config = rs.config()

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    print(f"Connected: {device_product_line}")

    # Vérifier si la caméra RGB est disponible
    found_rgb = any(
        s.get_info(rs.camera_info.name) == 'RGB Camera' for s in device.sensors
    )
    if not found_rgb:
        print("Depth camera with RGB required!")
        exit(0)

    # Configuration des flux de la caméra
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

    # Gestion de l'interruption
    signal.signal(signal.SIGINT, signal_interruption)

    # Démarrer le flux de la caméra
    pipeline.start(config)

    # Initialiser ROS2
    rclpy.init(args=args)
    rs_node = Realsense(pipeline)

    while is_running:
        rclpy.spin_once(rs_node, timeout_sec=0.001)

        # Obtenir une image RGB et une image de profondeur

        frames = pipeline.wait_for_frames()
        aligned_frames = rs_node.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Conversion en tableaux numpy
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image_bgr = np.asanyarray(color_frame.get_data())

        if len(color_image_bgr.shape) == 2:  # Image en niveaux de gris
            print("Converting grayscale image to BGR")
            color_image_bgr = cv2.cvtColor(color_image_bgr, cv2.COLOR_GRAY2BGR)
    
        # Appliquer YOLO pour la détection
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

        color_image_rgb = cv2.cvtColor(color_image_bgr, cv2.COLOR_BGR2RGB)
        results = model(color_image_rgb)
        results.render()
        rs_node.results = results
        #results20 = model20(image_for_model)
        #results20.render()
        
        
        detection_image = np.array(results.ims[0])

        detection_image = cv2.cvtColor(detection_image, cv2.COLOR_RGB2BGR)

        #detection_image20 = np.array(results20.ims[0])

        #detection_image20 = cv2.cvtColor(detection_image20, cv2.COLOR_RGB2BGR)

        # Publier les images, marqueurs et distances
        rs_node.publish_image(color_image, detection_image)
        rs_node.publish_markers(results, depth_frame, color_intrin)
        #rs_node.publish_distances(results, depth_frame, frames)

        # Afficher les résultats
        cv2.namedWindow('YOLO Detection', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('YOLO Detection', np.concatenate((detection_image,detection_image20),axis=0))
        cv2.imshow('YOLO Detection', detection_image)
        cv2.waitKey(1)

    # Arrêter le flux
    print("Stopping...")
    pipeline.stop()

    # Fermer proprement
    rs_node.destroy_node()
    rclpy.shutdown()

# Classe ROS2 pour la publication des images et des marqueurs
class Realsense(Node):
    def __init__(self, pipeline):
        super().__init__('realsense')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'realsense/color_image', 10)
        self.detection_publisher = self.create_publisher(Image, 'yolo/detection', 10)
        #self.marker_publisher = self.create_publisher(MarkerArray, 'yolo/markers', 10)
        self.marker_publisher = self.create_publisher(Marker, 'yolo/markers', 10)
        self.sound_publisher = self.create_publisher(Sound, '/commands/sound', 10)
        self.distance_publisher = self.create_publisher(Pose, 'distance', 10)
        self.pipeline = pipeline
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.detected = False
        self.number_object = 0

        # Transform tool:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.results=None
        self.marker_id=0
        self.Pose_markers=[]
        #self.marker_array=MarkerArray()
        #marker_array=MarkerArray()
        self.distance=0.2

        #self.color_intrin= None

    def publish_image(self, color_image, detection_image):
        # Publier l'image RGB originale
        msg_color = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        msg_color.header.stamp = self.get_clock().now().to_msg()
        self.image_publisher.publish(msg_color)

        # Publier l'image avec détection YOLO
        msg_detection = self.bridge.cv2_to_imgmsg(detection_image, "bgr8")
        msg_detection.header.stamp = self.get_clock().now().to_msg()
        self.detection_publisher.publish(msg_detection)

    def publish_markers(self, results, depth_frame, color_intrin):
        self.get_logger().info("publish_markers() called")
        if len(results.xyxy[0].cpu().numpy())==0:
            self.detected = False

        current_time = self.get_clock().now().to_msg()
        currentTime =rclpy.time.Time()
        if self.results is None:
            return

        # Convertir l'image annotée en BGR pour OpenCV
        detection_image = np.array(results.ims[0])
        detection_image_bgr = cv2.cvtColor(detection_image, cv2.COLOR_RGB2BGR)

        for i, (x1, y1, x2, y2, conf, cls) in enumerate(results.xyxy[0].cpu().numpy()):
            self.get_logger().info(f"Number of detections: {len(results.xyxy[0])}")

            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)

            # Obtenir la distance minimale dans la boîte englobante
            '''distance_min = 5000
            for x in range(int(x1), int(x2), 30):
                for y in range(int(y1), int(y2), 30):
                    distance = depth_frame.get_distance(x, y)
                    if distance_min > distance:
                        distance_min = distance'''
       
            depth = depth_frame.get_distance(x_center, y_center)
            dx, dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x_center, y_center], depth)
            distance = math.sqrt(dx**2 + dy**2 + dz**2)

            pose_from_camera = Pose()
            pose_from_camera.position.x = dx
            pose_from_camera.position.y = dy
            pose_from_camera.position.z = dz
            pose_from_camera.orientation.x = 0.0
            pose_from_camera.orientation.y = 0.0
            pose_from_camera.orientation.z = 0.0
            pose_from_camera.orientation.w = 1.0

            '''
            pose_from_camera.x = distance
            pose_from_camera.y = (x_center - 424) * distance / 848
            pose_from_camera.z = (y_center - 240) * distance / 480
            pose_from_camera.w = 1.0
            '''
      
            


            if cls == 0 and len(results.xyxy[0].cpu().numpy())-self.number_object > 0 :
                self.detected = True
                sound_msg = Sound()
                sound_msg.value = 1
                self.sound_publisher.publish(sound_msg)
            # Get Transformation
            try:
                stampedTransform = self.tf_buffer.lookup_transform(
                    'odom',
                    'laser_link',
                    currentTime
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.TransformException) as tex:
                self.get_logger().info( f'Could not transform: {tex}')
                return
            
            # Compute goal into local coordinates
            self.map_pose = tf2_geometry_msgs.do_transform_pose( pose_from_camera, stampedTransform )
            for pt in self.Pose_markers:

                sound_msg = Sound()
                sound_msg.value = 0
                self.sound_publisher.publish(sound_msg)
                
                print("\n\non est là\n\n\n")
                if euclidean_distance((self.map_pose.position.x, self.map_pose.position.y), pt) <= self.distance:
                    break

            self.Pose_markers.append((self.map_pose.position.x, self.map_pose.position.y))

            # Créer un marqueur
            marker = Marker()
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose=self.map_pose
            marker.header.frame_id = "odom"
            marker.header.stamp = current_time
            #marker.ns = "yolo_markers"
            marker.id = self.marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            self.get_logger().info(f"Marker created: ID={self.marker_id}, Position=({dx}, {dy}, {dz})")

            self.get_logger().info(f"Publishing marker ID={self.marker_id}")
            self.marker_publisher.publish(marker)
            #self.marker_array.markers.append(marker)
            self.marker_id=+1
        
        self.number_object = len(results.xyxy[0].cpu().numpy())
        #self.marker_publisher.publish(self.marker_array)
            
def euclidean_distance(a, b):
    return ((a[0]-b[0])**2+(a[1]-b[1])**2)**0.5


    '''def publish_distances(self, results, depth_frame, frames):
        color_intrin = frames.get_profile().as_video_stream_profile().get_intrinsics()
        for i, (x1, y1, x2, y2, conf, cls) in enumerate(results.xyxy[0].cpu().numpy()):
            if conf < confidence_threshold:  # Filtrer les faibles confiances
                continue

            x = int(x1)
            y = int(y1)
            w = int(x2 - x1)
            h = int(y2 - y1)

            # Calculer la profondeur du pixel central
            depth = depth_frame.get_distance(int(x + w / 2), int(y + h / 2))

            # Obtenir les coordonnées 3D
            dx, dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [int(x + w / 2), int(y + h / 2)], depth)

            # Calcul de l'angle theta
            a = 424 / (35 * 3.14159 / 180)  # Facteur de conversion pixels/radians
            theta = -(int(x + w / 2) - 424) / a
            distance = math.sqrt(dx**2 + dy**2 + dz**2)

            # Créer un message Pose
            pose = Pose()
            if distance**2 < 0.04:
                pose.position.x = 0.0
            else:
                pose.position.x = math.sqrt(distance**2 - 0.04)
                pose.position.y = distance * math.sin(theta)
                pose.position.z = 0.0
                pose.orientation.w = 1.0

            # Publier la pose
            self.distance_publisher.publish(pose)'''

if __name__ == '__main__':
    main()
