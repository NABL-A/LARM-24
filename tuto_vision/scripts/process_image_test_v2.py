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
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose
import math

# Charger les deux modèles YOLOv5
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
        aligned_frames = rs.align(rs.stream.color).process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Conversion en tableaux numpy
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Appliquer YOLO pour la détection avec les deux modèles
        image_for_model = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results1 = model1(image_for_model)
        results1.render()

        results2 = model2(image_for_model)
        results2.render()

        detection_image1 = np.array(results1.ims[0])
        detection_image1 = cv2.cvtColor(detection_image1, cv2.COLOR_RGB2BGR)

        detection_image2 = np.array(results2.ims[0])
        detection_image2 = cv2.cvtColor(detection_image2, cv2.COLOR_RGB2BGR)

        # Publier les images, marqueurs et distances
        rs_node.publish_image(color_image, detection_image1, detection_image2)
        rs_node.publish_markers(results1, depth_frame)
        rs_node.publish_distances(results1, depth_frame, frames)

        # Afficher les résultats des deux modèles
        combined_detection_image = np.vstack((detection_image1, detection_image2))
        cv2.imshow('YOLO Detection (Model 1 & 2)', combined_detection_image)
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
        self.image_publisher1 = self.create_publisher(Image, 'yolo/detection_model1', 10)
        self.image_publisher2 = self.create_publisher(Image, 'yolo/detection_model2', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'yolo/markers', 10)
        self.distance_publisher = self.create_publisher(Pose, 'distance', 10)
        self.pipeline = pipeline

    def publish_image(self, color_image, detection_image1, detection_image2):
        # Publier l'image RGB originale
        msg_color = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        msg_color.header.stamp = self.get_clock().now().to_msg()
        
        # Publier les détections YOLO des deux modèles
        msg_detection1 = self.bridge.cv2_to_imgmsg(detection_image1, "bgr8")
        msg_detection1.header.stamp = self.get_clock().now().to_msg()
        self.image_publisher1.publish(msg_detection1)

        msg_detection2 = self.bridge.cv2_to_imgmsg(detection_image2, "bgr8")
        msg_detection2.header.stamp = self.get_clock().now().to_msg()
        self.image_publisher2.publish(msg_detection2)

    def publish_markers(self, results, depth_frame):
        marker_array = MarkerArray()
        current_time = self.get_clock().now().to_msg()

        for i, (x1, y1, x2, y2, conf, cls) in enumerate(results.xyxy[0].cpu().numpy()):
            if conf < confidence_threshold:
                continue

            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)
            depth = depth_frame.get_distance(x_center, y_center)

            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = current_time
            marker.ns = "yolo_markers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x_center
            marker.pose.position.y = y_center
            marker.pose.position.z = depth
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0

            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def publish_distances(self, results, depth_frame, frames):
        color_intrin = frames.get_profile().as_video_stream_profile().get_intrinsics()
        for i, (x1, y1, x2, y2, conf, cls) in enumerate(results.xyxy[0].cpu().numpy()):
            if conf < confidence_threshold:
                continue

            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)
            depth = depth_frame.get_distance(x_center, y_center)
            dx, dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x_center, y_center], depth)

            pose = Pose()
            pose.position.x = dx
            pose.position.y = dy
            pose.position.z = dz
            pose.orientation.w = 1.0

            self.distance_publisher.publish(pose)

if __name__ == '__main__':
    main()
