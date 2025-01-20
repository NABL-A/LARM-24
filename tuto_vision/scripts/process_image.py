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

# Charger le modèle YOLOv5
model = torch.hub.load(
    "/home/a2s4/yolov5/", 'custom', path='/home/a2s4/yolov5/weightsss/best.pt', source='local')

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
    rs_node = Realsense()

    while is_running:
        rclpy.spin_once(rs_node, timeout_sec=0.001)

        # Obtenir une image RGB et une image de profondeur
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Conversion en tableaux numpy
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Appliquer YOLO pour la détection
        image_for_model = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = model(image_for_model)
        results.render()
        detection_image = np.array(results.ims[0])
        detection_image = cv2.cvtColor(detection_image, cv2.COLOR_RGB2BGR)

        # Publier les images et les marqueurs
        rs_node.publish_image(color_image, detection_image)
        rs_node.publish_markers(results, depth_frame)
        rs_node.publish_distances(results, depth_frame)

        # Afficher les résultats
        cv2.namedWindow('YOLO Detection', cv2.WINDOW_AUTOSIZE)
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
    def __init__(self):
        super().__init__('realsense')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'realsense/color_image', 10)
        self.detection_publisher = self.create_publisher(Image, 'yolo/detection', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'yolo/markers', 10)
        self.distance_publisher = self.create_publisher(Pose, 'distance', 10)

    def publish_image(self, color_image, detection_image):
        # Publier l'image RGB originale
        msg_color = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        msg_color.header.stamp = self.get_clock().now().to_msg()
        self.image_publisher.publish(msg_color)

        # Publier l'image avec détection YOLO
        msg_detection = self.bridge.cv2_to_imgmsg(detection_image, "bgr8")
        msg_detection.header.stamp = self.get_clock().now().to_msg()
        self.detection_publisher.publish(msg_detection)

    def publish_markers(self, results, depth_frame):
        marker_array = MarkerArray()
        current_time = self.get_clock().now().to_msg()

        for i, (x1, y1, x2, y2, conf, cls) in enumerate(results.xyxy[0].cpu().numpy()):
            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)

            # Obtenir la distance du LiDAR
            distance = depth_frame.get_distance(x_center, y_center)

            # Créer un marqueur
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = current_time
            marker.ns = "yolo_markers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = distance
            marker.pose.position.y = (x_center - 424) * distance / 848  # Ajuster les dimensions
            marker.pose.position.z = (y_center - 240) * distance / 480  # Ajuster les dimensions
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def publish_distances(self, results, depth_frame):
        for i, (x1, y1, x2, y2, conf, cls) in enumerate(results.xyxy[0].cpu().numpy()):
            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)

            # Obtenir la distance du LiDAR
            distance = depth_frame.get_distance(x_center, y_center)

            # Créer un message Pose
            pose_msg = Pose()
            pose_msg.position.x = distance
            pose_msg.position.y = (x_center - 424) * distance / 848  # Ajuster les dimensions
            pose_msg.position.z = (y_center - 240) * distance / 480  # Ajuster les dimensions
            pose_msg.orientation.w = 1.0

            self.distance_publisher.publish(pose_msg)

if __name__ == '__main__':
    main()
