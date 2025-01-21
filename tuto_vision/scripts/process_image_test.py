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

# Charger le modèle YOLOv5
model = torch.hub.load(
    "/home/a2s4/yolov5/", 'custom', path='/home/a2s4/yolov5/weightsss/best.pt', source='local'
)
model.conf = 0.5

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
        distance = []
        x_center = 0
        y_center = 0
        rclpy.spin_once(rs_node, timeout_sec=0.001)

        # Obtenir les images alignées
        frames = pipeline.wait_for_frames()
        aligned_frames = rs_node.align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            print("Frames not available")
            continue

        # Conversion de l'image couleur
        color_image_bgr = np.asanyarray(color_frame.get_data())

        # Vérifier si l'image a le bon nombre de canaux
        if len(color_image_bgr.shape) == 2:  # Image en niveaux de gris
            print("Converting grayscale image to BGR")
            color_image_bgr = cv2.cvtColor(color_image_bgr, cv2.COLOR_GRAY2BGR)

        # Convertir en HSV
        color_image_hsv = cv2.cvtColor(color_image_bgr, cv2.COLOR_BGR2HSV)

        # Obtenir les paramètres intrinsèques
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

        # Utiliser le centre de l'image pour calculer la distance
        

        # Appliquer YOLO pour la détection
        color_image_rgb = cv2.cvtColor(color_image_bgr, cv2.COLOR_BGR2RGB)
        results = model(color_image_rgb)
        results.render()

        annotated_images = []
        print("1")
        for img_idx in range(len(results.ims)):
            print("2")
            detection_image = np.array(results.ims[img_idx])  # Extraire l'image annotée
            detection_image_bgr = cv2.cvtColor(detection_image, cv2.COLOR_RGB2BGR)  # Convertir en BGR pour OpenCV

            # Parcourir les détections associées à cette image
            for i, (x1, y1, x2, y2, conf, cls) in enumerate(results.xyxy[img_idx].cpu().numpy()):
                print("3")
                if conf < confidence_threshold:  # Filtrer les faibles confiances
                    continue

                x_center = int((x1 + x2) / 2)
                y_center = int((y1 + y2) / 2)

                # Calculer la distance à partir du centre de la boîte englobante
                depth = depth_frame.get_distance(x_center, y_center)
                dx, dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x_center, y_center], depth)
                print("4")
                a = math.sqrt(dx**2 + dy**2 + dz**2)

                # Annoter l'image avec la distance
                cv2.putText(
                    print("5")
                    detection_image_bgr,
                    f"Distance: {a:.2f} meters",
                    (x_center - 50, y_center - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

            annotated_images.append(detection_image_bgr)

        # Afficher les résultats
        for img_idx, annotated_img in enumerate(annotated_images):
            print("6")
            cv2.imshow(f"YOLO Detection {img_idx}", annotated_img)

        # Afficher la carte de profondeur colorisée
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(np.asanyarray(depth_frame.get_data()), alpha=0.03),
            cv2.COLORMAP_JET,
        )

        cv2.waitKey(1)

    # Arrêter le pipeline
    print("Stopping...")
    pipeline.stop()

    # Fermer proprement ROS2
    rs_node.destroy_node()
    rclpy.shutdown()


class Realsense(Node):
    def __init__(self, pipeline):
        super().__init__('realsense')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'realsense/color_image', 10)
        self.detection_publisher = self.create_publisher(Image, 'yolo/detection', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'yolo/markers', 10)
        self.distance_publisher = self.create_publisher(Pose, 'distance', 10)
        self.pipeline = pipeline

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)


if __name__ == '__main__':
    main()
