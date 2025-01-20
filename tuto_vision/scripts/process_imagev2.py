#!/usr/bin/env python3

import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2, rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

def signalInteruption(signum, frame):
    global isOk
    print("\nCtrl-c pressed")
    isOk = False

def calculate_centroid(contour):
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy
    else:
        return None


# Node processes:
def main(args=None):
    global isOk
    isOk = True
    
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    print(f"Connect: {device_product_line}")
    found_rgb = False
    for s in device.sensors:
        print("Name:" + s.get_info(rs.camera_info.name))
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True

    if not found_rgb:
        print("Depth camera required !!!")
        exit(0)

    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
    
    # Enable infrared streams
    config.enable_stream(rs.stream.infrared, 1, 848, 480, rs.format.y8, 60)
    config.enable_stream(rs.stream.infrared, 2, 848, 480, rs.format.y8, 60)

    signal.signal(signal.SIGINT, signalInteruption)

    # Start streaming
    pipeline.start(config)

    count = 1
    refTime = time.process_time()
    freq = 60

    sys.stdout.write("-")
    
    rclpy.init(args=args)
    rsNode = Realsense()

    while isOk:
        rsNode.read_imgs()
        rclpy.spin_once(rsNode, timeout_sec=0.001)

        # Wait for a coherent tuple of frames: depth, color, and accel
        frames = pipeline.wait_for_frames()

        color_frame = frames.first(rs.stream.color)
        depth_frame = frames.first(rs.stream.depth)
        
        # Get the infrared frames by index
        infra_frame_1 = frames.get_infrared_frame(1)
        infra_frame_2 = frames.get_infrared_frame(2)

        if not (depth_frame and color_frame and infra_frame_1 and infra_frame_2):
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        green_image = color_image

        hsv_image = cv2.cvtColor(green_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35,50,50])
        upper_green = np.array([85,255,255])
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)


        kernel = np.ones((5, 5), np.uint8)
        mask_green = cv2.erode(mask_green, kernel, iterations=1)
        im_green = cv2.bitwise_and(green_image, green_image, mask=mask_green)



        binary_image = cv2.cvtColor(im_green, cv2.COLOR_BGR2GRAY)
        _,binary_image=cv2.threshold(binary_image,20,255,cv2.THRESH_BINARY)
        binary_image=cv2.erode(binary_image, kernel, iterations=1)
        kernel2=cv2.getStructuringElement(cv2.MORPH_RECT,(25,25))
        binary_image=cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE,kernel2)

        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        num_objects = len(contours)
        print(f"Nombre d'objets détectés : {num_objects}")

        if contours:
            # Trouver le plus grand contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculer le centroïde
            centroid = calculate_centroid(largest_contour)

        if centroid:
            cx, cy = centroid
            print(f"Centroïde de la structure verte : ({cx}, {cy})")

            # Obtenir la distance en profondeur au centroïde
            depth_at_centroid = depth_image[cy, cx]
            print(f"Distance au centroïde : {depth_at_centroid:.3f} mètres")


        infra_image_1 = np.asanyarray(infra_frame_1.get_data())
        infra_image_2 = np.asanyarray(infra_frame_2.get_data())

        # Publish images
        rsNode.publish_imgs(color_image,im_green)
        rsNode.publish_infra_imgs(infra_image_1, infra_image_2)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        color_image_resized = cv2.resize(color_image, (848, 480))
        green_image_resized = cv2.resize(im_green, (848, 480))
        depth_colormap_resized = cv2.resize(depth_colormap, (848, 480))
        infra_image_1_resized = cv2.resize(infra_image_1, (848, 480))
        infra_image_2_resized = cv2.resize(infra_image_2, (848, 480))
        #binary = cv2.resize(binary_image, (848, 480))

        infra_image_1_rgb = cv2.cvtColor(infra_image_1_resized, cv2.COLOR_GRAY2BGR)
        infra_image_2_rgb = cv2.cvtColor(infra_image_2_resized, cv2.COLOR_GRAY2BGR)

        #top_row = np.hstack((color_image_resized, depth_colormap_resized))  
        #bottom_row = np.hstack((binary, infra_image_1_rgb))
        #all_images = np.vstack((top_row, bottom_row))

        # Show images
        image_with_contours = green_image_resized.copy()
        #cv2.drawContours(image_with_contours, contours, -1, (0,255,0), 3) 

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', binary_image)
        cv2.waitKey(1)

        # Frequency:
        if count == 10:
            newTime = time.process_time()
            freq = 10 / ((newTime - refTime))
            refTime = newTime
            count = 0
        count += 1
    
    # Stop streaming
    print("Ending...")
    rsNode.pipeline.stop()
    # Clean end
    rsNode.destroy_node()
    rclpy.shutdown()

# Realsense Node:
class Realsense(Node):
    def __init__(self, fps=60):
        super().__init__('realsense')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'realsense/color_image', 10)
        self.green_detection_publisher = self.create_publisher(Image, 'green_detection', 10)

        self.infra_publisher_1 = self.create_publisher(Image, 'infrared_1', 10)
        self.infra_publisher_2 = self.create_publisher(Image, 'infrared_2', 10)
    
    def read_imgs(self):
        pass

    def publish_imgs(self, color_image, green_image):
        msg_image = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)

        green_detection = self.bridge.cv2_to_imgmsg(green_image, "bgr8")
        green_detection.header.stamp = self.get_clock().now().to_msg()
        green_detection.header.frame_id = "image with green"
        self.green_detection_publisher.publish(green_detection)

    def publish_infra_imgs(self, infra_image_1, infra_image_2):
        msg_infra_1 = self.bridge.cv2_to_imgmsg(infra_image_1, "mono8")
        msg_infra_1.header.stamp = self.get_clock().now().to_msg()
        msg_infra_1.header.frame_id = "infra_1"
        self.infra_publisher_1.publish(msg_infra_1)

        msg_infra_2 = self.bridge.cv2_to_imgmsg(infra_image_2, "mono8")
        msg_infra_2.header.stamp = self.get_clock().now().to_msg()
        msg_infra_2.header.frame_id = "infra_2"
        self.infra_publisher_2.publish(msg_infra_2)

if __name__ == '__main__':
    main()