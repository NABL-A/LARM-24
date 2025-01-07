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
        infra_image_1 = np.asanyarray(infra_frame_1.get_data())
        infra_image_2 = np.asanyarray(infra_frame_2.get_data())

        # Publish images
        rsNode.publish_imgs(color_image)
        rsNode.publish_infra_imgs(infra_image_1, infra_image_2)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        color_image_resized = cv2.resize(color_image, (848, 480))
        depth_colormap_resized = cv2.resize(depth_colormap, (848, 480))
        infra_image_1_resized = cv2.resize(infra_image_1, (848, 480))
        infra_image_2_resized = cv2.resize(infra_image_2, (848, 480))

        infra_image_1_rgb = cv2.cvtColor(infra_image_1_resized, cv2.COLOR_GRAY2BGR)
        infra_image_2_rgb = cv2.cvtColor(infra_image_2_resized, cv2.COLOR_GRAY2BGR)

        top_row = np.hstack((color_image_resized, depth_colormap_resized))  
        bottom_row = np.hstack((infra_image_1_rgb, infra_image_2_rgb))
        all_images = np.vstack((top_row, bottom_row))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', all_images)
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
        self.infra_publisher_1 = self.create_publisher(Image, 'infrared_1', 10)
        self.infra_publisher_2 = self.create_publisher(Image, 'infrared_2', 10)
    
    def read_imgs(self):
        pass

    def publish_imgs(self, color_image):
        msg_image = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)

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
