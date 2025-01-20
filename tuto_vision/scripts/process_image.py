#!/usr/bin/env python3

import os
import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2, rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

import torch
#from PIL import Image


# Chemin vers le répertoire local YOLOv5


# Charger le modèle YOLOv5
model = torch.hub.load("/home/a2s4/yolov5/", 'custom', path='/home/a2s4/yolov5/weightsss/best.pt', source='local')


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
        green_image = color_image
        imageTestModel=color_image
        imageTestModel=cv2.cvtColor(imageTestModel, cv2.COLOR_BGR2RGB)
        results = model(imageTestModel)
        results.render()
        #results.imgs
        #results.show()
        imageImageImage=np.array(results.ims[0])

        imageImageImage = cv2.cvtColor(imageImageImage, cv2.COLOR_RGB2BGR)  # Convertir RGB -> BGR pour OpenCV

        

        height1, width1 = green_image.shape[:2]

        new_width1= 400  # Par exemple, changer la largeur à 400 pixels
        # Calculer la nouvelle hauteur en gardant le ratio
        new_height1 = int((new_width1 / width1) * height1)
        green_image=cv2.resize(green_image,(new_width1,new_height1))
        ####green_image=cv2.resize(green_image,(848,480))
        hsv_image = cv2.cvtColor(green_image, cv2.COLOR_BGR2HSV)
        #lower_green = np.array([20,50,50])#DEBASE
        lower_green = np.array([20,40,40])
        #min de h 22
        #min de s1 120
        #min de s2 60
        #min de v a 50
        
        #le max de h à 90
        #max h 240
        #max s à 255
        #upper_green = np.array([85,255,255])#DEBASE
        upper_green = np.array([100,255,255])
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
    
        ##################
        #mask_green = cv2.equalizeHist(gray_image)
        #mask_green=cv2.cvtColor(green_image, cv2.COLOR_BGR2GRAY)
        #mask_green = cv2.equalizeHist(mask_green)
        #_, mask_green = cv2.threshold(mask_green, 50, 255, cv2.THRESH_BINARY)
        mask_green = cv2.bitwise_and(green_image, green_image, mask=mask_green)
        mask_green=cv2.cvtColor(mask_green, cv2.COLOR_BGR2GRAY)
        mask_green = cv2.equalizeHist(mask_green)
        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #all_images=np.vstack((mask_green, mask_green))
        """cv2.imshow('RealSense', mask_green)
        cv2.waitKey(1)"""

        """

        kernel = np.ones((5, 5), np.uint8)
        mask_green = cv2.erode(mask_green, kernel, iterations=1)
        im_green = cv2.bitwise_and(green_image, green_image, mask=mask_green)
        """
        #################################################
        #image1 = cv2.imread("IMG_20250110_114408.jpg", cv2.IMREAD_COLOR)
        #
        image2 = cv2.imread("/home/a2s4/ros_space/A2-S4/tuto_vision/scripts/Fantome.jpg", cv2.IMREAD_COLOR)
        """if image2 is None:
            print("Erreur : L'image n'a pas pu être chargée.")
        else:
            print("Image chargée avec succès.")"""
        
        height2, width2 = image2.shape[:2]

        new_width2= 400  # Par exemple, changer la largeur à 400 pixels
        # Calculer la nouvelle hauteur en gardant le ratio
        new_height2 = int((new_width2 / width2) * height2)
        image2=cv2.resize(image2,(new_width2,new_height2))

        ###########image2=cv2.resize(image2,(848,480))

        lower_green2 = np.array([30,50,50])
        upper_green2 = np.array([80,255,255])

        #lower_green = np.array([20, 50, 50])  # Borne inférieure pour le vert (HSV)
        #upper_green = np.array([80, 255, 255])  # Borne supérieure pour le vert (HSV)


        #image1_hsv = cv2.cvtColor(image1, cv2.COLOR_BGR2HSV)
        # 2. Appliquer un masque pour isoler l'objet vert
        #mask1 = cv2.inRange(image1_hsv, lower_green, upper_green)  # Créer le masque
        #mask = cv2.blur(mask, (5, 5))
        #image1= cv2.bitwise_and(image1, image1, mask=mask1)

        image2_hsv = cv2.cvtColor(image2, cv2.COLOR_BGR2HSV)
        # 2. Appliquer un masque pour isoler l'objet vert
        mask2 = cv2.inRange(image2_hsv, lower_green2, upper_green2)  # Créer le masque
        #mask = cv2.blur(mask, (5, 5))
        image2= cv2.bitwise_and(image2, image2, mask=mask2)
        ########
        
        # Threshold the images to binary (assuming binary image)
        #_, thresh1 = cv2.threshold(image1, 127, 255, cv2.THRESH_BINARY)
        #_, thresh2 = cv2.threshold(image2, 127, 255, cv2.THRESH_BINARY)
        _, mask_green = cv2.threshold(mask_green, 20, 255, cv2.THRESH_BINARY)
        # Find contours in both images
        contours1, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours2, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        imax=0
        perimeterMax=0
        for i,contour in enumerate(contours2):
            perimeter = cv2.arcLength(contour, closed=True)
            if perimeter>perimeterMax:
                perimeterMax=perimeter
                imax=i
        #IndiceGrandContourIm2= contours2.index(max(contours2))
        # Calculate Hu Moments for both contours (for simplicity, we compare the first contour of each image)
        #hu_moments1 = cv2.HuMoments(cv2.moments(contours1[309])).flatten()
        hu_moments2 = cv2.HuMoments(cv2.moments(contours2[imax])).flatten()

        hu_moments_list = []
        for i,contour in enumerate(contours1):
            perimeter = cv2.arcLength(contour, closed=True)
            if perimeter >100:
                cv2.drawContours(green_image, contours1, i, (0, 255, 0), 3)
                #print(i)
                #hu_moments1 = cv2.HuMoments(cv2.moments(contours1[309])).flatten()
                hu_moments_list.append(cv2.HuMoments(cv2.moments(contours1[i])).flatten())
        hu_moments_array = np.array(hu_moments_list)
        
        
        #print(contours1)
        

        #output_image = cv2.cvtColor(image2, cv2.COLOR_GRAY2BGR)  # Convertir l'image en couleur pour la visualisation
        
        cv2.drawContours(image2, contours2, imax, (0, 255, 0), 3)
        
        #print(contours2)
        """
        cv2.imshow('Contours', image2)
        cv2.waitKey(0)
        cv2.destroyAllWindows()"""
        #cv2.drawContours(image1, contours1, 309, (0, 255, 0), 3)
        """
        cv2.imshow('Contours', green_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()"""
        # Compare Hu Moments (lower distance means more similarity)
        #distance = np.sum(np.abs(hu_moments1 - hu_moments2))
        distances = []
        for hu_moment in hu_moments_array:
            # Calculer la distance entre les Hu Moments
            distance = np.sum(np.abs(hu_moment - hu_moments2))
            distances.append(distance)
        #print(f"Contour similarity (Hu Moments): {distance}")
        nb_Fantome=0
        for i, distance in enumerate(distances):
            #print(f"Distance avec le contour {i+1} de image1 : {distance}")
            if distance<0.01:#de base 0.5
                nb_Fantome+=1
        #print(f"Dans l'image 1, il y a donc {nb_Fantome} fantomes")


        #################################################







        """

        binary_image = cv2.cvtColor(im_green, cv2.COLOR_BGR2GRAY)
        _,binary_image=cv2.threshold(binary_image,20,255,cv2.THRESH_BINARY)
        binary_image=cv2.erode(binary_image, kernel, iterations=1)
        kernel2=cv2.getStructuringElement(cv2.MORPH_RECT,(25,25))
        binary_image=cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE,kernel2)
        

        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        num_objects = len(contours)
        print(f"Nombre d'objets détectés : {num_objects}")
        """
        infra_image_1 = np.asanyarray(infra_frame_1.get_data())
        infra_image_2 = np.asanyarray(infra_frame_2.get_data())

        # Publish images
        rsNode.publish_imgs(color_image,green_image)
        rsNode.publish_infra_imgs(infra_image_1, infra_image_2)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        color_image_resized = cv2.resize(color_image, (848, 480))
        #green_image_resized = cv2.resize(im_green, (848, 480))
        depth_colormap_resized = cv2.resize(depth_colormap, (848, 480))
        infra_image_1_resized = cv2.resize(infra_image_1, (848, 480))
        infra_image_2_resized = cv2.resize(infra_image_2, (848, 480))
        #binary = cv2.resize(binary_image, (848, 480))

        infra_image_1_rgb = cv2.cvtColor(infra_image_1_resized, cv2.COLOR_GRAY2BGR)
        infra_image_2_rgb = cv2.cvtColor(infra_image_2_resized, cv2.COLOR_GRAY2BGR)

        #top_row = np.hstack((color_image_resized, depth_colormap_resized))  
        #bottom_row = np.hstack((green_image, image2))
        #all_images = np.vstack((top_row, bottom_row))
        #all_images=np.vstack((green_image, image2))
        all_images=imageImageImage
        # Show images
        #image_with_contours = green_image_resized.copy()
        #cv2.drawContours(image_with_contours, contours, -1, (0,255,0), 3)
        
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