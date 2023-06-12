#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool, UInt8
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge 
import numpy as np
from operator import itemgetter
import colorsys

import math

class Traffic_light:
    def __init__(self):
        self.img = None
        self.resized_image = None
        self.preprocessed_image = None
        self.analysis_image = None


        # Semaphore state:
        # 0 - No semaphore
        # 1 - Stop (red)
        # 2 - Caution (yellow)
        # 3 - Go (green)
        self.semaphore_state = None
        self.semaphore_previous_state = None

        rospy.init_node('color_traffic')
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)

        self.image_traffic_detection = rospy.Publisher('/video/traffic_lights_detection', Image, queue_size=10)
        # self.flag_pub_red = rospy.Publisher('/flag_red', Bool, queue_size=10)
        # self.flag_pub_yellow = rospy.Publisher('/flag_yellow', Bool, queue_size=10)
        # self.flag_pub_traffic_light = rospy.Publisher('/flag_green', Bool, queue_size=10)
        self.analysis_image_publisher = rospy.Publisher('/semaphore_detection/analysis_image', Image, queue_size=10)

        self.semaphore_state_publisher = rospy.Publisher('/semaphore_detection/semaphore_state', UInt8, queue_size=10)

        
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        self.bridge = CvBridge()

    def img_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.resize_image()

    def resize_image(self):
        if self.img is not None:
            height, width = self.img.shape[:2]
            start_row, end_row = int(height * 0.05), int(height * 0.60)
            start_col, end_col = 0, int(width * 0.2)  # Adjusted for left part extraction
            start_col2, end_col2 = int(width * 0.67), int(width * 0.90)  # Adjusted for left part extraction

            cropped_img = self.img[start_row:end_row, start_col:end_col]
            cropped_img2 = self.img[start_row:end_row, start_col2:end_col2]

            # roi1 = cropped_img[:,:int(cropped_img.shape[1]*0.150)] 
            # roi2 = cropped_img[:,int(cropped_img.shape[1]*0.950):]


            roiComplete = cv2.hconcat([cropped_img,cropped_img2])

            self.resized_image = roiComplete

            # Converting the image to grayscale
            gray_image = cv2.cvtColor(roiComplete, cv2.COLOR_BGR2GRAY)

            # Applying a Gaussian blur
            gaussian_kernel_size = 9
            gaussian_blur_image = cv2.GaussianBlur(gray_image, (gaussian_kernel_size, gaussian_kernel_size), 0)

            # Applying median blur
            median_kernel_size = 9
            median_blur_image = cv2.medianBlur(gaussian_blur_image, median_kernel_size)

            self.preprocessed_image = median_blur_image

    def semaphore_detection(self, image):
        canny_image = cv2.Canny(image, 75, 200)

        if canny_image.shape[0] == 0:
            rospy.logwarn("Canny image has zero height. Skipping conversion to Image message.")
            return False
        
        circles = cv2.HoughCircles(canny_image, cv2.HOUGH_GRADIENT, dp=1, minDist=3, param1= 15, param2=15, minRadius=3, maxRadius=16)
        biggest_circle = None
        if circles is not None:
            biggest_circle = max(circles[0,:], key=itemgetter(2))
            cv2.circle(self.resized_image,(biggest_circle[0],biggest_circle[1]),biggest_circle[2],(0,255,0),5)

        else:
            if self.semaphore_previous_state != 0:
                self.semaphore_state = 0
                self.semaphore_state_publisher.publish(self.semaphore_state)

        return True

    def timer_callback(self, timer):
        if self.preprocessed_image is not None:
            self.semaphore_detection(self.preprocessed_image)
            print(self.semaphore_state)
            self.image_traffic_detection.publish(self.bridge.cv2_to_imgmsg(self.preprocessed_image, encoding="mono8"))

            if self.analysis_image is not None:
                self.analysis_image_publisher.publish(self.bridge.cv2_to_imgmsg(self.resized_image, encoding="bgr8"))


    def run(self):
        if not rospy.is_shutdown():
            rospy.spin()




if __name__ == '__main__':

    try:
        s = Traffic_light()
        s.run()
    except rospy.ROSInterruptException:
        pass