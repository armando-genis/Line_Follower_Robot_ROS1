#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool
from sensor_msgs.msg import Image
import cv2
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge 
import numpy as np

import math

class Horizontal_lines:
    def __init__(self):
        self.img = None
        self.img2 = None
        self.img3 = None
        self.horizontal_line_detected = Bool()

        rospy.init_node('horizontal_line_node')
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)

        self.image_line = rospy.Publisher('/video/horizontal_line', Image, queue_size=10)
        self.horizontal_line_publisher = rospy.Publisher('/horizontal_line_detected', Bool, queue_size=10)
        
        
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        

        self.bridge = CvBridge()

    def img_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.resize_image()

    def resize_image(self):
        # Changes that are made to get the lower part of the image
        height, width = self.img.shape[:2]
        start_row, end_row = int(height * .6), height
        start_col,end_col  = int(width * 0.15), int(width * 0.85)                
        cropped_img = self.img[start_row:end_row, start_col:end_col]
        

        gray = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
        # Applying a Gaussian blur
        gaussian_kernel_size = 9
        gaussian_blur_image = cv2.GaussianBlur(gray, (gaussian_kernel_size, gaussian_kernel_size), 0)

        # Applying median blur
        median_kernel_size = 9
        median_blur_image = cv2.medianBlur(gaussian_blur_image, median_kernel_size)
        self.img3 = median_blur_image
        
        edges = cv2.Canny(median_blur_image,50,150)

        self.img2 = edges
                   

    def detect_horizontal_lines(self):
        # Send an empty image to avoid errors
        if self.img2 is None:
            empty_image = Image()
            empty_image.header.stamp = rospy.Time.now()
            empty_image.height = 0
            empty_image.width = 0
            empty_image.encoding = "mono8"
            empty_image.is_bigendian = False
            empty_image.step = 0
            empty_image.data = b""
            return empty_image, False
        
        image_to_use = self.img2
        lines = cv2.HoughLinesP(image_to_use, 1, np.pi/180, 20, minLineLength = 9, maxLineGap = 9)
        # If any line is detected
        count = 0
        bool = False
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if abs(y1 - y2) < 10:  # If line is horizontal
                    count += 1
                    cv2.line(self.img3, (x1, y1), (x2, y2), (0, 255, 255), 2)

            if count > 5:
                bool = True
            

        ros_image = self.bridge.cv2_to_imgmsg(self.img3, encoding="mono8")
        return ros_image, bool

        
    def timer_callback(self, timer):
        ros_image,bool = self.detect_horizontal_lines()


        self.horizontal_line_publisher.publish(bool)
        self.image_line.publish(ros_image)
        
        


    def run(self):
        if not rospy.is_shutdown():
            rospy.spin()




if __name__ == '__main__':

    
    try:
        s = Horizontal_lines()
        s.run()
    except rospy.ROSInterruptException:
        pass
