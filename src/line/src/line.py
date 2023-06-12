#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool
from sensor_msgs.msg import Image
import cv2
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge 
import numpy as np


import math

class Lines:
    def __init__(self):
        self.img = None
        self.img2 = None
        self.img3 = None
        self.flag_white = False
        
        # self.img_publish = None
        self.line_detection_twist = Twist()
        
        rospy.init_node('lines_node')
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)

        self.image_line = rospy.Publisher('/video/line', Image, queue_size=10)
        self.robot_move = rospy.Publisher('/cmd_vel/line_planning', Twist, queue_size=10)
        self.flag_white_space = rospy.Publisher('/flag_white_space', Bool, queue_size=10)
        
        
        
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        

        self.bridge = CvBridge()

    def img_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.resize_image()

    def resize_image(self):
        # Changes that are made to get the lower part of the image
        height, width = self.img.shape[:2]
        start_row, end_row = int(height * .40), int(height * 0.95)
        start_col,end_col  = int(width * 0.1), int(width * 0.9)                
        cropped_img = self.img[start_row:end_row, start_col:end_col] 
        smoothed_img = cv2.GaussianBlur(cropped_img, (5, 5), 0)

        gray = cv2.cvtColor(smoothed_img, cv2.COLOR_BGR2GRAY)
        # Increase contrast
        alpha = 1.9  # Contrast control (1.0 - 3.0)
        adjusted_img = cv2.convertScaleAbs(gray, alpha=alpha, beta=1)
        self.img2 = adjusted_img
                   
        
#  ================================= START CENTER =========================================
    def processing_center(self):
        
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
            return empty_image
            
        # Dibujo de la linea
        gray_img = self.img2
        height, width = gray_img.shape[:2]    #512    
        mid_x = int(width / 2)
        
        mid_y = 350
        left_line = 407
        right_line = 617


        #Group Left
        region_y = 350
        region_left_init = 114
        region_left_end = 374 #414
        region_left_part = gray_img[region_y:region_y+1, region_left_init:region_left_end]
        region_left = self.check_region(region_left_part)

        # Group Right
        region_right_init = 650 #610
        region_right_end = 910
        region_right_part = gray_img[region_y:region_y+1, region_right_init:region_right_end]
        region_right = self.check_region(region_right_part)
        
        #Region square center
        # mid_y = 390
        region_center_init_x = 412 #412
        region_center_end_x = 612 #612
        region_center_init_y = mid_y - 60
        region_center_end_y = mid_y
        
        region_center_part = gray_img[mid_y:mid_y+1, region_center_init_x:region_center_end_x]
        center_point = self.check_center(region_center_part)

        # region_upper 
        region_y_upper = 230
        region_left_init_upper = 412
        region_left_end_upper = 612 #414
        region_upper_part = gray_img[region_y_upper:region_y_upper+1, region_left_init_upper:region_left_end_upper]
        region_upper = self.check_region(region_upper_part)   
        
        
        
        
        self.planning(gray_img[mid_y,left_line],gray_img[mid_y,right_line],region_left, region_right,center_point,region_upper)
        # Individual 
        gray_img = cv2.line(gray_img, (left_line, mid_y-10), (right_line, mid_y-10), 0, 5)
        
        cv2.rectangle(gray_img, (region_center_init_x, region_center_init_y), (region_center_end_x, region_center_end_y), (0, 255, 0), 2)  # (0, 255, 0) represents the color in BGR format, (0, 255, 0) is green, and 2 is the thickness

        gray_img = cv2.line(gray_img, (region_left_init, region_y), (region_left_end, region_y), (0, 255, 255), 5)
        gray_img = cv2.line(gray_img, (region_right_init, region_y), (region_right_end, region_y), (0, 255, 255), 5)
        gray_img = cv2.line(gray_img, (region_left_init_upper, region_y_upper), (region_left_end_upper, region_y_upper), (0, 255, 255), 5)
        cv2.circle(gray_img, (mid_x, mid_y), 5, 0, -1)
        image = self.bridge.cv2_to_imgmsg(gray_img, encoding="mono8")
        
        return image
    
    def planning(self,left,right,left_region,right_region,center_region,region_upper):

        # print(left_region,self.following_pad(left),self.following_pad(right),right_region,center_region,region_upper)
        # Planner to see if the line is centered
        # if center_region or region_upper:
        #     #Is in the pad
        #     # print("Forward")
        #     self.forward()
        # elif self.following_pad(left) and not self.following_pad(right):
        #     #Go left
        #     # print("left")
        #     self.left()
        # elif not self.following_pad(left) and self.following_pad(right):
        #     #Go right
        #     # print("right")
        #     self.right()
        # elif left_region and not right_region and not self.following_pad(right) and (not center_region or center_region):
        #     #Go left
        #     # print("left fast")
        #     self.left_fast()
        # elif left_region and right_region and not self.following_pad(left) and not self.following_pad(right) and (not center_region or center_region):
        #     #excetion
        #     # print("Forward")
        #     self.forward()
        # elif not left_region and right_region and not self.following_pad(left) and (not center_region or center_region):
        #     #Go right
        #     # print("right fast")
        #     self.right_fast()
        # elif not self.following_pad(left) and not self.following_pad(right):
        #     # print("Stop")  
        #     self.stop()  
        # elif self.following_pad(left) and self.following_pad(right) and center_region and self.following_pad(right) and center_region:
        #     self.forward()
        
        self.flag_white = False
        if center_region or region_upper:
            self.forward()
        elif self.following_pad(left) and not self.following_pad(right) and not left_region and not right_region:
            self.left()
        elif not self.following_pad(left) and self.following_pad(right) and not left_region and not right_region:
            self.right()
        elif left_region and not right_region:
            self.left_fast()
        elif not left_region and right_region:
            self.right_fast()
        elif not self.following_pad(left) and not self.following_pad(right) and not region_upper and not left_region and not right_region:
            self.flag_white = True
            self.stop()
            
   
    def check_region(self, region):
        return np.any(region < 200)

    def check_center(self, region):
        threshold = 0.30 * region.size
        count = np.sum(region < 200)
    
        if count >= threshold:
            return True
        else:
            return False
        
        
    def following_pad(self,val):
        # Return if the value is higher or lower than the threshold 
        return val < 200
        
    def forward(self):
        data = self.line_detection_twist
        data.linear.x = 0.07
        data.angular.z = 0.0
        self.robot_move.publish(data)

    def left(self):
        data = self.line_detection_twist
        data.linear.x = 0.07
        data.angular.z = 0.085
        self.robot_move.publish(data)
        
    def left_fast(self):
        data = self.line_detection_twist
        data.linear.x = 0.07
        data.angular.z = 0.099
        self.robot_move.publish(data)

    def right(self):
        data = self.line_detection_twist
        data.linear.x = 0.07
        data.angular.z = -0.085
        self.robot_move.publish(data)
        
    def right_fast(self):
        data = self.line_detection_twist
        data.linear.x = 0.07
        data.angular.z = -0.099
        self.robot_move.publish(data)
        
    def stop(self):        
        zero_twist = self.line_detection_twist
        zero_twist.linear.x = 0.0
        zero_twist.angular.z = 0.0
        self.robot_move.publish(zero_twist)
        
# =========================== END CENTER ===========================
    
        
    def timer_callback(self, timer):
        image = self.processing_center()
        if image is not None:
            self.image_line.publish(image)
            self.flag_white_space.publish(self.flag_white)
        
        


    def run(self):
        if not rospy.is_shutdown():
            rospy.spin()




if __name__ == '__main__':
    
    try:
        s = Lines()
        s.run()
    except rospy.ROSInterruptException:
        pass
