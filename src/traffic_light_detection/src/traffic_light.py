#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool
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
        self.img2 = None
        self.window_size = 12
        self.window = np.zeros(self.window_size)
        self.mode_sign = 0
        self.semaphore_state = None
        self.semaphore_previous_state = None
        self.prevYCenter = 0

        self.colorPrev = 0
        self.ratio = None


        self.biggest_circle_y = 0
        self.biggest_circle_radius = 0

        self.img_publish = None
        rospy.init_node('color_traffic')
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)

        self.image_traffic_detection = rospy.Publisher('/video/traffic_lights_detection', Image, queue_size=10)
        self.image_traffic_cropt = rospy.Publisher('/video/traffic_lights_cropt', Image, queue_size=10)

        self.flag_pub_red = rospy.Publisher('/flag_red', Bool, queue_size=10)
        self.flag_pub_traffic_light = rospy.Publisher('/flag_green', Bool, queue_size=10)
        
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        self.bridge = CvBridge()

    def img_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.resize_image()


# * ================= Mode filter =================================================================

    def rolling_mode_filter(self,number_list):
        self.window = np.roll(self.window, -1)
        self.window[-1] = number_list
        values, counts = np.unique(self.window, return_counts=True)
        mode_index = np.argmax(counts)
        self.mode_sign = values[mode_index]

#  =======================================================================

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



            smoothed_img = cv2.GaussianBlur(roiComplete, (5, 5), 0)


            self.img2 = smoothed_img
            self.img_publish = self.bridge.cv2_to_imgmsg(self.img2, encoding="bgr8")


    def detect_red(self, cropped_image):
        if cropped_image is None or cropped_image.size == 0:
            rospy.logwarn("Empty or invalid image received in detect_red")
            return False
        hsv_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
        
        # Define lower and upper bounds for red color in HSV color space
        lower_red = np.array([120,110,130], dtype="uint8")
        upper_red = np.array([190,255,255], dtype="uint8")
        mask0 = cv2.inRange(hsv_image, lower_red, upper_red)
        
        
        # Check if any red pixels are present
        if cv2.countNonZero(mask0) > 0:
            return True
        else:
            return False
        
    def detect_green(self, cropped_image):
        hsv_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
        
        lower_red = np.array([85,100,255], dtype="uint8")
        upper_red = np.array([25,0,100], dtype="uint8")
        mask0 = cv2.inRange(hsv_image, upper_red, lower_red)
 
        # Check if any red pixels are present
        if cv2.countNonZero(mask0) > 0:
            return True
        else:
            return False
        

    

    def procesamiento4(self):
        if self.img2 is None:
            rospy.loginfo("Image is None")
            return None
        
        gray = cv2.cvtColor(self.img2, cv2.COLOR_BGR2GRAY)

        # Applying a Gaussian blur
        gaussian_kernel_size = 9
        gaussian_blur_image = cv2.GaussianBlur(gray, (gaussian_kernel_size, gaussian_kernel_size), 0)

        # Applying median blur
        median_kernel_size = 9
        median_blur_image = cv2.medianBlur(gaussian_blur_image, median_kernel_size)

        # _, thresh = cv2.threshold(median_blur_image, 100, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(median_blur_image, 75, 200)

        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=3, param1= 25, param2=20, minRadius=7, maxRadius=35)

        biggest_circle = None


        if circles is not None:

            circles = np.uint16(np.around(circles))
            biggest_circle = max(circles[0,:], key=itemgetter(2))
            cv2.circle(self.img2,(biggest_circle[0],biggest_circle[1]),biggest_circle[2],(0,255,0),1)

            if biggest_circle is not None:
                x, y, r = biggest_circle[0], biggest_circle[1], biggest_circle[2] + 7
                cropped_image = self.img2[y - r:y + r, x - r:x + r]

                if cropped_image.shape[0] == 0:
                    rospy.logwarn("Analysis image has zero height. Skipping conversion to Image message.")
                    return False
                
                cropped_image_to_p  = self.bridge.cv2_to_imgmsg(cropped_image, encoding="bgr8")

                self.image_traffic_cropt.publish(cropped_image_to_p)



                # Detect red color in cropped image
                red_detected = self.detect_red(cropped_image)
                
                if red_detected:
                    self.flag_pub_red.publish(True)
                    self.flag_pub_traffic_light.publish(False)  

                else:
                    self.flag_pub_red.publish(False)
                    self.flag_pub_traffic_light.publish(True)  

    

        else:
            self.flag_pub_traffic_light.publish(False)            
            self.flag_pub_red.publish(False)

 

                    

        image_ros = self.bridge.cv2_to_imgmsg(self.img2, encoding="bgr8")
        
        return image_ros
        

    def timer_callback(self, timer):

        if self.img_publish is not None:

            circles = self.procesamiento4()




            self.image_traffic_detection.publish(circles)



    def run(self):
        if not rospy.is_shutdown():
            rospy.spin()




if __name__ == '__main__':

    try:
        s = Traffic_light()
        s.run()
    except rospy.ROSInterruptException:
        pass