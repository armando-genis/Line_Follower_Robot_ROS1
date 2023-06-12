#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String, Int32


import cv2
from cv_bridge import CvBridge 

import math
import numpy as np

class SignIdentification:
    def __init__(self):
        self.img = None
        self.resized_image = None
        self.estado_anterior = None

        #self.sift = cv2.features2d.SIFT_create()
        self.ORB = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
        self.stop_image = cv2.cvtColor(cv2.imread("/home/puzzlebot/catkin_ws/src/sign_identification/images/stop_sign.png"), cv2.COLOR_BGR2GRAY)
        self.stop_resized = cv2.resize(self.stop_image, (690, 690))
        self.left_image = cv2.cvtColor(cv2.imread("/home/puzzlebot/catkin_ws/src/sign_identification/images/left_sign.png"), cv2.COLOR_BGR2GRAY)
        self.left_resized = cv2.resize(self.left_image, (690, 690))
        self.right_image = cv2.cvtColor(cv2.imread("/home/puzzlebot/catkin_ws/src/sign_identification/images/right_sign.png"), cv2.COLOR_BGR2GRAY)
        self.right_resized = cv2.resize(self.right_image, (690, 690))
        self.turnaround_image = cv2.cvtColor(cv2.imread("/home/puzzlebot/catkin_ws/src/sign_identification/images/turn_around.png"), cv2.COLOR_BGR2GRAY)
        self.turnaround_resized = cv2.resize(self.turnaround_image, (690, 690))
        self.giveway_image = cv2.cvtColor(cv2.imread("/home/puzzlebot/catkin_ws/src/sign_identification/images/give_way.png"), cv2.COLOR_BGR2GRAY)
        self.giveway_resized = cv2.resize(self.giveway_image, (690, 690))
        self.forward_image = cv2.cvtColor(cv2.imread("/home/puzzlebot/catkin_ws/src/sign_identification/images/forward.png"), cv2.COLOR_BGR2GRAY)
        self.forward_resized = cv2.resize(self.forward_image, (690, 690))
        self.work_image = cv2.cvtColor(cv2.imread("/home/puzzlebot/catkin_ws/src/sign_identification/images/work_sign.png"), cv2.COLOR_BGR2GRAY)
        self.work_resized = cv2.resize(self.work_image, (690, 690))

        self.stopkp, self.stopdsc = self.ORB.detectAndCompute(self.stop_image, None)
        self.leftkp, self.leftdsc = self.ORB.detectAndCompute(self.left_image, None)
        self.rightkp, self.rightdsc = self.ORB.detectAndCompute(self.right_image, None)
        self.turnkp, self.turndsc = self.ORB.detectAndCompute(self.turnaround_image, None)
        self.givewaykp, self.givewaydsc = self.ORB.detectAndCompute(self.giveway_image, None)
        self.forwardkp, self.forwarddsc = self.ORB.detectAndCompute(self.forward_image, None)
        self.workkp, self.workdsc = self.ORB.detectAndCompute(self.work_image, None)

        self.sign = "None"
        # Sign id:
        # 9: stop, 
        # 1: left, 
        # 2: right, 
        # 3: turnaround, 
        # 4: forward,
        # 5: giveway,  
        # 6: work
        # 0: nothing
        self.sign_id = 0

        rospy.init_node('signal_id')

        self.signal_id_pub = rospy.Publisher('/sign_identification/signal_id', String, queue_size=10)
        # self.signal_id_img = rospy.Publisher('/sign_identification/camera', Image, queue_size=10)
        self.signal_id_num = rospy.Publisher('/sign_identification/signal_number', Int32, queue_size=10)

        rospy.Subscriber('/video_source/raw', Image, self.img_callback)
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.bridge = CvBridge()

    def img_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
    def mse(self, imageA, imageB):
        err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
        err /= float(imageA.shape[0] * imageA.shape[1])
        return err
    
    def procesamiento(self):

        frame = self.img
        frame = frame[0:frame.shape[0]/2, 0:frame.shape[1]]
        
        topublish = self.img
        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Crop image to get the top part
        # gray = gray[0:gray.shape[0]/2, 0:gray.shape[1]]
        
        # Filtrar la imagen para reducir el ruido
        blurred = cv2.bilateralFilter(gray, 5, 75, 75)
        
        v = np.median(blurred)
        sigma = 0.33
        lower = int(max(0, (1.0 - sigma) * v)) 
        upper = int(min(255, (1.0 + sigma) * v))
        
        # Detectar los bordes en la imagen
        edges = cv2.Canny(blurred, lower, upper)
        
        # Encontrar los contornos en la imagen
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            self.sign = "None"
            self.sign_id = 0
            return self.sign
        
        big_contour = max(contours, key=cv2.contourArea)
        
        area = cv2.contourArea(big_contour)
        peri = cv2.arcLength(big_contour, True)
        
        vertices = cv2.approxPolyDP(big_contour, 0.01 * peri, True)
        num_vertices = len(vertices)
        
        x, y, w, h = cv2.boundingRect(big_contour)
        
        src = np.float32([(x, y), (x, y + h), (x + w, y + h), (x + w, y)])
        dst = np.float32([(0, 0), (0, 690), (690, 690), (690, 0)])
        
        cv2.rectangle(topublish, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        matrix = cv2.getPerspectiveTransform(src, dst)
        
        output_image = cv2.warpPerspective(frame, matrix, (690, 690), flags=cv2.INTER_LINEAR)
        
        # Convertir la imagen de salida a escala de grises
        output_gray = cv2.cvtColor(output_image, cv2.COLOR_BGR2GRAY)
        
        keypoints_1, descriptor_1 = self.ORB.detectAndCompute(output_image, None)
        if num_vertices >= 3 and num_vertices <= 6 and area > 800:
            matches_work = self.bf.match(descriptor_1, self.workdsc)
            matches_giveway = self.bf.match(descriptor_1, self.givewaydsc)
            
            # Ordenar las coincidencias por distancia
            matches1 = sorted(matches_work, key=lambda x: x.distance)
            matches2 = sorted(matches_giveway, key=lambda x: x.distance)
            
            mse1 = self.mse(output_gray, self.work_resized)
            mse2 = self.mse(output_gray, self.giveway_resized)
            #rospy.loginfo("Giveway: %s", mse2)

            # rospy.loginfo("Work: %d, Giveway: %d", len(matches1), len(matches2))
            # rospy.loginfo("Work: %d, Giveway: %d", mse1, mse2)
        
            if (len(matches1) > len(matches2)) and len(matches1) > 16 and mse1 < 18000:
                self.sign = "Work"
                self.sign_id = 6
            elif (len(matches2) > len(matches1)) and len(matches2) > 16 and mse2 < 18000:
                self.sign = "Give Way"
                self.sign_id = 5
        
        elif num_vertices == 8 and area > 800:
            matches_stop = self.bf.match(descriptor_1, self.stopdsc)
            mse5 = self.mse(output_gray, self.stop_resized)
            matches3 = sorted(matches_stop, key=lambda x: x.distance)

            if (len(matches3) > 16):
                self.sign = "Stop"
                self.sign_id = 9
        
        elif num_vertices > 8 and area > 800:
            matches_left = self.bf.match(descriptor_1, self.leftdsc)
            matches_right = self.bf.match(descriptor_1, self.rightdsc)
            matches_stop = self.bf.match(descriptor_1, self.stopdsc)
            matches_turn = self.bf.match(descriptor_1, self.turndsc)
            matches_forward = self.bf.match(descriptor_1, self.forwarddsc)
            
            mse3 = self.mse(output_gray, self.left_resized)
            mse4 = self.mse(output_gray, self.right_resized)
            mse6 = self.mse(output_gray, self.turnaround_resized)
            mse7 = self.mse(output_gray, self.forward_resized)
            
            mses = [mse4, mse3, mse6, mse7]
            
            matches1 = sorted(matches_left, key=lambda x: x.distance)
            matches2 = sorted(matches_right, key=lambda x: x.distance)
            matches4 = sorted(matches_turn, key=lambda x: x.distance)
            matches5 = sorted(matches_forward, key=lambda x: x.distance)
            
            matches = [len(matches1), len(matches2), len(matches4), len(matches5)]

 
            if (len(matches1) > 5 and mse3 == min(mses)):
                self.sign = "Left"
                self.sign_id = 1
            elif (len(matches2) > 5 and mse4 == min(mses)):
                self.sign = "Right"
                self.sign_id = 2 
            elif  (len(matches5) > 5 and mse7 == min(mses)):
                self.sign = "Forward"
                self.sign_id = 4
        else:
            self.sign = "None"
            self.sign_id = 0
        

        return self.sign
        
    def timer_callback(self, timer):
        if self.img is not None:
            estado = self.procesamiento()
            # rospy.loginfo("Senal: %s", estado)

            if estado != self.estado_anterior:
                self.signal_id_pub.publish(estado)
                self.signal_id_num.publish(self.sign_id)

                self.estado_anterior = estado

    def run(self):
        if not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('signal_id')
        signal_id = SignIdentification()
        signal_id.run()
    except rospy.ROSInterruptException:
        pass