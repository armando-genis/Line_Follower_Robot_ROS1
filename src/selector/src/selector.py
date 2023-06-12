#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool, String, Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import math


# To do 
#  TODO 1. checar si traffic_ligh_to_zero(self): is working *
#  TODO 2. poner las flgas despues de la rutina *
#  TODO 3. Subcribirse al topico de la zona blanca y hace la condicoin de que se pare *
#  TODO 4. checar todas las senales. *

class Selector:
    def __init__(self):
        self.line_detection_twist = Twist()
        self.flag_red = Bool()
        self.flag_yellow = Bool()
        self.flag_green = Bool()
        self.flag_horizontal = Bool()
        self.signal_id = "None"
        self.flag_white = Bool()
 
        # last color traffic
        self.last_color = 0

        # sign detection
        self.number_sign = 0
        self.window_size = 14
        self.window = np.zeros(self.window_size)
        self.mode_sign = 0
        # Sing detection past state
        self.past_sign = 0

        # delay
        self.sampleTime = 0.1

        # Counter for traffic_light_to_zero()
        self.counter_traffic = 0.0
        
        # state for the routines
        self.state = 0
        
        #Data
        self.data = Twist()

        rospy.init_node('selector_node')
        # Sub to line detection 
        rospy.Subscriber('/cmd_vel/line_planning', Twist, self.line_detection_callback)
        # Sub to flag red of traffic light detection pkg
        rospy.Subscriber('/flag_red', Bool, self.flag_red_callback)
        # Sub to flag green of traffic light detection pkg
        rospy.Subscriber('/flag_green', Bool, self.flag_green_callback)
        # Sub to flag white spaces of line pkg
        rospy.Subscriber('/flag_white_space', Bool, self.flag_white_callback)
        # Sub to flag horizontal flags from line detection pkg
        rospy.Subscriber('/horizontal_line_detected', Bool, self.flag_horizontal_callback)
        # Sub to signal id of the sign identification pkg
        rospy.Subscriber('/sign_identification/signal_id', String, self.signal_id_callback)
        # Sub to signal id of the sign identification pkg
        rospy.Subscriber('/sign_identification/signal_number', Int32, self.signal_num_callback)

        self.robot_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

# * =================== Callbacks ============================================================

    def flag_white_callback(self,msg):
        self.flag_white = msg.data

    def signal_num_callback(self,msg):
        self.number_sign = msg.data
        
    def signal_id_callback(self,msg):
        self.signal_id = msg.data

    def line_detection_callback(self,msg):
        self.line_detection_twist = msg
        
    def flag_red_callback(self,msg):
        self.flag_red = msg.data

    
    def flag_green_callback(self,msg):
        self.flag_green = msg.data
    
    def flag_horizontal_callback(self,msg):
        self.flag_horizontal = msg.data


# * ===================== Move function ========================================================

    def stop(self):        
        zero_twist = self.line_detection_twist
        zero_twist.linear.x = 0.0
        zero_twist.angular.z = 0.0
        self.robot_move.publish(zero_twist)
        
    def reduce_speed(self):
        reduced_twist = self.line_detection_twist
        reduced_twist.linear.x *= 0.2
        reduced_twist.angular.z *= 0.2
        self.robot_move.publish(reduced_twist)

    def normal_speed(self):
        self.robot_move.publish(self.line_detection_twist)
        
    def forward(self):
        data = self.line_detection_twist
        data.linear.x = 0.07
        data.angular.z = 0.0
        self.robot_move.publish(data)

# * ================= Mode filter =================================================================

    def rolling_mode_filter(self):
        self.window = np.roll(self.window, -1)
        self.window[-1] = self.number_sign
        values, counts = np.unique(self.window, return_counts=True)
        mode_index = np.argmax(counts)
        self.mode_sign = values[mode_index]
        
        
# * ==================== Traffic light detection ====================================================
    def traffic_light_detec(self):
        bool = False
        if self.flag_red or self.flag_green:
            bool = True
        return bool
    
    def traffic_light_planning(self):
        if self.flag_red:
            print("stop (traffic light -> red)")
            self.last_color = 1
            self.stop()
        elif self.flag_green:
            print("normal speed  (traffic light -> green or green & horizontal)")
            self.last_color = 3
            self.normal_speed()




# * ================================ Sign detection ====================================================

    def sign_detection(self):

        # Sign id:
        # 9: stop, 
        # 1: left, 
        # 2: right, 
        # 4: forward,
        # 5: giveway,  
        # 6: work
        # 0: nothing

        if self.mode_sign == 9:
            print("stop")
            self.past_sign = 9
        elif self.mode_sign == 1:
            self.past_sign = 1
            print("left")
        elif self.mode_sign == 2:
            self.past_sign = 2
            print("right")
        elif self.mode_sign == 4:
            self.past_sign = 4
            print("forward")
        elif self.mode_sign == 5:
            self.past_sign = 5
            print("giveway")
        elif self.mode_sign == 6:
            self.past_sign = 6
            print("work")
        elif self.mode_sign == 0:
            print("nothing")
            # self.past_sign = 0

# * ================================= Routines for signs and logic ===============================================

    # ! ----- if there is a traffic light
    def intercept_routine(self):
        # <------- turning left ------>
        if self.past_sign == 1:
            print("turning left")
            self.forward_routine()
            self.left_turn_routine()
            self.past_sign = 0
            self.last_color ==  0
                
        #  <------- turning right ------>
        elif self.past_sign == 2:
            print("turning right")
            self.forward_routine()
            self.right_turn_routine()
            self.past_sign = 0
            self.last_color ==  0

        elif self.past_sign == 4:
            print("foward")
            # self.stop_before()
            self.forward_routine_sign()  
            self.past_sign = 0
            self.last_color ==  0
        else:
            self.stop()
            self.past_sign = 0

            
    # ! ----- if there is only lines
    def intercept_routine_onlyLines(self):
        # <------- turning left ------>
        if self.past_sign == 1:
            print("turning left")
            self.stop_before()
            self.forward_routine()
            self.left_turn_routine()
            
                
        #  <------- turning right ------>
        elif self.past_sign == 2:
            print("turning right")
            self.stop_before()
            self.forward_routine()
            self.right_turn_routine()

        #  <------- foward ------>
        elif self.past_sign == 4:
            print("foward")
            self.stop_before()
            self.forward_routine_sign() 
            self.past_sign = 0      

        else:

            self.stop()
            

            
# * ======================= ROUTINES =================================================

    def right_routine(self,delay):
        timerCounter = 0
        while(timerCounter < delay):
            timerCounter += self.sampleTime
            data = self.line_detection_twist
            data.linear.x = 0.07
            data.angular.z = -0.085
            self.robot_move.publish(data)
        self.state = 0
        self.last_color ==  0
        

    def left_routine(self,delay):
        timerCounter = 0
        while(timerCounter < delay):
            timerCounter += self.sampleTime
            data = self.line_detection_twist
            data.linear.x = 0.07
            data.angular.z = 0.085
            self.robot_move.publish(data)
        self.state = 0
        self.last_color ==  0
        

    def reduce_speed_sign(self,delay):
        timerCounter = 0
        if self.past_sign == 5 or self.past_sign == 6:
            reduced_twist = self.line_detection_twist
            reduced_twist.linear.x *= 0.6
            reduced_twist.angular.z *= 0.6
            self.robot_move.publish(reduced_twist)
            timerCounter += self.sampleTime
            if timerCounter >= delay:
                self.past_sign = 0
                
                
    def right_turn_routine(self):
        turning_twist = Twist()
        turning_twist.linear.x = 0.1
        turning_twist.angular.z = -0.3  # negative for turning right
        self.robot_move.publish(turning_twist)
        rospy.sleep(1)  # let it turn for 1 second
        self.stop()  # stop the robot after turning
        self.past_sign = 0
        self.last_color ==  0


    def stop_before(self):
        self.stop()
        rospy.sleep(2)  # let it turn for 1 second
        self.stop()  # stop the robot after turning

    
    def left_turn_routine(self):
        turning_twist = Twist()
        turning_twist.linear.x = 0.1
        turning_twist.angular.z = 0.3  # positive for turning left
        self.robot_move.publish(turning_twist)
        rospy.sleep(1)  # let it turn for 1 second
        self.stop()  # stop the robot after turning
        self.past_sign = 0
        self.last_color ==  0
        
        
    def forward_routine(self):
        forward_twist = Twist()
        forward_twist.linear.x = 0.1
        forward_twist.angular.z = 0.0
        self.robot_move.publish(forward_twist)
        rospy.sleep(2.5)  # let it move forward for 1 second
        self.stop()  # stop the robot after moving forward

    def forward_routine_sign(self):
        forward_twist = Twist()
        forward_twist.linear.x = 0.1
        forward_twist.angular.z = 0.0
        self.robot_move.publish(forward_twist)
        rospy.sleep(3)  # let it move forward for 1 second
        self.stop()  # stop the robot after moving forward
        self.past_sign = 0
        self.last_color ==  0
        

# * ================================== Delay ==========================================================
    def delay_ros(self, delay):
        timerCounter = 0
        while(timerCounter < delay):
            timerCounter += self.sampleTime
            rospy.sleep(self.sampleTime)

# * =========================== self.last_color to zero (traffic light) ================================
    def traffic_light_to_zero(self):
        if self.last_color ==  3 and not self.flag_red:
            self.counter_traffic += self.sampleTime
            if self.counter_traffic >= 4:
                self.last_color = 0
                self.counter_traffic = 0


# * ================================== Main brain ======================================================
    def timer_callback(self, timer):
        self.traffic_light_to_zero()
        self.rolling_mode_filter()
        self.sign_detection()
        # print("------------------------------> HORI:", self.flag_horizontal )
        # self.traffic_light_to_zero()
        print()
        print("SIGN PAST:  <------ ",self.past_sign)
        print("SIGN COLOR:  <------ ",self.last_color)

        print('RED: ', self.flag_red, 'GREEN: ',self.flag_green)


        # !<-------------- if work. give way or work is detected ------------>

        if self.mode_sign == 9: 
            print("stop")
            self.stop()
        elif self.mode_sign == 5 or self.mode_sign == 6:
            print("give way" if self.mode_sign == 5 else "work")
            self.reduce_speed_sign(3)


        #! <-------------- if they are not detected ------------------------>
        else:
            #! <-----------  if only is detected the traffic light --------->

            if self.traffic_light_detec() and not self.flag_horizontal:
                print(" detected the traffic light ")
                self.traffic_light_planning()

            # ! <----------- if only lines are detected --------------------->

            # elif self.flag_horizontal and not self.last_color ==  3:
            #     print("Only horizontal lines detected -> stop")
            #     self.intercept_routine_onlyLines()

            #! <------------ If lines are detected and is was green --------->
            elif self.flag_horizontal and self.last_color ==  3:
                print(" hORI DETECTED ")
                self.intercept_routine()

            #! <------------ if nothing is detected ------------------------->
            else:
                print("normal")
                self.normal_speed()
                
# * ====================================================================================================
        
    def run(self):
        if not rospy.is_shutdown():
            rospy.spin()



if __name__ == '__main__':
    
    try:
        s = Selector()
        s.run()
    except rospy.ROSInterruptException:
        pass