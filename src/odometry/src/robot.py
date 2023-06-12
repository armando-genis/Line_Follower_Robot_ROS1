#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D 
from std_msgs.msg import Float32
from math import cos as cos
from math import sin as sin
import numpy as np

class Odom:
    def __init__(self):
        self.wl = 0.0
        self.wr = 0.0
        
        
        self.l = 0.18
        self.r = 0.05
        
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose = Pose2D()
        
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0
        
        
        self.velocity = 0.0
        self.distance = 0.0
        self.angular_velocity = 0.0

        self.dt = 0.1
        
        
        
        rospy.init_node('odometry')
        self.odometry_pub = rospy.Publisher('/odometry', Pose2D, queue_size=10)
        rospy.Subscriber('/wl', Float32, self.wl_callback)
        rospy.Subscriber('/wr', Float32, self.wr_callback)

        rospy.Timer(rospy.Duration(self.dt), self.timer_callback)


    def wl_callback(self, msg):
        self.wl =msg.data

    def wr_callback(self,msg):
        self.wr = msg.data
        


    def timer_callback(self, timer):

        self.velocity = (self.wr + self.wl) * (self.r/2.0)
        self.angular_velocity = (self.wr - self.wl) * (self.r/self.l)
        

        # riman sum
        self.distance += self.velocity * self.dt
        self.theta += self.angular_velocity * self.dt
        # x & y calculation
        self.pose.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        self.pose.x += cos(self.theta) * self.velocity * self.dt
        self.pose.y += sin(self.theta) * self.velocity * self.dt
        
        self.odometry_pub.publish(self.pose)
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    s= Odom()
    s.run()



