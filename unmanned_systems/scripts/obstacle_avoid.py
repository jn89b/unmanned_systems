#!/usr/bin/env python3
# -*- coding: utf-8 -*- 


"""
read turtlebot /scan topic which is the lidar

If obstacle close then publish to twist vel command to avoid 
the robot

"""
#import python 
import rospy
import numpy as np
import math as m
from unmanned_systems import Turtlebot 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #
from nav_msgs.msg import Odometry

class ObstacleAvoid():
    def __init__(self):
        laser_sub = rospy.Subscriber("scan", LaserScan, self.laser_cb)

        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        
        #setting to meters
        self.scan_tol_min = 0.5
        self.scan_tol_max = 0.5

        self.angle_left = 300
        self.angle_right = 60

    def laser_cb(self,msg):
        """read lidar information from turtlebot """
        
        mid_heading = msg.ranges[0]
        right_heading = msg.ranges[self.angle_right]
        left_heading= msg.ranges[self.angle_left]

        #this is stupid, but lidar has some issues of detection and would be 0.0 even though
        #there is not an object in front of it
        if mid_heading == 0.0:
            mid_heading = 5.0
        if left_heading == 0.0:
            left_heading = 5.0
        if right_heading == 0.0:
            right_heading = 5

        print('-------------------------------------------')
        print('Range data at 0 deg:   {}'.format(mid_heading))
        print('Range data at 60 deg:  {}'.format(right_heading))
        print('Range data at 300 deg: {}'.format(left_heading))
        print('-------------------------------------------')


        if (mid_heading>self.scan_tol_min) and (left_heading>self.scan_tol_max) and (right_heading>self.scan_tol_min): 
            print("good to go")
            avoid = Twist()
            avoid.linear.x = 0.0
            avoid.angular.z = 0.0
            self.twist_pub.publish(avoid)
        else:
            avoid = Twist()
            avoid.linear.x = 0.0
            avoid.angular.z = 0.35
            print("juking")
            if (mid_heading>self.scan_tol_min) and (left_heading>self.scan_tol_max) and (right_heading>self.scan_tol_min): 
                avoid.linear.x = 0.0
                avoid.angular.z = 0.0
                print("avoid complete")

            self.twist_pub.publish(avoid)

if __name__=='__main__':
    rospy.init_node('obs_avoid')

    obstacleavoid = ObstacleAvoid()

    rospy.spin()


