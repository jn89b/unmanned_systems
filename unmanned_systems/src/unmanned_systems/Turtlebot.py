#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Turtlebot():
    def __init__(self):
        self.odom_x = None
        self.odom_y = None
        self.odom_z = None

        #declare publishers and subscribers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=5)

    def odom_cb(self,msg):
        """recieve message of odom"""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z

    def go_forward(self, speed):
        """commands the turtle to begin going forward at a certain speed"""
        twist = Twist()
        twist.linear.x = speed
        self.vel_publisher.publish(twist)

    def go_turn(self, turn_speed):
        """commands the turtle to begin turning at a angular speed"""
        twist = Twist()
        twist.angular.z = turn_speed
        self.vel_publisher.publish(twist)
