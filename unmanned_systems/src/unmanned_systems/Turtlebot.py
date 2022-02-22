#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import rospy
import math as m
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from unmanned_systems.path_finding import compute_distance

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def compute_angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'"""

    return m.atan2(v2[1]-v1[1],v2[0]-v1[0])

def compute_distance(v1,v2):
    return m.dist(v2,v1)

class PID():
    """Compensator"""
    def __init__(self, kp, ki, kd, rate):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pre_error = 0.0
        self.pre_ierror = 0.0
        self.rate = rate

    def compute_p_error(self, desired, actual):
        """get errors"""
        return desired - actual

    def __compute_i_error(self, error):
        """compute i gain with trapezoid rule"""
        return self.pre_ierror + ((error + self.pre_error)) * (1/self.rate)

    def __compute_d__error(self, error):
        """compute d gain"""
        return (error - self.pre_error)/(1/self.rate)


    def compute_gains(self, desired, actual):
        """return PID gains"""
        error = self.compute_p_error(desired, actual)
        i_error = self.__compute_i_error(error)
        d_error = self.__compute_d__error(error)

        gain = self.kp*error + self.ki*i_error + self.kd*d_error

        self.pre_error = error
        self.pre_ierror = i_error

        return gain,error

class Turtlebot():
    def __init__(self,kp,ki,kd,rate):

        self.odom_position = [None,None]
        self.odom_yaw_rad = None
        self.odom_yaw_deg = None
        self.pid = PID(kp,ki,kd,rate)
        #declare publishers and subscribers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=5)

    def odom_cb(self,msg):
        """recieve message of odom"""
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation

        #orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                             orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.odom_yaw_rad = yaw
        self.odom_yaw_deg = m.degrees(self.odom_yaw_rad) % 360

        self.odom_position = [odom_x, odom_y]

    def go_forward(self, speed):
        """commands the turtle to begin going forward at a certain speed"""
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0
        self.vel_publisher.publish(twist)

    def go_turn(self, turn_speed):
        """commands the turtle to begin turning at a angular speed"""
        turn_speed = self.check_angular_threshold(turn_speed)
        twist = Twist()
        twist.angular.z = turn_speed
        twist.linear.x = 0
        self.vel_publisher.publish(twist)

    def go_forward_turn(self, speed, turn_speed):
        """allows turtlebot to go forward and turn based on """
        turn_speed = self.check_angular_threshold(turn_speed)
        speed = self.check_linear_speed(speed)
        twist = Twist()
        twist.angular.z = turn_speed
        twist.linear.x = speed
        self.vel_publisher.publish(twist)

    def check_linear_speed(self,speed):
        max_line_speed = 0.15
        if abs(speed) >= max_line_speed:
            return max_line_speed
        else:
            return speed

    def check_angular_threshold(self, turn_speed):
        """check if gains are too much"""
        max_ang_speed = 1.5
        if abs(turn_speed)>= max_ang_speed:
            #print("too much")
            if turn_speed >= 0:
                return max_ang_speed
            else:
                return -max_ang_speed
        else:
            return turn_speed

    def track_target(self, error_tol, desired_position):
        """track the target with compensator 
        control x position and control yaw 
        """
        desired_angle = compute_angle_between(desired_position, self.odom_position)
        gain, error = self.pid.compute_gains(desired_angle, self.odom_yaw_rad)

        if error >= error_tol:
            #print("turning right")
            #why is this opposite
            self.go_turn(gain)
            return False
        elif error <= -error_tol:
            #print("turning left")
            self.go_turn(gain)
            return False
        elif abs(error) <= error_tol:
            return True

    def go_to_target(self, error_tol, desired_position):
        """go to the target with compensator 
        control x position and control yaw 
        """
        current_dist = compute_distance(desired_position, self.odom_position)
        gain, error = self.pid.compute_gains(error_tol, current_dist)
        if error >= error_tol:
            #why is this opposite
            self.go_forward(-0.15)
            return False
        elif error <= -error_tol:
            #print("turning left")
            self.go_forward(-0.15)
            return False
        elif abs(error) <= error_tol:
            self.go_forward_turn(0.0, 0.0)
            return True

    def track_go_target(self, error_tol, desired_position):
        """go to the target with compensator 
        control x position and control yaw 
        """
        desired_angle = compute_angle_between(desired_position, self.odom_position)
        angle_gains, angle_error = self.pid.compute_gains(desired_angle, self.odom_yaw_rad)

        current_dist = compute_distance(desired_position, self.odom_position)
        distance_gain, distance_error = self.pid.compute_gains(error_tol, abs(current_dist))
        if distance_error >= error_tol:
            #why is this opposite
            self.go_forward_turn(-0.1, angle_gains)
            return False
        elif distance_error <= -error_tol:
            #print("turning left")
            self.go_forward_turn(-0.1, angle_gains)
            return False
        elif abs(distance_error) <= error_tol:
            self.go_forward_turn(0.0, 0.0)
            return True

        