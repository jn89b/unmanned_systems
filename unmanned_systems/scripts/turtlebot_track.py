#! /usr/bin/env python
from curses import KEY_PPAGE
from distutils.log import error
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

import rospy
from geometry_msgs.msg import Point, PoseStamped, Twist, \
    PoseWithCovarianceStamped

from nav_msgs.msg import Odometry



e = """
Communications Failed
"""



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

        return gain

class Turtlebot():
    """
    turtlebot
    """
    def __init__(self, rate):
        self.odom_x = None
        self.odom_y = None
        self.odom_z = None
        
        #sampled rate -> should probably be how fast I'm processing the apriltag
        self.rate_val = rate_val

        #declare publishers and subscribers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=5)

        #tracking apriltag
        self.target_sub = rospy.Subscriber('tag/pose',  PoseStamped, self.target_cb)
        self.target_position = [None, None, None]

        self.pid = PID(kp=1, ki=0.0, kd=0.05, rate=self.rate_val)

        ##MAXIMUM values for turtlebot for linear and angular speed
        self.max_linear_speed = 0.22 #m/s
        self.max_angular_speed = 2.84 #rad/s

    def target_cb(self,msg):
        """retrieves position of target"""
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        target_z = msg.pose.position.z

        self.target_position = [target_x, target_y, target_z]

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

    def compute_error(self,desired, current):
        """computes the error of actual and current position"""
        return desired - current

    def center_target(self):
        """track target"""
        error_tol = 0.05
        if self.target_position != [None,None,None]:
            error_y = self.compute_error(error_tol, self.target_position[1])
            #print("error y is", error_y)
            
            ## if negative error turn left, if positive error turn right
            if error_y >= error_tol:
                #print("turning right")
                self.go_turn(-0.15)
            elif error_y <= -error_tol:
                #print("turning left")
                self.go_turn(0.15)
            else:
                #print("im good")
                self.go_turn(0)

    def check_linear_speed(self,gain_val):
        """checks if we are above the maximum threshold of the speed"""
        if gain_val > self.max_linear_speed:
            return self.max_linear_speed
        if gain_val < -self.max_linear_speed:
            return -self.max_linear_speed 

        return gain_val

    def check_angular_speed(self, gain_val):
        
        if gain_val > self.max_angular_speed:
            return self.max_angular_speed

        if gain_val < self.max_angular_speed:
            return -self.max_angular_speed 
        
        return gain_val

    def track_target(self):
        """track the target with compensator 
        control x position and control yaw 
        """
        error_tol = 0.015
        if self.target_position != [None,None,None]:
            error = self.pid.compute_p_error(error_tol, self.target_position[1])        
            turn_gain = self.pid.compute_gains(error_tol, self.target_position[1])
            #turn_gain = self.check_angular_speed(turn_gain)
            #print("error is", error)
            #print("turn gain is", turn_gain)
        
            # if negative error turn left, if positive error turn right
            if error >= error_tol:
                #print("turning right")
                #why is this opposite
                self.go_turn(-turn_gain)
            elif error <= -error_tol:
                #print("turning left")
                self.go_turn(-turn_gain)
            else:
                #print("im good")
                self.go_turn(0)
            

if __name__ == '__main__':

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

        #initiate node
        rospy.init_node('turtlebot_go_python_class', anonymous=False)
        
        #rate_val
        rate_val = 40

        #control the rate of the script to recieve messages and or send messages
        rate = rospy.Rate(rate_val)

        #instantiate Turtlebot or create an object of class Turtlebot 
        turtlebot = Turtlebot(rate_val)

        while not rospy.is_shutdown():
            #turtlebot.center_target()
            turtlebot.track_target()
            rate.sleep()

        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        turtlebot.vel_publisher.publish(twist)
