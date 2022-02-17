#!/usr/bin/env python
# -*- coding: utf-8 -*- 
"""
Make a turtlebot:
    move forward for some 0.1 linear vel for 3 seconds 
    turn for some some x ang vel for t seconds
    stop after x seconds

log data
"""

#import python 
import rospy 
import time 

#import any messages you want
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PID():
    """kp, ki, kd, desired, actual, dt"""
    def __init__(self):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def get_error(self):
        pass

    def get_ierror(self):
        pass

    def get_derror(self):
        pass

    def compute_gains(self):
        pass

class Turtlebot():
    """turtlebot has position, and position control commands"""
    def __init__(self):
        self.odom_x = None
        self.odom_y = None
        self.odom_z = None
        #self.odom_position = [None,None,None]
        self.odom_yaw = None

        #declare publishers and subscribers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=5)

        #linear controller
        self.linear_controller = PID()
        #heading controller
        self.heading_controller = PID()        

    def odom_cb(self, msg):
        """get odometry position"""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z
        #self.odom_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                             orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.odom_yaw = yaw
    
    def move(self, vel_cmd):
        """move turtlebot in linear path in m/s"""
        move = Twist()
        move.linear.x = vel_cmd
        self.vel_publisher.publish(move)

    def turn(self, vel_cmd):
        """turn turtlebot about the z axis in rad/s"""
        move = Twist()
        move.angular.z = vel_cmd
        self.vel_publisher.publish(move)

if __name__ == '__main__':

    #initiate your node here
    rospy.init_node("some_node", anonymous=False)

    #control the publishing rate, if you want to..
    rate = rospy.Rate(20)

    #create object turtlebot of class Turtlebot
    turtlebot = Turtlebot()
    
    #initialize some time interval
    start_time = time.time()

    #declare time bounds
    time_stop = 3.0
    time_stop2 = 5.0

    """Option 1 using a while not command"""
    #this is like a while loop, until you Ctrl+C to end the node
    while not rospy.is_shutdown():
        
        time_interval = time.time() - start_time
        #print("time interval is", time_interval)
        
        if time_interval <= time_stop:
            #print("im going bro")
            turtlebot.move(-0.1)
        elif time_interval >= time_stop and time_interval <= time_stop2:
            print("my heading is", turtlebot.odom_yaw)
            #print("im turning bro")
            turtlebot.turn(0.15)
        else:
            #print("im stopping bro")
            turtlebot.move(0.0)

        #print("turtlebot x", turtlebot.odom_x)
        #use rate.sleep to control it at your specified ros rate
        rate.sleep()

    """Option 2 using rospy.spin()"""
    #rospy.spin will run your node at whatever rate it wants to do it at
    # run_something()
    # rospy.spin()
