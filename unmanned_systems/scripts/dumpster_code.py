#!/usr/bin/env python
# -*- coding: utf-8 -*- 

"""
Control Turtlebot:
    SHOW THIS: go forward at vel speed for x seconds
        after x seconds make it stop
    SHOW THIS: turn for some ang speed for x  seconds
    
    go forward at vel speed for x seconds

    IF I HAVE TIME: put into a launch file log data

"""

#import python 
import rospy 
import time

#import any messages you want
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Turtlebot():
    def __init__(self):
        """define attributes of turtlebot"""
        #turtlebot has position 
        self.odom_x = None
        self.odom_y = None
        self.odom_z = None

        #declare publishers and subscribers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        #velocity publisher  
        self.vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=5)

    def odom_cb(self,msg):
        """recieve position from odometry"""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z

    def move_forward(self, cmd_speed):
        """tell turtlebot to go forward"""
        forward_cmd = Twist()
        forward_cmd.linear.x = cmd_speed
        #publish the command
        self.vel_publisher.publish(forward_cmd)

    def go_turn(self,ang_speed):
        """tell turtlebot to turn"""
        forward_cmd = Twist()
        forward_cmd.angular.z = ang_speed
        self.vel_publisher.publish(forward_cmd)

def run_something():
    """empty code to run whatever your functions"""
    print("hello world")

if __name__ == '__main__':

    #initiate your node here
    rospy.init_node("some_node", anonymous=False)

    #control the publishing rate, if you want to..
    rate = rospy.Rate(20)

    #create object turtlebot of class Turtlebot
    turtlebot = Turtlebot()

    """Comment out Option 1 or Option 2"""
    """Option 1 using a while not command"""
    start_time = time.time()
    
    time_stop = 3.0
    time_stop2 = 5.0

    cmd_speed = 0.15 

    #this is like a while loop, until you Ctrl+C to end the node
    while not rospy.is_shutdown():

        elapsed_time = time.time() - start_time
        if elapsed_time <= time_stop:
            print("im going forward")
            turtlebot.move_forward(cmd_speed)
        elif elapsed_time >= time_stop and elapsed_time<=time_stop2:
            print("im turning")
            turtlebot.move_forward(0.0)
            turtlebot.go_turn(1.0)
        else:
            print("im stopping")
            turtlebot.move_forward(0.0)
                
        #use rate.sleep to control it at your specified ros rate
        rate.sleep()

    #turtlebot.move_forward(0.0)
    """Option 2 using rospy.spin()"""
    #rospy.spin will run your node at whatever rate it wants to do it at
    #run_something()
    #rospy.spin()
