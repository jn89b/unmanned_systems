#!/usr/bin/env python
# -*- coding: utf-8 -*- 

#import python 
import rospy 

#import any messages you want
from nav_msgs.msg import Odometry

def run_something():
    """empty code to run whatever your functions"""
    print("hello world")

if __name__ == '__main__':

    #initiate your node here
    rospy.init_node("some_node", anonymous=False)

    #control the publishing rate, if you want to..
    rate = rospy.Rate(20)

    """Comment out Option 1 or Option 2""""
    """Option 1 using a while not command"""
    #this is like a while loop, until you Ctrl+C to end the node
    while not rospy.is_shutdown():
        
        run_something()
        
        #use rate.sleep to control it at your specified ros rate
        rate.sleep()

    """Option 2 using rospy.spin()"""
    #rospy.spin will run your node at whatever rate it wants to do it at
    #run_something()
    #rospy.spin()
