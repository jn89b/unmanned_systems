#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import rospy 
from unmanned_systems import Turtlebot
from unmanned_systems import util_functions
if __name__ == '__main__':
    #initiate node
    rospy.init_node('turtlebot_go_python_class', anonymous=False)
    
    #control the rate of the script to recieve messages and or send messages
    rate = rospy.Rate(20)

    #instantiate Turtlebot or create an object of class Turtlebot 
    turtlebot = Turtlebot.Turtlebot()

    while not rospy.is_shutdown():
        turtlebot.go_forward(0.0)
        #importing utility funciton and using the stupid function
        util_functions.get_stupid_function()
        print("x position: ", turtlebot.odom_x)
        rate.sleep() # sleep at this controlled rate