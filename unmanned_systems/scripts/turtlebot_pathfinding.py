#!/usr/bin/env python
# -*- coding: utf-8 -*- 

#import python 
import rospy 

#import any messages you want
from unmanned_systems import path_finding
from nav_msgs.msg import Odometry

def run_something():
    """empty code to run whatever your functions"""
    print("hello world")

if __name__ == '__main__':

    #initiate your node here
    rospy.init_node("turtlebot_pathfinding", anonymous=False)

    #control the publishing rate, if you want to..
    rate = rospy.Rate(20)

    ### ENTER PARAMS
    start = [0,0]
    goal = [8,9]

    x_span = [0,10]
    y_span = [0,10]
    spacing = 0.5
    obst_radius = 0.5
    config_space = path_finding.ConfigSpace(x_span, y_span, spacing)
    obstacle_list = [[1,1], [4,4], [3,4], [5,0], [5,1], [0,7], [1,7], [2,7], [3,7]]
    config_space.set_obstacles(obstacle_list)

    waypoints = path_finding.astar(start_position=start, goal_position =goal, 
                    configSpace=config_space, iter_max=500, 
                    obstacle_radius=obst_radius)
    
    print("waypoints are", waypoints)
    
    """Comment out Option 1 or Option 2"""
    """Option 1 using a while not command"""
    #this is like a while loop, until you Ctrl+C to end the node
    # while not rospy.is_shutdown():
        
    #     run_something()
        
    #     #use rate.sleep to control it at your specified ros rate
    #     rate.sleep()

    """Option 2 using rospy.spin()"""
    #rospy.spin will run your node at whatever rate it wants to do it at
    #run_something()
    #rospy.spin()
