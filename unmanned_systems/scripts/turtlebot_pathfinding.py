#!/usr/bin/env python
# -*- coding: utf-8 -*- 

#import python 
import rospy
import numpy as np
import math as m
from unmanned_systems import Turtlebot 
#import any messages you want
from unmanned_systems import path_finding
from nav_msgs.msg import Odometry

def run_something():
    """empty code to run whatever your functions"""
    print("hello world")

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def compute_angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    
if __name__ == '__main__':

    #initiate your node here
    rospy.init_node("turtlebot_pathfinding", anonymous=False)

    #control the publishing rate, if you want to..
    rate = rospy.Rate(20)

    ### ENTER PARAMS

    turtlebot = Turtlebot.Turtlebot(kp=1.0,ki=0.0,kd=0.0,rate=20)
    start = [0,0]
    goal = [8,9]

    x_span = [0,10]
    y_span = [0,10]
    spacing = 0.5
    obst_radius = 0.65
    config_space = path_finding.ConfigSpace(x_span, y_span, spacing)
    obstacle_list = [[1,1], [4,4], [3,4], [5,0], [5,1], [0,7], [1,7], [2,7], [3,7]]
    
    config_space.set_obstacles(obstacle_list)

    heading_tolerance = 0.01
    distance_tolerance = 0.05
    waypoints = path_finding.astar(start_position=start, goal_position =goal, 
                    configSpace=config_space, iter_max=500, 
                    obstacle_radius=obst_radius)

    #waypoints = [[1,0]]

    done = False
    #this is like a while loop, until you Ctrl+C to end the node
    while not rospy.is_shutdown():
        if turtlebot.odom_position == [None,None]:
            continue
        
        if done == True:
            #print("stopping")
            turtlebot.go_forward_turn(0.0,0.0)

        else:
            for waypoint in waypoints:
                print('going to waypoint', waypoint)
                at_heading = False
                at_waypoint = False
                
                while at_heading == False and at_waypoint == False:
                    #print("turtlebot", turtlebot.odom_yaw_deg)
                    while at_heading == False:
                        at_heading  = turtlebot.track_target(heading_tolerance, waypoint)
                        if at_heading ==  True:
                            print("heading is done")
                            break

                    while at_waypoint == False:
                        at_waypoint = turtlebot.track_go_target(distance_tolerance, waypoint)
                        if at_waypoint == True:
                            print("at waypoint", waypoint)
                            break
                    
                if waypoint == waypoints[-1]:
                    print("waypoints reached")
                    done = True
                    break
    

        #use rate.sleep to control it at your specified ros rate
        rate.sleep()


