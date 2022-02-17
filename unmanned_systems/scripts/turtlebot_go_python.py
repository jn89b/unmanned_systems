#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

"""initialize variables"""
odom_x = None
odom_y = None
odom_z = None

def odom_cb(msg):
    """callback odometry message from subscriber with global variables"""
    global odom_x
    global odom_y
    global odom_z 

    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y
    odom_z = msg.pose.pose.position.z
    
def go_forward(vel_publisher,speed):
    """commands the turtle to begin going forward at a certain speed"""
    twist = Twist()
    twist.linear.x = speed
    vel_publisher.publish(twist)

def go_turn(vel_publisher,turn_speed):
    """tell the turtle to begin turning"""
    twist = Twist()
    twist.angular.z = turn_speed
    vel_publisher.publish(twist)

if __name__ == '__main__':

    #initiate node
    rospy.init_node('turtlebot_go_python', anonymous=False)
    
    #declare publishers and subscribers
    odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)

    #control the rate of the script to recieve messages and or send messages
    rate = rospy.Rate(10)
    odom_x = 0
    while not rospy.is_shutdown():

        go_forward(vel_pub, 0.1)
        print("x position: ", odom_x)

        if odom_x >= 1.5:
            go_forward(vel_pub, 0.0)

        rate.sleep() # sleep at this controlled rate