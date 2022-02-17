#!/usr/bin/env python
# -*- coding: utf-8 -*- 

#import python 
import rospy 

#import any messages you want
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def run_something():
    """empty code to run whatever your functions"""
    #print("hello world")
    pub = rospy.Publisher('chatter', String, queue_size=10)
    hello_str = "hello world %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    pub.publish(hello_str)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

if __name__ == '__main__':
    #initiate your node here
    rospy.init_node("some_node", anonymous=False)

    #control the publishing rate, if you want to..
    rate = rospy.Rate(20)

    
    """Comment out Option 1 or Option 2"""
    """Option 1 using a while not command"""
    #this is like a while loop, until you Ctrl+C to end the node
    while not rospy.is_shutdown():
        
        run_something()
        rospy.Subscriber("chatter", String, callback)
        #use rate.sleep to control it at your specified ros rate
        rate.sleep()

    """Option 2 using rospy.spin()"""
    #rospy.spin will run your node at whatever rate it wants to do it at
    #run_something()
    #rospy.spin()
