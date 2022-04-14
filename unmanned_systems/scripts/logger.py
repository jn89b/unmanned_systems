#!/usr/bin/env python

import rospy
import time
import csv
import os
import datetime
import math

#import any messages you want
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class TBInfo():
	def __init__(self,odom_topic, vel_topic):
		self.odom_x = None
		self.odom_y = None
		self.yaw = None

		self.vel_x = 0.0
		self.vel_yaw = 0.0

		self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_cb) 
		self.vel_sub = rospy.Subscriber(vel_topic, Twist, self.vel_cb)

	def odom_cb(self,msg):
		"""callback to get information"""
		self.odom_x = msg.pose.pose.position.x
		self.odom_y = msg.pose.pose.position.y

		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y,
						 orientation_q.z, orientation_q.w]
		
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		#print("self.yaw", self.yaw)
		self.yaw = math.degrees(yaw % 360)

	def vel_cb(self,msg):
		"""get velocity information"""
		self.vel_x = msg.linear.x
		self.vel_yaw = msg.angular.z
		
if __name__ == '__main__':

	turtlebot_name = "turtlebot2"
	odom_position_topic = turtlebot_name+"/odom"
	vel_cmd_topic = "/cmd_vel"

	tbinfo = TBInfo(odom_position_topic, vel_cmd_topic)

	# register the node with the name logger
	rospy.init_node('logger', anonymous=True)

	# uav_id = rospy.get_param("~uav_id","uav")
	print(os.getcwd())
	#---------Logfile Setup-------------#
	# populate the data header, these are just strings, you can name them anything
	myData = ["time", "x", "y", "yaw", "vel_x", "yaw_vel"]

	# this creates a filename which contains the current date/time RaspberryPi does not have a real time clock, the files
	# will have the correct sequence (newest to oldest is preserved) but unless you set it explicitely the time will not
	# be the correct (it will not be the "real" time
	# the syntax for the command to set the time is:  bashrc: $ sudo time -s "Mon Aug 26 22:20:00 CDT 2019"
	# note that the path used here is an absolute path, if you want to put the log files somewhere else you will need
	# to include an updated absolute path here to the new directory where you want the files to appear
	fileNameBase = "/home/justin/catkin_ws/src/unmanned_systems/unmanned_systems/scripts/logfiles/" + \
	datetime.datetime.now().strftime("%b_%d_%H_%M")
	fileNameSuffix = ".csv"
	# num is used for incrementing the file path if we already have a file in the directory with the same name
	num = 1
	fileName = fileNameBase + fileNameSuffix
	# check if the file already exists and increment num until the name is unique
	while os.path.isfile(fileName):
		fileName = fileNameBase + "_" + str(num)+"_" + fileNameSuffix
		num = num + 1

	# now we know we have a unique name, let's open the file, 'a' is append mode, in the unlikely event that we open
	# a file that already exists, this will simply add on to the end of it (rather than destroy or overwrite data)
	myFile = open(fileName, 'a')
	with myFile:
		writer = csv.writer(myFile)
		writer.writerow(myData)

	# get the CPU time at which we started the node, we will use this to subtract off so that our time vector
	# starts near 0
	zero_time = rospy.get_time()

	# this is some ros magic to control the loop timing, you can change this to log data faster/slower as needed
	# note that the IMU publisher publishes data at a specified rate (500Hz) and while this number could be
	# changes, in general, you should keep the loop rate for the logger below the loop rate for the IMU publisher
	rate = rospy.Rate(20) #100 Hz
	# try/except block here is a fancy way to allow code to cleanly exit on a keyboard break (ctrl+c)
	try:
		while not rospy.is_shutdown():
			# get the current time and subtract off the zero_time offset
			now = (rospy.get_time()-zero_time)
			# create the data vector which we will write to the file, remember if you change
			# something here, but don't change the header string, your column headers won't
			# match the data
			myData = [now, tbinfo.odom_x, tbinfo.odom_y, tbinfo.yaw, tbinfo.vel_x, tbinfo.vel_yaw]

			# stick everything in the file
			myFile = open(fileName, 'a')
			with myFile:
				writer = csv.writer(myFile)
				writer.writerow(myData)

			# this is ros magic, basically just a sleep function with the specified dt
			rate.sleep()

	# as stated before, try/except is used to nicely quit the program using ctrl+c
	except rospy.ROSInterruptException:
		pass
