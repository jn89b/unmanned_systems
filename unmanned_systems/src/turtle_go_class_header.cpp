#include <iostream>
#include <turtle_go_class_header.h>

//this is a constructor allows you to construct an object of class  
Turtlebot::Turtlebot(ros::NodeHandle* nh)
{ 
    // declare subscribrers and publishers
    odom_sub = nh->subscribe<nav_msgs::Odometry>
        ("odom", 10, &Turtlebot::odomCallback, this);
    vel_pub = nh->advertise<geometry_msgs::Twist>
                        ("cmd_vel",10);
    initOdomPos();
}

void Turtlebot::initOdomPos()
{
    odom_x = 0;
    odom_y = 0;
    odom_z = 0;
}

void Turtlebot::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;
    odom_z = msg->pose.pose.position.z;
    std::cout<<"odom x: "<<odom_x<<std::endl; 
}

void Turtlebot::go_turn(float turn_speed)
{
    twist.angular.z = turn_speed;
    vel_pub.publish(twist);
}

// this is how to publish a message 
void Turtlebot::go_straight(float linear_speed)
{
    twist.linear.x = linear_speed;
    vel_pub.publish(twist);
}
