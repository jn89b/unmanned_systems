//import cpp packages
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// declare global variables
float odom_x;
float odom_y;
float odom_z;

geometry_msgs::Twist twist;

// this is how you retrieve information back from your sensors
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;
    odom_z = msg->pose.pose.position.z;
    //std::cout<<"odom x: "<<odom_x<<std::endl; 
}

// this is how to publish a message 
void go_straight(ros::Publisher vel_publisher)
{
    twist.linear.x = 0.1;
    vel_publisher.publish(twist);
}

void go_turn(ros::Publisher vel_publisher)
{
    twist.angular.z = 0.1;
    vel_publisher.publish(twist);
}

int main(int argc, char **argv)
{   
    //initiate node 
    ros::init(argc,argv,"turtlebot_go");
    ros::NodeHandle nh;
        
    // declare subscribers and or publishers
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
                    ("odom", 10, &odom_cb);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
                    ("cmd_vel",10);

    //control the rate of data retrieved or published
    ros::Rate rate(20.0);
    
    //main loop iteration
    while (ros::ok())
    {
        //the spin once does the call back for retrieval of info from your sensors
        //ros::spinOnce();
        go_turn(vel_pub);
        rate.sleep();
    }
    return 0;   
}



