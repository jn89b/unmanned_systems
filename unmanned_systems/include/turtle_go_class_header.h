#ifndef NULL_TURTLE_GO_CLASS_HEADER_H
#define TURTLE_GO_CLASS_HEADER_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class Turtlebot {

    private:
        float odom_x;
        float odom_y;
        float odom_z;
        ros::Subscriber odom_sub; 
        ros::Publisher vel_pub;
        geometry_msgs::Twist twist;

    public:
        Turtlebot(ros::NodeHandle* nh);
        void initOdomPos();
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void go_turn(float turn_speed);
        void go_straight(float linear_speed);
};

#endif