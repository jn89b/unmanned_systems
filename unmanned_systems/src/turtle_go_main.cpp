#include <iostream>
#include <ros/ros.h>
#include <turtle_go_class_header.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"turtlebot_go_main");
    //instantiating a class -- creating an object or instance of a class
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    Turtlebot turtlebot(&nh); // pass nodehandle reference
    
    while(ros::ok())
    {
        turtlebot.go_straight(0.2);
        rate.sleep();
    }
    
    return 0; 
}