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

    //public methods to access private members
    public:
        //this is a constructor allows you to construct an object of class  
        Turtlebot(ros::NodeHandle *nh)
        { 
            // declare subscribrers and publishers
            odom_sub = nh->subscribe<nav_msgs::Odometry>
                ("odom", 10, &Turtlebot::odomCallback, this);
            vel_pub = nh->advertise<geometry_msgs::Twist>
                                ("cmd_vel",10);
            initOdomPos();

        }

        void initOdomPos()
        {
            odom_x = 0;
            odom_y = 0;
            odom_z = 0;
        }

        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            odom_x = msg->pose.pose.position.x;
            odom_y = msg->pose.pose.position.y;
            odom_z = msg->pose.pose.position.z;
            std::cout<<"odom x: "<<odom_x<<std::endl; 
        }

        void go_turn(float turn_speed)
        {
            twist.angular.z = turn_speed;
            vel_pub.publish(twist);
        }

        // this is how to publish a message 
        void go_straight(float linear_speed)
        {
            twist.linear.x = linear_speed;
            vel_pub.publish(twist);
        }

};
 

int main(int argc, char **argv)
{
    ros::init(argc,argv,"turtlebot_go_class");
    //instantiating a class -- creating an object or instance of a class
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    Turtlebot turtlebot = Turtlebot(&nh); // pass nodehandle reference
    
    while(ros::ok())
    {
        turtlebot.go_straight(0.2);
        rate.sleep();
    }
    
}