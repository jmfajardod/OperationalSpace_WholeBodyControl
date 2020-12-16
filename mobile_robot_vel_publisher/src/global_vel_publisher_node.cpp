#include <ros/ros.h>
#include <mobile_robot_vel_publisher/Global_Vel_Pub.hpp>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "global_vel_publisher_node");
    ros::NodeHandle nodeHandle("~");

    global_vel_publisher::GlobalVelPub globalPub(nodeHandle);
    return 0;
}

