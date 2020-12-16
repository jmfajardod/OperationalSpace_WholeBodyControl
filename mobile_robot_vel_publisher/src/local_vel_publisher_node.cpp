#include <ros/ros.h>
#include <mobile_robot_vel_publisher/Local_Vel_Pub.hpp>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "local_vel_publisher_node");
    ros::NodeHandle nodeHandle("~");

    local_vel_publisher::LocalVelPub publisher(nodeHandle);
    return 0;
}

