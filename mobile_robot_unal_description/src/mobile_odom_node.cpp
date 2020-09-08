#include <ros/ros.h>
#include <mobile_robot_unal_description/MobileOdom.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "robotino_odometry");
    ros::NodeHandle nodeHandle("~");

    mobile_odometry::MobileOdom odometry(nodeHandle);
    return 0;
}

