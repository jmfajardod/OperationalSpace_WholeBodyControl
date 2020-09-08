#include <ros/ros.h>
#include <robotino_effort_publisher/Local_Effort_Pub.hpp>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "local_effort_publisher_node");
    ros::NodeHandle nodeHandle("~");

    local_effort_publisher::LocalEffortPub publisher(nodeHandle);
    return 0;
}

