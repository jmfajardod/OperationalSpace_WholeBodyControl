#include <ros/ros.h>
#include <robotino_effort_publisher/Global_Effort_Pub.hpp>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "global_effort_publisher_node");
    ros::NodeHandle nodeHandle("~");

    global_effort_publisher::GlobalEffortPub globalPub(nodeHandle);
    return 0;
}

