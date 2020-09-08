#include <ros/ros.h>
#include <osc_hybrid_mob_manipulator/OSC_Hybrid.hpp>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "osc_controller");
    ros::NodeHandle nodeHandle("~");

    osc_hybrid_controller::OscHybridController controller(nodeHandle);
    return 0;
}

