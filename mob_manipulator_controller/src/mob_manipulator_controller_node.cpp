#include <ros/ros.h>
#include <mob_manipulator_controller/Mob_Manipulator_Controller.hpp>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "osc_controller");
    ros::NodeHandle nodeHandle("~");

    mob_manipulator_controller::MobManipulatorController controller(nodeHandle);
    return 0;
}

