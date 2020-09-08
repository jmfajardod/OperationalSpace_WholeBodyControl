cmake_minimum_required(VERSION 3.0.2)
project(mobile_manipulator_unal_description)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

###########
## Build ##
###########