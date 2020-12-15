cmake_minimum_required(VERSION 3.0.2)
project(mob_manipulator_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs 
  nav_msgs
  tf2
  tf2_ros
  mobile_manipulator_msgs
)
find_package(Eigen3 REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    mob_manipulator_controller
  CATKIN_DEPENDS 
    roscpp 
    rospy
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    tf2
    tf2_ros
    mobile_manipulator_msgs
  DEPENDS 
    Eigen3
)

###########
## Build ##
###########

#-----------------------#
# Include Eigen3 Libraries
set(Eigen3_INCLUDE_DIRS /usr/include/eigen3/)
include_directories(${Eigen3_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#-----------------------#
# Include DART Libraries

set(required_components_dart utils-urdf gui-osg)
set(required_libraries_dart dart dart-utils-urdf dart-gui-osg)

#if(DART_IN_SOURCE_BUILD)
#  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries_dart})
#  return()
#endif()

find_package(DART 6.6.0 REQUIRED COMPONENTS ${required_components_dart} CONFIG)

#---------------------------------------------------------------#
#---------------------------------------------------------------#
# OSC Program

add_executable(mob_manipulator_controller
  src/mob_manipulator_controller_node.cpp
  src/Mob_Manipulator_Controller.cpp
  src/Load_model.cpp
  src/Tasks/Effort_Tasks.cpp
  src/Tasks/Position/Position_Manipulator.cpp
  src/Tasks/Position/Position_Mobile_Robot.cpp
  src/Tasks/Position/Position_Mob_Manipulator.cpp
  src/Tasks/Orientation/Orientation_Manipulator.cpp
  src/Tasks/Orientation/Orientation_Mob_Manipulator.cpp
  src/Tasks/Orientation/Error_functions.cpp
  src/Tasks/Joint_limits/Repulsive_potencials.cpp
  src/Tasks/Joint_limits/Intermediate_value.cpp
  src/Tasks/Joint_limits/Saturation_joint_space.cpp
)

add_dependencies(mob_manipulator_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(mob_manipulator_controller
  ${catkin_LIBRARIES}
  ${required_libraries_dart}
)
