cmake_minimum_required(VERSION 3.0.2)
project(mobile_robot_unal_description)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
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
    mobile_robot_unal_description
  CATKIN_DEPENDS 
    controller_manager
    roscpp
    sensor_msgs
    std_msgs
    nav_msgs
    tf2
    tf2_ros
  DEPENDS 
    Eigen3
)

###########
## Build ##
###########

# Include Eigen3 Libraries
set(Eigen3_INCLUDE_DIRS /usr/include/eigen3/)
include_directories(${Eigen3_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry_node
  src/mobile_odom_node.cpp
  src/MobileOdom.cpp
)

add_dependencies(odometry_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(odometry_node
  ${catkin_LIBRARIES})
