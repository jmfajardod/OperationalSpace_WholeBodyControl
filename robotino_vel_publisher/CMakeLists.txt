cmake_minimum_required(VERSION 3.0.2)
project(robotino_vel_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
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
    robotino_vel_publisher
  CATKIN_DEPENDS 
    geometry_msgs 
    roscpp 
    std_msgs
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
#---------------------------------------------------------------#
#---------------------------------------------------------------#
# Local Velocity Publisher
add_executable(local_vel_pub_node
  src/local_vel_publisher_node.cpp
  src/Local_Vel_Pub.cpp
)

add_dependencies(local_vel_pub_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(local_vel_pub_node
  ${catkin_LIBRARIES}
)

#---------------------------------------------------------------#
#---------------------------------------------------------------#
# Global Velocity Publisher
add_executable(global_vel_pub_node
  src/global_vel_publisher_node.cpp
  src/Global_Vel_Pub.cpp
)

add_dependencies(global_vel_pub_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(global_vel_pub_node
  ${catkin_LIBRARIES}
)