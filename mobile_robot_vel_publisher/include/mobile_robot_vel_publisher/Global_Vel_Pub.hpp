/*******************************************************************************
* Copyright 2020 Jose Manuel Fajardo
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#pragma once

#include <iostream> 
#include <algorithm> 
#include <Eigen/Dense>
#include <math.h> 
#include <vector>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

#include<nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace global_vel_publisher {

/*!
 * Class containing the EKF Filter
 */
class GlobalVelPub {
public:
	/*!
	 * Constructor.
	 */
	GlobalVelPub(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~GlobalVelPub();


private:

	// Node handle
	ros::NodeHandle nodeHandle_;

	/*
	*	Subscribers
	*/
	ros::Subscriber subs_cmd_vel_;
	ros::Subscriber subs_odom_;

	/*
	* Publisher 
	*/
	ros::Publisher pub_command_wheel_0;
    ros::Publisher pub_command_wheel_1;
    ros::Publisher pub_command_wheel_2;

	/*!
	* Variables 
	*/

	ros::Time time_update_odom;

    std::string robot_name;
	std::string robot_frame;
	std::string odom_topic;

    std::string wheel_0_name;
    std::string wheel_1_name;
    std::string wheel_2_name;

    std::string wheel_0_command_topic;
    std::string wheel_1_command_topic;
    std::string wheel_2_command_topic;

    double wheel_radius;
    double wheel_sep;


	int odom_rate;

    // Eigen variables
    Eigen::MatrixXd J1;
    Eigen::MatrixXd J2;
    Eigen::MatrixXd Jacob_inv;
	Eigen::Matrix3d Rot_mat;
    Eigen::VectorXd q_dot;
    Eigen::VectorXd x_dot;

	/*
	Functions
	*/

	/*!
	* Function to wrap angle to [-pi,pi]
	* @param x the angle in radians
	*/
	float constrainAngle(float x);
	
	/*!
	*  ROS topic callback
	*  @param msg the cmd_vel received
	*/
	void Cmd_vel_callback(const geometry_msgs::Twist msg);

	/*!
	*  ROS topic callback
	*  @param msg the odometry message received
	*/
	void Odom_callback(const nav_msgs::Odometry msg);
    
    /*!
	* Function to read parameters from server
	* @return if the parameters where correctly loaded
	*/
    bool readParameters();

};

} /* namespace */
