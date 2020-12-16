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
#include <Eigen/Geometry> 
#include <math.h> 
#include <vector>

#include <ros/ros.h>
#include <angles/angles.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

#include <nav_msgs/Odometry.h>

#include <mobile_manipulator_msgs/MobileManipulator.h>
#include <mobile_manipulator_msgs/Trajectory.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <osc_controller/OSC_Controller.hpp>


namespace mob_manipulator_controller {

/*!
 * Class containing the controller for the mobile manipulator
 */
class MobManipulatorController {
public:
	/*!
	 * Constructor.
	 */
	MobManipulatorController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~MobManipulatorController();


private:

	// Node handle
	ros::NodeHandle nodeHandle_;

	/*
	*	Subscribers
	*/
	ros::Subscriber subs_joint_state_;
	ros::Subscriber subs_odometry_;
	ros::Subscriber subs_desired_traj_;
	
	/*
	* Publisher 
	*/
	ros::Publisher pub_mobile_platfrm;

	ros::Publisher pub_manipulator_dof_1;
	ros::Publisher pub_manipulator_dof_2;
	ros::Publisher pub_manipulator_dof_3;
	ros::Publisher pub_manipulator_dof_4;
	ros::Publisher pub_manipulator_dof_5;
	ros::Publisher pub_manipulator_dof_6;

	ros::Publisher pub_mob_manipulator_data;

	/*
	* TF Broadcaster
	*/

	tf2_ros::TransformBroadcaster broadcaster_;
	geometry_msgs::TransformStamped transformDesiredPos;

	/*!
	* Parameters 
	*/

	// Robot related parameters
    std::string robot_name;
	std::string robot_frame;
	int robot_dofs;

	// Maximum velocities when using the controllers: Go to point with constant vel
	double point_const_vel_max_lineal;
	double point_const_vel_max_angular;

	// Cartesian gains
	double pos_stiff_X, pos_stiff_Y, pos_stiff_Z;
	double pos_damp_X, pos_damp_Y, pos_damp_Z;

	double ori_stiff_X, ori_stiff_Y, ori_stiff_Z;
	double ori_damp_X, ori_damp_Y, ori_damp_Z;

	double mobile_base_p_gain, mobile_base_d_gain;
	double manipulator_p_gain, manipulator_d_gain;

	// Variable to define if only using manipulator (For testing purposes)
	bool using_only_manipulator;

	// Odometry topic
	std::string odometry_topic;

	// Parameters of manipulator joints
	std::vector<std::string> manipulator_dofs; // Name of joints in URDF model
	std::vector<std::string> manipulator_controllers; // Name of joint controllers

	// Max frequency of control loop
	int frecuency_rate;

	// Parameters of controller
	bool topdown_;
    bool jtspace_;
    bool augmented_;

	int method_sing_handling;
	int method_joint_limit_avoidance;

	int orient_error_calc;

	// Parameters required to load model in DART
	std::vector<std::string> model_packages_paths;
	std::vector<std::string> model_packages_names;
	std::string 			 urdf_model_path;

	/*!
	* Variables 
	*/

	double time_previous_sjs;
	double time_actual_sjs;

	// Variable for minimum singular values
	double min_sv_pos;
	double min_sv_ori;

	// Variable to publish commands of manipulator in topic
	std_msgs::Float64 manipulator_cmd;

	// Variable to publish commands of mobile platform in topic
	geometry_msgs::Twist mobile_pltfrm_cmd;

	// Variable to publish infomation about robot in topic
	mobile_manipulator_msgs::MobileManipulator mobile_manipulator_data;

	// Variable to obtain the trajectory to follow
	mobile_manipulator_msgs::Trajectory mob_man_traj;

    /*!
	* Eigen Variables 
	*/

	Eigen::MatrixXd M;
	Eigen::VectorXd C_k;
	Eigen::VectorXd g_k;

	Eigen::VectorXd q_k;
    Eigen::VectorXd q_dot_k;

	Eigen::VectorXd tau_zero;
	Eigen::VectorXd tau_result;
	Eigen::VectorXd tau_joints;
	Eigen::VectorXd tau_ext;

	Eigen::VectorXd q_dot_zero;
	Eigen::VectorXd q_dot_result;

	// Variables for trajectory targets
	Eigen::Vector3d targetCartPos;
	Eigen::Vector3d targetCartVel; 
	Eigen::Vector3d targetCartAccel;

	Eigen::Matrix3d targetOrientPos;
	Eigen::Vector3d targetOrientVel; 
	Eigen::Vector3d targetOrientAccel;

	Eigen::VectorXd q_desired;

	/*!
	* DART Objects
	*/

	dart::dynamics::SkeletonPtr dart_robotSkeleton;
	dart::dynamics::BodyNode* mEndEffector_;

	/*!
	 * Instance of OSF controller
	 * 
	*/

	osc_controller::OSC_Controller osc_controller_;

	/******************************************************************/
	/******************************************************************/

	/*
	Functions
	*/

	/*!
	*  ROS topic callback
	*  @param msg the joint states msg received
	*/
	void jointState_CB(const sensor_msgs::JointState msg);

	/*!
	*  ROS topic callback
	*  @param msg the odometry msg received
	*/
	void odometry_CB(const nav_msgs::Odometry msg);

	void change_DesPose_CB(const mobile_manipulator_msgs::Trajectory msg);

    /*!
	* Function to read parameters from server
	* @return if the parameters where correctly loaded
	*/
    bool readParameters();

	/*!
	* Function to refresh publisher and subscribers
	*/
	void spin();

	/*!
	* Function to load the robot in DART
	*/
    void loadDARTModel();

	/*!
	* Function to initialize the msgs
	*/
    void initMsgs();

	/*!
	* Function to calculate torque due to tasks
	*/
    void calcTorqueDueTasks();

	/*!
	* Function to define the stack of tasks
	*/
	void StackTasks(Eigen::MatrixXd *Null_space, Eigen::VectorXd *torque_ns, int cycle);

	/*!
	* Function to update target variables based on data received from topic
	*/
	void updateTarget(Eigen::VectorXd current_joint_pos);

};

} /* namespace */
