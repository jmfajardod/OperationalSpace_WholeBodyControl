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

#include <mob_manipulator_controller/Effort_Tasks.hpp>

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

	// Parameters required to load model in DART
	std::vector<std::string> model_packages_paths;
	std::vector<std::string> model_packages_names;
	std::string 			 urdf_model_path;

	/*!
	* Variables 
	*/

	double time_previous_sjs;
	double time_actual_sjs;

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

	Eigen::VectorXd q_dot_zero;
	Eigen::VectorXd q_dot_result;

	/*!
	* DART Objects
	*/

	dart::dynamics::SkeletonPtr dart_robotSkeleton;
	dart::dynamics::BodyNode* mEndEffector_;

	/*!
	 * Instance of OSF controller
	 * 
	*/

	effort_tasks::EffortTask effortSolver_;

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

};

} /* namespace */
