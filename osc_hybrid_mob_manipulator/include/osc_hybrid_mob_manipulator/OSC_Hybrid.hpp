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

#include <sensor_msgs/JointState.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <osc_hybrid_mob_manipulator/Effort_Tasks.hpp>

#include <osc_hybrid_mob_manipulator/Velocity_Tasks.hpp>

namespace osc_hybrid_controller {

/*!
 * Class containing the Operational Space Hybrid Controller
 */
class OscHybridController {
public:
	/*!
	 * Constructor.
	 */
	OscHybridController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~OscHybridController();


private:

	// Node handle
	ros::NodeHandle nodeHandle_;

	/*
	*	Subscribers
	*/
	ros::Subscriber subs_joint_state_;
	ros::Subscriber subs_odometry_;
	
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

	/*!
	* Variables 
	*/

    std::string robot_name;
	std::string robot_frame;

	std::string odometry_topic;

	std::vector<std::string> manipulator_dofs;
	std::vector<std::string> manipulator_controllers;

	int frecuency_rate;

	std_msgs::Float64 manipulator_cmd;

	geometry_msgs::Twist mobile_pltfrm_cmd;

	bool transition_state;
	double init_trans_time;
	double transition_period;
	double transition_omega;
	int  state;
	
	bool received_joint_state;
	bool received_odometry;

    // Eigen variables
	Eigen::MatrixXd M;
	Eigen::VectorXd C_k;
	Eigen::VectorXd g_k;

	Eigen::VectorXd q_k;
    Eigen::VectorXd q_dot_k;

	Eigen::VectorXd transition_q_dot;
	Eigen::VectorXd transition_tau;

	Eigen::VectorXd tau_zero;
	Eigen::VectorXd tau_result;

	Eigen::VectorXd q_dot_zero;
	Eigen::VectorXd q_dot_result;

	/*!
	* DART Objects
	*/
	dart::dynamics::SkeletonPtr dart_robotSkeleton;
	dart::dynamics::BodyNode* mEndEffector_;

	/*!
	 * Objects to obtain efforts and velocities
	 * 
	*/
	effort_tasks::EffortTask effortSolver_;
	vel_tasks::VelocityTask  velSolver_;

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
	*  @param msg the joint states msg received
	*/
	void jointState_CB(const sensor_msgs::JointState msg);

	/*!
	*  ROS topic callback
	*  @param msg the odometry msg received
	*/
	void odometry_CB(const nav_msgs::Odometry msg);

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

};

} /* namespace */
