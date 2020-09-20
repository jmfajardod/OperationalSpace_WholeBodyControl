#pragma once

#include <iostream> 
#include <algorithm> 
#include <vector>
#include <math.h> 
#include <Eigen/Dense>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/JointState.h>

#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_srvs/SetBool.h>

namespace mobile_odometry {

/*!
 * Class containing the Robotino Odometry class
 */
class MobileOdom {
public:
	/*!
	 * Constructor.
	 */
	MobileOdom(ros::NodeHandle& nodeHandle);
	
	/*!
	 * Destructor.
	 */
	virtual ~MobileOdom();


private:

	// Node handle
	ros::NodeHandle nodeHandle_;

	/* 
	*	Subscribers
	*/
	ros::Subscriber joint_sts_sub_;

	/* 
	*	Publisher
	*/
	ros::Publisher pub_odom;

	/*
	*  Service servers
	*/
	ros::ServiceServer service_reset_odom_;
	ros::ServiceServer service_reset_odom_cov_;

	/*!
	*	TF objects
	*/ 
	tf2_ros::TransformBroadcaster broadcaster_;

	/*!
	 * Parameters 
	*/
	std::string output_rf;
	std::string input_rf;
	std::string robot_name;
	float init_x;
	float init_y;
	float init_theta;

	/*!
	* Variables 
	*/

	geometry_msgs::TransformStamped odom_transform_;
	std::string joint_state_topic ;

	float phi_0_dot;
	float phi_1_dot;
	float phi_2_dot;

	float radius;
	float wheel_sep;

	int  frecuency_rate;
	bool broadcast_tf;

	std::string odometry_topic;
	nav_msgs::Odometry odom_msg;
	float scale;

	// Eigen Variables
	Eigen::MatrixXd J1;
	Eigen::MatrixXd J2;
	Eigen::MatrixXd Jacob;
	Eigen::Matrix3d Rot_mat;
	Eigen::VectorXd x_dot_local;
	Eigen::VectorXd x_dot_global;
	Eigen::VectorXd q_dot;


	/*
	Functions
	*/

	/*!
	* Reads and verifies the ROS parameters.
	* @return true if successful.
	*/
	bool readParameters();

	/*!
	* Callback function for the joint states of the robot
	* @param jointState the states of the three wheels 
	*/
	void Joint_state_Callback_function(const sensor_msgs::JointState jointState);

	/*!
	* Function to refresh publisher and subscribers
	*/
	void spin();

	/*!
	* Service callback function to reset odometry
	* @param req the request of the service
	* @param res the reply of the service
	*/
	bool reset_odometry(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res);

	/*!
	* Service callback function to reset odometry covariance
	* @param req the request of the service
	* @param res the reply of the service
	*/
	bool reset_odom_cov(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res);

};

} /* namespace */
