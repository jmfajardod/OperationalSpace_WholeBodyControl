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
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

namespace local_effort_publisher {

/*!
 * Class containing the EKF Filter
 */
class LocalEffortPub {
public:
	/*!
	 * Constructor.
	 */
	LocalEffortPub(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~LocalEffortPub();


private:

	// Node handle
	ros::NodeHandle nodeHandle_;

	/*
	*	Subscribers
	*/
	ros::Subscriber subs_cmd_vel_;
	
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
    std::string wheel_0_name;
    std::string wheel_1_name;
    std::string wheel_2_name;
    std::string wheel_0_command_topic;
    std::string wheel_1_command_topic;
    std::string wheel_2_command_topic;

    double wheel_radius;
    double wheel_sep;

    // Eigen variables
    Eigen::MatrixXd J1;
    Eigen::MatrixXd J2;
    Eigen::MatrixXd Jacob;
    Eigen::MatrixXd Jacob_trans;
    Eigen::VectorXd tau;
    Eigen::VectorXd f;

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
	void Cmd_Wrench_callback(const geometry_msgs::Wrench msg);
    
    /*!
	* Function to read parameters from server
	* @return if the parameters where correctly loaded
	*/
    bool readParameters();
};

} /* namespace */
