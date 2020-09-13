#include <mobile_robot_unal_description/MobileOdom.hpp>

namespace mobile_odometry {

////////////////////////////////////////////////////////////////////////////////
// Callback function to obtain the velocity of the wheels

void MobileOdom::Joint_state_Callback_function(const sensor_msgs::JointState jointState){

    int counterJoints = 0;

    for (int ii=0; ii<jointState.name.size() ; ii++){

        //ROS_INFO("Joint Name %s", (jointState.name[ii]).c_str());
        if(jointState.name[ii]=="wheel0_joint"){
            q_dot(0) = jointState.velocity[ii];
            counterJoints += 1;
        }
        if(jointState.name[ii]=="wheel1_joint"){
            q_dot(1) = jointState.velocity[ii];
            counterJoints += 1;
        }
        if(jointState.name[ii]=="wheel2_joint"){
            q_dot(2) = jointState.velocity[ii];
            counterJoints += 1;
        }
    }
    
    if(counterJoints!=3){
        ROS_INFO("Joint velocities NOT found");
        q_dot = Eigen::VectorXd::Zero(3);
    }
    /*
    else{
        ROS_INFO("Joint velocities found");
    }
    */
    //ROS_INFO("Velocities %f %f %f", phi_0_dot, phi_1_dot, phi_2_dot);

}

////////////////////////////////////////////////////////////////////////////////
// Service to reset odometry
bool MobileOdom::reset_odometry(std_srvs::SetBool::Request  &req, 
                                std_srvs::SetBool::Response &res){
    
    // Reset tranformation
    odom_transform_.header.stamp = ros::Time::now();
    odom_transform_.header.frame_id = input_rf;
    odom_transform_.child_frame_id  = output_rf;
    odom_transform_.transform.translation.x = 0.0;
    odom_transform_.transform.translation.y = 0.0;
    odom_transform_.transform.translation.z = 0.0;
    odom_transform_.transform.rotation.x = 0.0;
    odom_transform_.transform.rotation.y = 0.0;
    odom_transform_.transform.rotation.z = 0.0;
    odom_transform_.transform.rotation.w = 1.0;

    // Reset odometry msg 
    odom_msg.header.frame_id = input_rf;
    odom_msg.child_frame_id  = output_rf;

    odom_msg.pose.covariance[0]  = 1e-6;
    odom_msg.pose.covariance[7]  = 1e-6;
    odom_msg.pose.covariance[14] = 1000.0;
    odom_msg.pose.covariance[21] = 1000.0;
    odom_msg.pose.covariance[28] = 1000.0;
    odom_msg.pose.covariance[35] = 1e-6;

    odom_msg.twist.covariance[0]   = 1e-6;
    odom_msg.twist.covariance[7]   = 1e-6;
    odom_msg.twist.covariance[14]  = 1000.0;
    odom_msg.twist.covariance[21]  = 1000.0;
    odom_msg.twist.covariance[28]  = 1000.0;
    odom_msg.twist.covariance[35]  = 1e-6;

    pub_odom.publish(odom_msg);

    res.success = true; 

    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Service to reset odometry covariance
bool MobileOdom::reset_odom_cov(std_srvs::SetBool::Request  &req, 
                                std_srvs::SetBool::Response &res){

    // Odometry msg initialization
    odom_msg.pose.covariance[0]  = 1e-6;
    odom_msg.pose.covariance[7]  = 1e-6;
    odom_msg.pose.covariance[14] = 1000.0;
    odom_msg.pose.covariance[21] = 1000.0;
    odom_msg.pose.covariance[28] = 1000.0;
    odom_msg.pose.covariance[35] = 1e-6;

    odom_msg.twist.covariance[0]   = 1e-6;
    odom_msg.twist.covariance[7]   = 1e-6;
    odom_msg.twist.covariance[14]  = 1000.0;
    odom_msg.twist.covariance[21]  = 1000.0;
    odom_msg.twist.covariance[28]  = 1000.0;
    odom_msg.twist.covariance[35]  = 1e-6;


    res.success = true; 

    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
MobileOdom::MobileOdom(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle), phi_0_dot(0), phi_2_dot(0), phi_1_dot(0),
    radius(0.060), 
    wheel_sep(0.1817),
    frecuency_rate(200),
    scale(0.1),
    J1(Eigen::MatrixXd::Zero(3,3)),
    J2(Eigen::MatrixXd::Identity(3,3)),
    Jacob(Eigen::MatrixXd::Zero(3,3)),
    q_dot(Eigen::VectorXd::Zero(3)),
    x_dot(Eigen::VectorXd::Zero(3))
{
    // Read parameters
    if (!readParameters()) {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    // Create the Jacobian matrix
    // Matrix J1
    double alpha_0 = M_PI_2;
    double beta_0  = 0.0;
    double gamma_0 = 0.0;
    
    J1(0,0) =  sin(alpha_0+beta_0+gamma_0); 
    J1(0,1) = -cos(alpha_0+beta_0+gamma_0);
    J1(0,2) = -wheel_sep*cos(beta_0+gamma_0);

    double alpha_1 = -5*M_PI/6;
    double beta_1  = 0.0;
    double gamma_1 = 0.0;

    J1(1,0) =  sin(alpha_1+beta_1+gamma_1); 
    J1(1,1) = -cos(alpha_1+beta_1+gamma_1);
    J1(1,2) = -wheel_sep*cos(beta_1+gamma_1);

    double alpha_2 = -M_PI/6;
    double beta_2  = 0.0;
    double gamma_2 = 0.0;

    J1(2,0) =  sin(alpha_2+beta_2+gamma_2); 
    J1(2,1) = -cos(alpha_2+beta_2+gamma_2);
    J1(2,2) = -wheel_sep*cos(beta_2+gamma_2);

    //std::cout << "Matrix J1: \n" << J1 << std::endl; // For Debug

    // Matrix J2
    J2 = radius*J2;
    //std::cout << "Matrix J2: \n" << J2 << std::endl; // For Debug

    // Jacobian
    Jacob = J1.inverse() * J2 ;
    //std::cout << "Jacobian: \n" << Jacob << std::endl; // For Debug

    //-----------------------------------------------------------------//
    // Init transform
    odom_transform_.header.stamp = ros::Time::now();
    odom_transform_.header.frame_id = input_rf;
    odom_transform_.child_frame_id  = output_rf;
    odom_transform_.transform.translation.x = init_x;
    odom_transform_.transform.translation.y = init_y;
    odom_transform_.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0,0.0,init_theta);

    odom_transform_.transform.rotation.x = q.getX();
    odom_transform_.transform.rotation.y = q.getY();
    odom_transform_.transform.rotation.z = q.getZ();
    odom_transform_.transform.rotation.w = q.getW();

    // Tf2 broadcaster
    broadcaster_.sendTransform(odom_transform_); // Broadcast transform

    // Odometry msg initialization
    odom_msg.header.frame_id = input_rf;
    odom_msg.child_frame_id  = output_rf;

    odom_msg.pose.covariance[0]  = 1e-6;
    odom_msg.pose.covariance[7]  = 1e-6;
    odom_msg.pose.covariance[14] = 1000.0;
    odom_msg.pose.covariance[21] = 1000.0;
    odom_msg.pose.covariance[28] = 1000.0;
    odom_msg.pose.covariance[35] = 1e-6;

    odom_msg.twist.covariance[0]   = 1e-6;
    odom_msg.twist.covariance[7]   = 1e-6;
    odom_msg.twist.covariance[14]  = 1000.0;
    odom_msg.twist.covariance[21]  = 1000.0;
    odom_msg.twist.covariance[28]  = 1000.0;
    odom_msg.twist.covariance[35]  = 1e-6;

    // Joint states subscriber
    joint_sts_sub_ = nodeHandle_.subscribe(joint_state_topic , 10, 
                                        &MobileOdom::Joint_state_Callback_function, this);

    // Odometry topic publisher
    pub_odom= nodeHandle_.advertise<nav_msgs::Odometry>(odometry_topic, 10);

    // Reset odometry service
    service_reset_odom_ = nodeHandle.advertiseService("reset_odom", &MobileOdom::reset_odometry, this );
    
    // Reset odometry covariance
    service_reset_odom_cov_ = nodeHandle.advertiseService("reset_odom_cov", &MobileOdom::reset_odom_cov, this );

    // Call the spin function of this class
    spin();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
MobileOdom::~MobileOdom()
{
}

////////////////////////////////////////////////////////////////////////////////
// Function to load values from the parameter server
bool MobileOdom::readParameters()
{
    if (!nodeHandle_.getParam("output_frame", output_rf))   return false;
    if (!nodeHandle_.getParam("input_frame", input_rf))     return false;
    if (!nodeHandle_.getParam("robot_name", robot_name))     return false;

    if (!nodeHandle_.getParam("initial_x", init_x))             return false;
    if (!nodeHandle_.getParam("initial_y", init_y))             return false;
    if (!nodeHandle_.getParam("initial_theta", init_theta))     return false;

    if (!nodeHandle_.getParam("loop_rate", frecuency_rate))     return false;

    output_rf = robot_name + "/" + output_rf;  

    joint_state_topic = "/" + robot_name + "/joint_states";
    ROS_INFO("Joint State topic %s",joint_state_topic.c_str());

    odometry_topic = "/" + robot_name + "/odom";

    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Function to publish topic at rate
void MobileOdom::spin()
{   
    // Create loop rate object
    ros::Rate loop_rate(frecuency_rate);

    while (ros::ok())
    {  
        // Find the angle of rotation
        double roll, pitch, yaw;
        tf2::Quaternion q;
        tf2::Matrix3x3 rot_matrix;

        q.setX(odom_transform_.transform.rotation.x);
        q.setY(odom_transform_.transform.rotation.y);
        q.setZ(odom_transform_.transform.rotation.z);
        q.setW(odom_transform_.transform.rotation.w);

        rot_matrix.setRotation(q);
        rot_matrix.getRPY(roll, pitch, yaw);

        Rot_mat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

        //std::cout << "Rotation Matrix: \n" << Rot_mat << std::endl; // For Debug

        x_dot = Jacob * q_dot; // q_dot in Robot(Local) Reference frame
        x_dot =  Rot_mat * x_dot; // q_dot in Global Reference frame

        ROS_INFO("VEL_I: \n %f \n %f \n %f \n", x_dot(0),x_dot(1),x_dot(2));

        // Find new values of X, Y and Theta
        double step_time = 1.0/double(frecuency_rate);
        double new_x, new_y, new_theta;
        ROS_INFO("Step Time Odometry: %f", step_time);
        
        //-----------------------------------------------------//
        // Check that x velocity is greater than 0.005 m/s
        if(abs(x_dot(0))>=0.005){  
            new_x = odom_transform_.transform.translation.x + x_dot(0)*step_time;
        }
        else{
            new_x = odom_transform_.transform.translation.x;
        }
        //-----------------------------------------------------//
        // Check that y velocity is greater than 0.005 m/s
        if(abs(x_dot(1))>=0.005){
            new_y = odom_transform_.transform.translation.y + x_dot(1)*step_time;
        }
        else{
            new_y = odom_transform_.transform.translation.y;
        }
        
        new_theta = yaw + x_dot(2)*step_time;

        //ROS_INFO("New %f %f %f", new_x, new_y, new_theta);

        //----------------------------------------------------------------------------//
        // Update Odometry 

        rot_matrix.setRPY(0,0,new_theta);
        rot_matrix.getRotation(q);
        
        // Create transformation matrix
        tf2::Transform tr_mat;
        tf2::Vector3 vec;
        vec.setX(new_x);
        vec.setY(new_y);
        vec.setZ(0.0);

        tr_mat.setOrigin(vec);
        tr_mat.setRotation(q);

        // Change values in the transform
        odom_transform_.header.stamp = ros::Time::now();
        odom_transform_.transform.translation.x = tr_mat.getOrigin().getX();
        odom_transform_.transform.translation.y = tr_mat.getOrigin().getY();
        odom_transform_.transform.translation.z = 0.0;
        odom_transform_.transform.rotation.x = tr_mat.getRotation().getX();
        odom_transform_.transform.rotation.y = tr_mat.getRotation().getY();
        odom_transform_.transform.rotation.z = tr_mat.getRotation().getZ();
        odom_transform_.transform.rotation.w = tr_mat.getRotation().getW();

        // Broadcast tf from robot to odom
        broadcaster_.sendTransform(odom_transform_);

        // Odometry message 
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.pose.pose.position.x = tr_mat.getOrigin().getX();
        odom_msg.pose.pose.position.y = tr_mat.getOrigin().getY();
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = tr_mat.getRotation().getX();
        odom_msg.pose.pose.orientation.y = tr_mat.getRotation().getY();
        odom_msg.pose.pose.orientation.z = tr_mat.getRotation().getZ();
        odom_msg.pose.pose.orientation.w = tr_mat.getRotation().getW();

        odom_msg.pose.covariance[0]  += scale*abs(x_dot(0))*step_time;
        odom_msg.pose.covariance[7]  += scale*abs(x_dot(1))*step_time;
        odom_msg.pose.covariance[35] += scale*abs(x_dot(2))*step_time;

        odom_msg.twist.twist.linear.x  = x_dot(0);
        odom_msg.twist.twist.linear.y  = x_dot(1);
        odom_msg.twist.twist.linear.z  = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = x_dot(2);

        odom_msg.twist.covariance[0]  = scale*abs(x_dot(0));
        odom_msg.twist.covariance[7]  = scale*abs(x_dot(1));
        odom_msg.twist.covariance[35] = scale*abs(x_dot(2));

        pub_odom.publish(odom_msg);
        
        // Set time and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::requestShutdown();
}

} /* namespace */