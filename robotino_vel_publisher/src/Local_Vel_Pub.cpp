#include <robotino_vel_publisher/Local_Vel_Pub.hpp>

namespace local_vel_publisher {

////////////////////////////////////////////////////////////////////////////////
// Function to wrap angles to [0,2pi]
float LocalVelPub::constrainAngle(float x){
    // Wrap to [0,2*pi]
    x = fmod(x,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    // Wrap to [-pi,pi]
    x = fmod(x+M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

////////////////////////////////////////////////////////////////////////////////
// Callback function of the cmd vel 
void LocalVelPub::Cmd_vel_callback(const geometry_msgs::Twist msg){

    x_dot(0) = msg.linear.x;
    x_dot(1) = msg.linear.y;
    x_dot(2) = msg.angular.z;

    q_dot = Jacob_inv * x_dot;

    std_msgs::Float64 command0;
    std_msgs::Float64 command1;
    std_msgs::Float64 command2;

    command0.data = q_dot(0);
    command1.data = q_dot(1);
    command2.data = q_dot(2);

    pub_command_wheel_0.publish(command0);
    pub_command_wheel_1.publish(command1);
    pub_command_wheel_2.publish(command2);

    std::cout << "Wheel Velocities: \n" << q_dot << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
LocalVelPub::LocalVelPub(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle),
    J1(Eigen::MatrixXd::Zero(3,3)),
    J2(Eigen::MatrixXd::Identity(3,3)),
    Jacob_inv(Eigen::MatrixXd::Zero(3,3)),
    q_dot(Eigen::VectorXd::Zero(3)),
    x_dot(Eigen::VectorXd::Zero(3))
{
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

    std::cout << "Matrix J1: \n" << J1 << std::endl;

    // Matrix J2
    J2 = wheel_radius*J2;
    std::cout << "Matrix J2: \n" << J2 << std::endl;

    // Inverse Jacobian
    Jacob_inv = J2.inverse() * J1 ;
    std::cout << "Inverse Jacobian: \n" << Jacob_inv << std::endl;

    // Create subscribers
    subs_cmd_vel_= nodeHandle_.subscribe("/"+ robot_name +"/commands/velocity" , 10, 
                                        &LocalVelPub::Cmd_vel_callback, this);

    // Init publisher
    pub_command_wheel_0 = nodeHandle_.advertise<std_msgs::Float64>(wheel_0_command_topic, 10);
    pub_command_wheel_1 = nodeHandle_.advertise<std_msgs::Float64>(wheel_1_command_topic, 10);
    pub_command_wheel_2 = nodeHandle_.advertise<std_msgs::Float64>(wheel_2_command_topic, 10);

    ros::spin();

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
LocalVelPub::~LocalVelPub()
{
}

////////////////////////////////////////////////////////////////////////////////
// Function to load parameters
bool LocalVelPub::readParameters()
{
    if (!nodeHandle_.getParam("robot_name", robot_name))        return false;
    if (!nodeHandle_.getParam("wheel_0_topic", wheel_0_name))   return false;
    if (!nodeHandle_.getParam("wheel_1_topic", wheel_1_name))   return false;
    if (!nodeHandle_.getParam("wheel_2_topic", wheel_2_name))   return false;
    if (!nodeHandle_.getParam("wheel_radius", wheel_radius))   return false;
    if (!nodeHandle_.getParam("wheel_sep", wheel_sep))   return false;

    wheel_0_command_topic = "/" + robot_name + "/" + wheel_0_name + "/command";
    wheel_1_command_topic = "/" + robot_name + "/" + wheel_1_name + "/command";
    wheel_2_command_topic = "/" + robot_name + "/" + wheel_2_name + "/command";
    
    return true;
}


} /* namespace */