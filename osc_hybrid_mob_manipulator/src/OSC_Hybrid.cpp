#include <osc_hybrid_mob_manipulator/OSC_Hybrid.hpp>

namespace osc_hybrid_controller {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

using namespace effort_tasks;

////////////////////////////////////////////////////////////////////////////////
// Function to wrap angles to [-pi,pi]
float OscHybridController::constrainAngle(float x){
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
// Callback function of the joint states
void OscHybridController::jointState_CB(const sensor_msgs::JointState msg){

    for (int ii = 0; ii < msg.position.size(); ii++){
        
        if(msg.name.at(ii)==manipulator_dofs.at(0)){
            q_k(3) = msg.position.at(ii);
            q_dot_k(3) = msg.velocity.at(ii);
        }
        if(msg.name.at(ii)==manipulator_dofs.at(1)){
            q_k(4) = msg.position.at(ii);
            q_dot_k(4) = msg.velocity.at(ii);
        }
        if(msg.name.at(ii)==manipulator_dofs.at(2)){
            q_k(5) = msg.position.at(ii);
            q_dot_k(5) = msg.velocity.at(ii);
        }
        if(msg.name.at(ii)==manipulator_dofs.at(3)){
            q_k(6) = msg.position.at(ii);
            q_dot_k(6) = msg.velocity.at(ii);
        }
        if(msg.name.at(ii)==manipulator_dofs.at(4)){
            q_k(7) = msg.position.at(ii);
            q_dot_k(7) = msg.velocity.at(ii);
        }
        if(msg.name.at(ii)==manipulator_dofs.at(5)){
            q_k(8) = msg.position.at(ii);
            q_dot_k(8) = msg.velocity.at(ii);
        }

    }    
    
    received_joint_state = true;

    /******************************/
    //std::cout << "Joint Pos: \n" << q_k << std::endl;
    //std::cout << "Joint Vel: \n" << q_dot_k << std::endl;

    return;
}
////////////////////////////////////////////////////////////////////////////////
// Callback function of the odometry
void OscHybridController::odometry_CB(const nav_msgs::Odometry msg){

    tf2::Quaternion q;

    q.setX(msg.pose.pose.orientation.x);
    q.setY(msg.pose.pose.orientation.y);
    q.setZ(msg.pose.pose.orientation.z);
    q.setW(msg.pose.pose.orientation.w);

    tf2::Matrix3x3 rot_matrix;
    double roll, pitch, yaw;
    rot_matrix.setRotation(q);
    rot_matrix.getRPY(roll, pitch, yaw);
    

    q_k(0) = msg.pose.pose.position.x;
    q_k(1) = msg.pose.pose.position.y;
    q_k(2) = yaw;

    q_dot_k(0) = msg.twist.twist.linear.x;
    q_dot_k(1) = msg.twist.twist.linear.y;
    q_dot_k(2) = msg.twist.twist.angular.z;

    received_odometry = true;

    /******************************/
    //std::cout << "Joint Pos: \n" << q_k << std::endl;
    //std::cout << "Joint Vel: \n" << q_dot_k << std::endl;

    return;

}

////////////////////////////////////////////////////////////////////////////////
// Constructor
OscHybridController::OscHybridController(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle),
    frecuency_rate(1000),
    q_k(Eigen::VectorXd::Zero(9)),
    q_dot_k(Eigen::VectorXd::Zero(9)),
    tau_zero(Eigen::VectorXd::Zero(9)),
    tau_result(Eigen::VectorXd::Zero(9)),
    transition_tau(Eigen::VectorXd::Zero(9)),
    q_dot_result(Eigen::VectorXd::Zero(9)),
    q_dot_zero(Eigen::VectorXd::Zero(9)),
    transition_q_dot(Eigen::VectorXd::Zero(9)),
    state(0),
    transition_state(false),
    transition_period(0.5),
    received_joint_state(false),
    received_odometry(false)
{   
    /******************************/
    // Load parameters
    if (!readParameters()) {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    transition_omega = (2*M_PI)/(2*transition_period);
    
    /******************************/
    // Init messages

    mobile_pltfrm_cmd.linear.x  = 0.0;
    mobile_pltfrm_cmd.linear.y  = 0.0;
    mobile_pltfrm_cmd.linear.z  = 0.0;
    mobile_pltfrm_cmd.angular.x = 0.0;
    mobile_pltfrm_cmd.angular.y = 0.0;
    mobile_pltfrm_cmd.angular.z = 0.0;

    /******************************/
    // Create subscribers
    subs_joint_state_= nodeHandle_.subscribe("/"+ robot_name +"/joint_states" , 10, 
                                        &OscHybridController::jointState_CB, this);
    subs_odometry_   = nodeHandle_.subscribe("/"+ robot_name +"/" + odometry_topic , 10, 
                                        &OscHybridController::odometry_CB, this);
    
    
    /******************************/
    // Create publishers
    pub_mobile_platfrm  = nodeHandle_.advertise<geometry_msgs::Twist>("/"+ robot_name +"/commands/velocity" , 10);

    pub_manipulator_dof_1 = nodeHandle_.advertise<std_msgs::Float64>("/"+ robot_name +"/"+ manipulator_controllers.at(0) +"/command" , 10);
    pub_manipulator_dof_2 = nodeHandle_.advertise<std_msgs::Float64>("/"+ robot_name +"/"+ manipulator_controllers.at(1) +"/command" , 10);
    pub_manipulator_dof_3 = nodeHandle_.advertise<std_msgs::Float64>("/"+ robot_name +"/"+ manipulator_controllers.at(2) +"/command" , 10);
    pub_manipulator_dof_4 = nodeHandle_.advertise<std_msgs::Float64>("/"+ robot_name +"/"+ manipulator_controllers.at(3) +"/command" , 10);
    pub_manipulator_dof_5 = nodeHandle_.advertise<std_msgs::Float64>("/"+ robot_name +"/"+ manipulator_controllers.at(4) +"/command" , 10);
    pub_manipulator_dof_6 = nodeHandle_.advertise<std_msgs::Float64>("/"+ robot_name +"/"+ manipulator_controllers.at(5) +"/command" , 10);
    
    /******************************/
    // Load DART Object
    loadDARTModel();

    /******************************/
    // Call the spin function
    spin();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
OscHybridController::~OscHybridController()
{
}

////////////////////////////////////////////////////////////////////////////////
// Function to load parameters
bool OscHybridController::readParameters()
{
    if (!nodeHandle_.getParam("robot_name", robot_name))        return false;
    if (!nodeHandle_.getParam("odom_topic", odometry_topic))        return false;
    if (!nodeHandle_.getParam("manipulator_dof_names", manipulator_dofs))        return false;
    if (!nodeHandle_.getParam("manipulator_controllers", manipulator_controllers))        return false;

    robot_frame = robot_name + "/mobile_base_link";

    ROS_INFO("Robot Frame: %s", robot_frame.c_str());

    ROS_INFO("Robot dof1: %s", manipulator_dofs.at(0).c_str());
    
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Spin function
void OscHybridController::spin(){

    // Create loop rate object
    ros::Rate loop_rate(frecuency_rate);

    while (ros::ok()){  
        
        // To send commands at least one type of data has to have been received
        if(received_joint_state || received_odometry){
            
            received_joint_state = false;
            received_odometry = false;

            //std::cout << "Joint Pos: \n" << q_k << std::endl;
            //std::cout << "Joint Vel: \n" << q_dot_k << std::endl;   

            /******************************/
            // Update state of the DART Model

            dart_robotSkeleton->setPositions(q_k);
            dart_robotSkeleton->setVelocities(q_dot_k);

            M = dart_robotSkeleton->getMassMatrix();  // Mass Matrix
            //std::cout << "Mass Matrix: \n" << M << std::endl;

            C_k = dart_robotSkeleton->getCoriolisForces();  // Coriolis vector forces
            g_k = dart_robotSkeleton->getGravityForces();   // Gravity vector forces
            //std::cout << "Coriolis vector forces: \n" << C_k << std::endl;
            //std::cout << "Gravity vector forces: \n" << g_k << std::endl;
            
            /******************************/
            // Target definition

            // Cartesian Target
            Eigen::Vector3d targetPos = Eigen::Vector3d::Zero(); 
            targetPos << 3.0, 3.5, 0.737; //0.54, 0.001, 0.737;

            // Desired rotation matrix
            Eigen::Matrix3d R_world_desired = Eigen::MatrixXd::Zero(3,3);
            R_world_desired = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()); //Eigen::MatrixXd::Identity(3, 3);

            // Joint Conf Desired
            Eigen::VectorXd q_desired = Eigen::VectorXd::Zero(9);
            q_desired(0) = q_k(0);
            q_desired(1) = q_k(1);
            q_desired(2) = q_k(2);

            /******************************/
            // Task Definition for effort

            tau_result = Eigen::VectorXd::Zero(9);
            Eigen::MatrixXd Null_space = Eigen::MatrixXd::Identity(9,9);

            std::cout << "Initial tau: \n" << tau_result << std::endl;
            std::cout << "Initial Null space: \n" << Null_space << std::endl;

            q_desired = q_k;

            effortSolver_.AvoidJointLimits(M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
            std::cout << "Tau result after avoid joint limits: \n" << tau_result << std::endl;
            std::cout << "Null space after avoid joint limits: \n" << Null_space << std::endl;

            //effortSolver_.AchieveCartesian(targetPos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
            //std::cout << "Tau result after achieve cart pos: \n" << tau_result << std::endl;
            //std::cout << "Null space after achieve cart pos: \n" << Null_space << std::endl;

            effortSolver_.MakeStraightLine(targetPos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
            std::cout << "Tau result after straight line: \n" << tau_result << std::endl;
            std::cout << "Null space after straight line: \n" << Null_space << std::endl;

            effortSolver_.AchieveOrientationQuat3(R_world_desired, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space); 
            std::cout << "Tau result after achieve orient: \n" << tau_result << std::endl;
            std::cout << "Null space after achieve orient: \n" << Null_space << std::endl;

            effortSolver_.AchieveJointConf(q_desired, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
            std::cout << "Tau result after achieve joint: \n" << tau_result << std::endl;
            
            //std::cout << "Tau result : \n" << tau_result << std::endl;

            /******************************/
            // Admittance controller
            // send_vel -> q_dot_result
            
            Eigen::MatrixXd damp_des_   = 100.0*Eigen::MatrixXd::Identity(3, 3);
            Eigen::MatrixXd inertia_des = 1.0*Eigen::MatrixXd::Identity(3, 3);

            damp_des_ = damp_des_.inverse();

            Eigen::VectorXd mob_base_tor = Eigen::VectorXd::Zero(3);
            Eigen::VectorXd mob_base_vel = Eigen::VectorXd::Zero(3);

            mob_base_tor << tau_result(0), tau_result(1), tau_result(2);
            //std::cout << "Tau mobile base : \n" << mob_base_tor << std::endl;

            mob_base_vel = damp_des_*mob_base_tor - damp_des_*inertia_des*Eigen::VectorXd::Zero(3);

            q_dot_result(0) = mob_base_vel(0);
            q_dot_result(1) = mob_base_vel(1);
            q_dot_result(2) = mob_base_vel(2);

            //std::cout << "Vel Command: \n" << q_dot_result << std::endl;

            /******************************/
            // Limit efforts

            for (size_t ii = 0; ii < 9; ii++){
                if(ii<3){ // Mobile platform efforts
                    if( abs(tau_result(ii)) > 10.0)  tau_result(ii) = tau_result(ii) * (10.0/abs(tau_result(ii)));
                }
                else{    // Manipulator efforts
                    if( abs(tau_result(ii)) > 10.0)  tau_result(ii) = tau_result(ii) * (10.0/abs(tau_result(ii)));
                }
            }

            //std::cout << "Limited Efforts: \n" << tau_result << std::endl;

            /******************************/
            // Limit Velocities

            for (size_t ii = 0; ii < 9; ii++){
                if(ii<2){ // Mobile platform efforts linear vel
                    if( abs(q_dot_result(ii)) > 0.7 )  q_dot_result(ii) = q_dot_result(ii) * (0.7/abs(q_dot_result(ii)));
                }
                else if(ii<3){ // Mobile platform angular vel
                    if( abs(q_dot_result(ii)) > 1.75 )  q_dot_result(ii) = q_dot_result(ii) * (1.75/abs(q_dot_result(ii)));
                }
            }

            //std::cout << "Limited Vel: \n" << q_dot_result << std::endl;

            /******************************/
            // Publish commands to mobile platform

            mobile_pltfrm_cmd.linear.x  = q_dot_result(0);
            mobile_pltfrm_cmd.linear.y  = q_dot_result(1);
            mobile_pltfrm_cmd.angular.z = q_dot_result(2);

            pub_mobile_platfrm.publish(mobile_pltfrm_cmd);

            /******************************/
            // Publish commands to manipulator

            manipulator_cmd.data = tau_result(3);
            pub_manipulator_dof_1.publish(manipulator_cmd);

            manipulator_cmd.data = tau_result(4);
            pub_manipulator_dof_2.publish(manipulator_cmd);

            manipulator_cmd.data = tau_result(5);
            pub_manipulator_dof_3.publish(manipulator_cmd);

            manipulator_cmd.data = tau_result(6);
            pub_manipulator_dof_4.publish(manipulator_cmd);

            manipulator_cmd.data = tau_result(7);
            pub_manipulator_dof_5.publish(manipulator_cmd);

            manipulator_cmd.data = tau_result(8);
            pub_manipulator_dof_6.publish(manipulator_cmd);
            
        }

        /******************************/
        // Set time and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::requestShutdown();
}

////////////////////////////////////////////////////////////////////////////////
// Function to load the robot in DART
void OscHybridController::loadDARTModel(){

    dart::utils::DartLoader loader;

    // Add packages
    loader.addPackageDirectory("mobile_robot_unal_description", "/home/josefajardo/tesis_2020/mobile_robot_unal_description/");
    loader.addPackageDirectory("manipulator_unal_description", "/home/josefajardo/tesis_2020/manipulator_unal_description/");
    loader.addPackageDirectory("mobile_manipulator_unal_description", "/home/josefajardo/tesis_2020/mobile_manipulator_unal_description/");
    

    std::string filename = "/home/josefajardo/tesis_2020/mobile_manipulator_unal_description/urdf/mobile_manipulator.urdf";

    // Load the robot
    dart_robotSkeleton = loader.parseSkeleton(filename);

    /******************************/
    // Change Wheel0 joint to WeldJoint
    dart::dynamics::Joint::Properties auxJoint0 = dart_robotSkeleton->getJoint("wheel0_joint")->getJointProperties();

    dart::dynamics::WeldJoint::Properties fixedJoint0;
    fixedJoint0.mName = auxJoint0.mName;
    fixedJoint0.mT_ParentBodyToJoint = auxJoint0.mT_ParentBodyToJoint;
    fixedJoint0.mT_ChildBodyToJoint = auxJoint0.mT_ChildBodyToJoint;

    dart_robotSkeleton->getBodyNode("mobile_manipulator/wheel0_link")->changeParentJointType<WeldJoint>(fixedJoint0);

    /******************************/
    // Change Wheel1 joint to WeldJoint
    dart::dynamics::Joint::Properties auxJoint1 = dart_robotSkeleton->getJoint("wheel1_joint")->getJointProperties();

    dart::dynamics::WeldJoint::Properties fixedJoint1;
    fixedJoint1.mName = auxJoint1.mName;
    fixedJoint1.mT_ParentBodyToJoint = auxJoint1.mT_ParentBodyToJoint;
    fixedJoint1.mT_ChildBodyToJoint = auxJoint1.mT_ChildBodyToJoint;

    dart_robotSkeleton->getBodyNode("mobile_manipulator/wheel1_link")->changeParentJointType<WeldJoint>(fixedJoint1);

    /******************************/
    // Change Wheel2 joint to WeldJoint
    dart::dynamics::Joint::Properties auxJoint2 = dart_robotSkeleton->getJoint("wheel2_joint")->getJointProperties();

    dart::dynamics::WeldJoint::Properties fixedJoint2;
    fixedJoint2.mName = auxJoint2.mName;
    fixedJoint2.mT_ParentBodyToJoint = auxJoint2.mT_ParentBodyToJoint;
    fixedJoint2.mT_ChildBodyToJoint = auxJoint2.mT_ChildBodyToJoint;

    dart_robotSkeleton->getBodyNode("mobile_manipulator/wheel2_link")->changeParentJointType<WeldJoint>(fixedJoint2);

    /******************************/
    // Change root_joint to planar joint
    dart::dynamics::Joint::Properties auxBaseJoint = dart_robotSkeleton->getRootJoint()->getJointProperties();

    dart::dynamics::PlanarJoint::Properties planarJoint;
    std::string jointName = "base_planar_joint";
    planarJoint.mName = jointName;

    planarJoint.setXYPlane();

    std::cout << "Planar 1 T: \n" << planarJoint.mTransAxis1 << std::endl;
    std::cout << "Planar 2 T: \n" << planarJoint.mTransAxis2 << std::endl;
    std::cout << "Planar 1 R: \n" << planarJoint.mRotAxis << std::endl;

    auto dofNames = planarJoint.mDofNames;
    dofNames.at(0) = "test_1";
    dofNames.at(1) = "test_2";
    dofNames.at(2) = "test_3";

    std::cout << "Name Planar 1 T:" << dofNames.at(0) << std::endl;

    planarJoint.mT_ParentBodyToJoint = auxBaseJoint.mT_ParentBodyToJoint;
    planarJoint.mT_ChildBodyToJoint = auxBaseJoint.mT_ChildBodyToJoint;

    dart_robotSkeleton->getRootBodyNode()->changeParentJointType<PlanarJoint>(planarJoint);

    Joint* base_planarJoint = dart_robotSkeleton->getJoint("base_planar_joint");
    base_planarJoint->setActuatorType(Joint::FORCE);

    std::cout << "Type of Joint: " << dart_robotSkeleton->getJoint("base_planar_joint")->getType() << std::endl;
    std::cout << "Actuator Type : " << dart_robotSkeleton->getJoint("base_planar_joint")->getActuatorType() << std::endl;

    dart_robotSkeleton->getDof(0)->setName("mobile_base_trans_x");
    dart_robotSkeleton->getDof(1)->setName("mobile_base_trans_y");
    dart_robotSkeleton->getDof(2)->setName("mobile_base_rot_z");

    /******************************/
    // Change Left finger joint to WeldJoint
    dart::dynamics::Joint::Properties auxJoint3 = dart_robotSkeleton->getJoint("left_finger_joint")->getJointProperties();

    dart::dynamics::WeldJoint::Properties fixedJoint3;
    fixedJoint3.mName = auxJoint3.mName;
    fixedJoint3.mT_ParentBodyToJoint = auxJoint3.mT_ParentBodyToJoint;
    fixedJoint3.mT_ChildBodyToJoint = auxJoint3.mT_ChildBodyToJoint;

    dart_robotSkeleton->getBodyNode("mobile_manipulator/left_finger_link")->changeParentJointType<WeldJoint>(fixedJoint3);

    /******************************/
    // Change Right finger joint to WeldJoint
    dart::dynamics::Joint::Properties auxJoint4 = dart_robotSkeleton->getJoint("right_finger_joint")->getJointProperties();

    dart::dynamics::WeldJoint::Properties fixedJoint4;
    fixedJoint4.mName = auxJoint4.mName;
    fixedJoint4.mT_ParentBodyToJoint = auxJoint4.mT_ParentBodyToJoint;
    fixedJoint4.mT_ChildBodyToJoint = auxJoint4.mT_ChildBodyToJoint;

    dart_robotSkeleton->getBodyNode("mobile_manipulator/right_finger_link")->changeParentJointType<WeldJoint>(fixedJoint4);

    /******************************/
    // Change Right finger joint to WeldJoint
    mEndEffector_ = dart_robotSkeleton->getBodyNode("mobile_manipulator/tcp_gripper_link");

    /******************************/
    // Add properties to the DART Model
    dart_robotSkeleton->setMobile(false);
    //dart_robotSkeleton->enableSelfCollisionCheck();

    /******************************/
    // INIT q_dot and tau

    dart_robotSkeleton->setMobile(false);
    dart_robotSkeleton->setPositions(q_k);
    dart_robotSkeleton->setVelocities(q_dot_k);

    M = dart_robotSkeleton->getMassMatrix();  // Mass Matrix
    //std::cout << "Mass Matrix: \n" << M << std::endl;

    C_k = dart_robotSkeleton->getCoriolisForces();  // Coriolis vector forces
    g_k = dart_robotSkeleton->getGravityForces();   // Gravity vector forces

    // Joint Conf Desired
    Eigen::VectorXd q_desired = Eigen::VectorXd::Zero(9);
    q_desired(0) = q_k(0);
    q_desired(1) = q_k(1);
    q_desired(2) = q_k(2);

    //effortSolver_.AchieveJointConf(&tau_zero, &tau_result, q_desired, M, C_k, g_k, dart_robotSkeleton, mEndEffector_ );
    //effortSolver_.AvoidJointLimits(&tau_zero, &tau_result, M, C_k, g_k, dart_robotSkeleton, mEndEffector_ );
    //state = 0;

}

} /* namespace */