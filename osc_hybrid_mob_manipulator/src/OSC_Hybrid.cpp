#include <osc_hybrid_mob_manipulator/OSC_Hybrid.hpp>

namespace osc_hybrid_controller {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

using namespace effort_tasks;

////////////////////////////////////////////////////////////////////////////////
// Callback function of the joint states
void OscHybridController::jointState_CB(const sensor_msgs::JointState msg){

    for (int ii = 0; ii < msg.position.size(); ii++){
        
        if(msg.name.at(ii)==manipulator_dofs.at(0)){
            q_k(3)        = msg.position.at(ii);
            q_dot_k(3)    = msg.velocity.at(ii);
            tau_joints(3) = msg.effort.at(ii);
        }
        if(msg.name.at(ii)==manipulator_dofs.at(1)){
            q_k(4)        = msg.position.at(ii);
            q_dot_k(4)    = msg.velocity.at(ii);
            tau_joints(4) = msg.effort.at(ii);
        }
        if(msg.name.at(ii)==manipulator_dofs.at(2)){
            q_k(5)        = msg.position.at(ii);
            q_dot_k(5)    = msg.velocity.at(ii);
            tau_joints(5) = msg.effort.at(ii);
        }
        if(msg.name.at(ii)==manipulator_dofs.at(3)){
            q_k(6)        = msg.position.at(ii);
            q_dot_k(6)    = msg.velocity.at(ii);
            tau_joints(6) = msg.effort.at(ii);
        }
        if(msg.name.at(ii)==manipulator_dofs.at(4)){
            q_k(7)        = msg.position.at(ii);
            q_dot_k(7)    = msg.velocity.at(ii);
            tau_joints(7) = msg.effort.at(ii);
        }
        if(msg.name.at(ii)==manipulator_dofs.at(5)){
            q_k(8)        = msg.position.at(ii);
            q_dot_k(8)    = msg.velocity.at(ii);
            tau_joints(8) = msg.effort.at(ii);
        }

    }    

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

    /******************************/
    //std::cout << "Joint Pos: \n" << q_k << std::endl;
    //std::cout << "Joint Vel: \n" << q_dot_k << std::endl;

    return;

}

////////////////////////////////////////////////////////////////////////////////
// Callback function to change the desired pose
void OscHybridController::change_DesPose_CB(const mobile_manipulator_msgs::Trajectory msg){

    transformDesiredPos.transform.translation.x = msg.pose.translation.x;
    transformDesiredPos.transform.translation.y = msg.pose.translation.y;
    transformDesiredPos.transform.translation.z = msg.pose.translation.z;
    transformDesiredPos.transform.rotation.x    = msg.pose.rotation.x;
    transformDesiredPos.transform.rotation.y    = msg.pose.rotation.y;
    transformDesiredPos.transform.rotation.z    = msg.pose.rotation.z;
    transformDesiredPos.transform.rotation.w    = msg.pose.rotation.w;

    mob_man_traj.pose.translation.x = msg.pose.translation.x;
    mob_man_traj.pose.translation.y = msg.pose.translation.y;
    mob_man_traj.pose.translation.z = msg.pose.translation.z;
    mob_man_traj.vel.linear.x       = msg.vel.linear.x;
    mob_man_traj.vel.linear.y       = msg.vel.linear.y;
    mob_man_traj.vel.linear.z       = msg.vel.linear.z;
    mob_man_traj.accel.linear.x     = msg.accel.linear.x;
    mob_man_traj.accel.linear.y     = msg.accel.linear.y;
    mob_man_traj.accel.linear.z     = msg.accel.linear.z;

    mob_man_traj.pose.rotation.x = msg.pose.rotation.x;
    mob_man_traj.pose.rotation.y = msg.pose.rotation.y;
    mob_man_traj.pose.rotation.z = msg.pose.rotation.z;
    mob_man_traj.pose.rotation.w = msg.pose.rotation.w;
    mob_man_traj.vel.angular.x   = msg.vel.angular.x;
    mob_man_traj.vel.angular.y   = msg.vel.angular.y;
    mob_man_traj.vel.angular.z   = msg.vel.angular.z;
    mob_man_traj.accel.angular.x = msg.accel.angular.x;
    mob_man_traj.accel.angular.y = msg.accel.angular.y;
    mob_man_traj.accel.angular.z = msg.accel.angular.z;

    mob_man_traj.joints.mobjoint1 = msg.joints.mobjoint1;
    mob_man_traj.joints.mobjoint2 = msg.joints.mobjoint2;
    mob_man_traj.joints.mobjoint3 = msg.joints.mobjoint3;
    mob_man_traj.joints.joint1    = msg.joints.joint1;
    mob_man_traj.joints.joint2    = msg.joints.joint2;
    mob_man_traj.joints.joint3    = msg.joints.joint3;
    mob_man_traj.joints.joint4    = msg.joints.joint4;
    mob_man_traj.joints.joint5    = msg.joints.joint5;
    mob_man_traj.joints.joint6    = msg.joints.joint6;

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
    tau_joints(Eigen::VectorXd::Zero(9)),
    q_dot_result(Eigen::VectorXd::Zero(9)),
    q_dot_zero(Eigen::VectorXd::Zero(9))
{   
    /******************************/
    // Load parameters
    if (!readParameters()) {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }
    
    /******************************/
    // Init messages

    mobile_pltfrm_cmd.linear.x  = 0.0;
    mobile_pltfrm_cmd.linear.y  = 0.0;
    mobile_pltfrm_cmd.linear.z  = 0.0;
    mobile_pltfrm_cmd.angular.x = 0.0;
    mobile_pltfrm_cmd.angular.y = 0.0;
    mobile_pltfrm_cmd.angular.z = 0.0;

    mobile_manipulator_data.header.frame_id = "odom";
    mobile_manipulator_data.header.stamp = ros::Time::now();

    /******************************/
    // Create subscribers
    subs_joint_state_= nodeHandle_.subscribe("/"+ robot_name +"/joint_states" , 10, 
                                        &OscHybridController::jointState_CB, this);
    subs_odometry_   = nodeHandle_.subscribe("/"+ robot_name +"/" + odometry_topic , 10, 
                                        &OscHybridController::odometry_CB, this);
    
    subs_desired_traj_ = nodeHandle_.subscribe("/"+ robot_name +"/" + "desired_traj" , 10, 
                                        &OscHybridController::change_DesPose_CB, this);
    
    /******************************/
    // Create publishers
    pub_mobile_platfrm  = nodeHandle_.advertise<geometry_msgs::Twist>("/"+ robot_name +"/commands/velocity" , 10);

    pub_manipulator_dof_1 = nodeHandle_.advertise<std_msgs::Float64>("/"+ robot_name +"/"+ manipulator_controllers.at(0) +"/command" , 10);
    pub_manipulator_dof_2 = nodeHandle_.advertise<std_msgs::Float64>("/"+ robot_name +"/"+ manipulator_controllers.at(1) +"/command" , 10);
    pub_manipulator_dof_3 = nodeHandle_.advertise<std_msgs::Float64>("/"+ robot_name +"/"+ manipulator_controllers.at(2) +"/command" , 10);
    pub_manipulator_dof_4 = nodeHandle_.advertise<std_msgs::Float64>("/"+ robot_name +"/"+ manipulator_controllers.at(3) +"/command" , 10);
    pub_manipulator_dof_5 = nodeHandle_.advertise<std_msgs::Float64>("/"+ robot_name +"/"+ manipulator_controllers.at(4) +"/command" , 10);
    pub_manipulator_dof_6 = nodeHandle_.advertise<std_msgs::Float64>("/"+ robot_name +"/"+ manipulator_controllers.at(5) +"/command" , 10);

    pub_mob_manipulator_data = nodeHandle_.advertise<mobile_manipulator_msgs::MobileManipulator>("/"+ robot_name +"/data" , 10);
    
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
    if (!nodeHandle_.getParam("compensate_topdownEffects", topdown_))        return false;
    if (!nodeHandle_.getParam("compensate_nonlinearInJointSpace", jtspace_))        return false;
    if (!nodeHandle_.getParam("use_augmented_projections", augmented_))        return false;

    robot_frame = robot_name + "/mobile_base_link";

    ROS_INFO("Robot Frame: %s", robot_frame.c_str());

    ROS_INFO("Robot dof1: %s", manipulator_dofs.at(0).c_str());

    //--- Top-down effects parameter
    if(topdown_){
        ROS_INFO("Compensating Top down effects");
        effortSolver_.compensate_topdown = true;
    }
    else{
        ROS_INFO("NOT Compensating Top down effects");
        effortSolver_.compensate_topdown = false;
    }

    //--- Compensate non-linear effects in joint space parameter
    if(jtspace_){
        ROS_INFO("Compensating Nonlinear effects in joint space");
        effortSolver_.compensate_jtspace = true;
    }
    else{
        ROS_INFO("NOT Compensating Nonlinear effects in joint space");
        effortSolver_.compensate_jtspace = false;
    }

    //--- Use succesive or augmented projections
    if(augmented_){
        ROS_INFO("Using augmented projections");
        effortSolver_.augmented_projections = true;
    }
    else{
        ROS_INFO("Using succesive projections");
        effortSolver_.augmented_projections = false;
    }
    
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Spin function
void OscHybridController::spin(){

    // Create loop rate object
    ros::Rate loop_rate(frecuency_rate);

    while (ros::ok()){  

        //std::cout << "Joint Pos: \n" << q_k << std::endl;
        //std::cout << "Joint Vel: \n" << q_dot_k << std::endl;  

        Eigen::VectorXd current_pos =  q_k;
        Eigen::VectorXd current_vel =  q_dot_k;

        /******************************/
        // Update state of the DART Model

        dart_robotSkeleton->setPositions(current_pos);
        dart_robotSkeleton->setVelocities(current_vel);

        M = dart_robotSkeleton->getMassMatrix();  // Mass Matrix
        //std::cout << "Mass Matrix: \n" << M << std::endl;

        C_k = dart_robotSkeleton->getCoriolisForces();  // Coriolis vector forces
        g_k = dart_robotSkeleton->getGravityForces();   // Gravity vector forces
        //std::cout << "Coriolis vector forces: \n" << C_k << std::endl;
        //std::cout << "Gravity vector forces: \n" << g_k << std::endl;

        
        /******************************/
        // Target definition

        //--- Publish tranform to desired pose frame
        transformDesiredPos.header.stamp = ros::Time::now();
        broadcaster_.sendTransform(transformDesiredPos);

        //--- Cartesian Target
        Eigen::Vector3d targetCartPos = Eigen::Vector3d::Zero(); 
        Eigen::Vector3d targetCartVel = Eigen::Vector3d::Zero(); 
        Eigen::Vector3d targetCartAccel = Eigen::Vector3d::Zero(); 

        targetCartPos(0)   = mob_man_traj.pose.translation.x;
        targetCartPos(1)   = mob_man_traj.pose.translation.y;
        targetCartPos(2)   = mob_man_traj.pose.translation.z;
        targetCartVel(0)   = mob_man_traj.vel.linear.x;
        targetCartVel(1)   = mob_man_traj.vel.linear.y;
        targetCartVel(2)   = mob_man_traj.vel.linear.z;
        targetCartAccel(0) = mob_man_traj.accel.linear.x;
        targetCartAccel(1) = mob_man_traj.accel.linear.y;
        targetCartAccel(2) = mob_man_traj.accel.linear.z;

        //std::cout << "Cart Desired: \n" << targetCartPos << std::endl;

        //--- Orientation target
        Eigen::Quaterniond quat_des(mob_man_traj.pose.rotation.w, mob_man_traj.pose.rotation.x, mob_man_traj.pose.rotation.y, mob_man_traj.pose.rotation.z);
        Eigen::Matrix3d targetOrientPos = quat_des.normalized().toRotationMatrix();
        //std::cout << "Rotation matrix: \n" << targetOrientPos << std::endl; // Print rot matrix

        Eigen::Vector3d targetOrientVel = Eigen::Vector3d::Zero(); 
        Eigen::Vector3d targetOrientAccel = Eigen::Vector3d::Zero(); 

        targetOrientVel(0)   = mob_man_traj.vel.angular.x;
        targetOrientVel(1)   = mob_man_traj.vel.angular.y;
        targetOrientVel(2)   = mob_man_traj.vel.angular.z;
        targetOrientAccel(0) = mob_man_traj.accel.angular.x;
        targetOrientAccel(1) = mob_man_traj.accel.angular.y;
        targetOrientAccel(2) = mob_man_traj.accel.angular.z;

        //--- Joint Conf Desired
        Eigen::VectorXd q_desired = Eigen::VectorXd::Zero(9);
        q_desired(0) = current_pos(0);
        q_desired(1) = current_pos(1);
        q_desired(2) = mob_man_traj.joints.mobjoint3;
        q_desired(3) = mob_man_traj.joints.joint1;
        q_desired(4) = mob_man_traj.joints.joint2;
        q_desired(5) = mob_man_traj.joints.joint3;
        q_desired(6) = mob_man_traj.joints.joint4;
        q_desired(7) = mob_man_traj.joints.joint5;
        q_desired(8) = mob_man_traj.joints.joint6;
        
        for(size_t jj = 2; jj< 9; jj++){
            if( q_desired(jj) == -10.0 ){
                q_desired(jj) = current_pos(jj);
            }
        }

        //--- External torques
        Eigen::VectorXd tau_ext = tau_result - tau_joints;
        tau_ext(0) = 0.0;
        tau_ext(1) = 0.0;
        tau_ext(2) = 0.0;
        tau_ext = Eigen::VectorXd::Zero(q_desired.size());
        //std::cout << "Ext Torques: \n" << tau_ext << std::endl;

        //--- Variables to save minimum singular value
        double min_sv_pos = 10.0e3, min_sv_ori = 10.0e3;

        /*********************************************************************************/
        // Task Definition for effort

        tau_result = Eigen::VectorXd::Zero(9);
        Eigen::MatrixXd Null_space = Eigen::MatrixXd::Identity(9,9);

        //std::cout << "Initial tau: \n" << tau_result << std::endl;
        //std::cout << "Initial Null space: \n" << Null_space << std::endl;

        Eigen::Vector3d x_error =  targetCartPos - mEndEffector_->getWorldTransform().translation();

        /*****************************************************/
        // Using different controllers based on position error

        /*
        if(x_error.norm()>0.2){
            
            if(x_error.norm()<1.0){
                q_desired = current_pos;
            }
            effortSolver_.AchieveCartesianMobilRob(targetCartPos, targetCartVel, targetCartAccel, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
            effortSolver_.AchievePosZConstVel(targetCartPos, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        }
        else{
            effortSolver_.AchieveCartesian(targetCartPos, targetCartVel, targetCartAccel, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);effortSolver_.AchieveCartesianConstVel(targetCartPos, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
            //effortSolver_.AchieveCartesianConstVel(targetCartPos, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
            //effortSolver_.CartesianImpedance(targetCartPos, targetCartVel, targetCartAccel, &min_sv_pos, tau_ext, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
            
            //std::cout << "Tau result after straight line: \n" << tau_result << std::endl;
            //std::cout << "Null space after straight line: \n" << Null_space << std::endl;
        }*/

        /*****************************************************/
        // Avoid Joint Limits task

        //effortSolver_.AvoidJointLimits(M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        //std::cout << "Tau result after avoid joint limits: \n" << tau_result << std::endl;
        //std::cout << "Null space after avoid joint limits: \n" << Null_space << std::endl;

        /*****************************************************/
        // Controller using pos XY with mobile robot and Z with mobile manipulator

        //---effortSolver_.AchieveCartesianMobilRob(targetCartPos, targetCartVel, targetCartAccel, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        //effortSolver_.AchieveCartesianMobilRobConstVel(targetCartPos, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        //std::cout << "Tau result after XY Cart: \n" << tau_result << std::endl;
        //std::cout << "Null space after straight line: \n" << Null_space << std::endl;

        //---effortSolver_.AchievePosZ(targetCartPos, targetCartVel, targetCartAccel, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        //effortSolver_.AchievePosZConstVel(targetCartPos, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        //std::cout << "Tau result after Z Cart: \n" << tau_result << std::endl;
        //std::cout << "Null space after Z Cart: \n" << Null_space << std::endl;

        /*****************************************************/
        // Controller using pos XYZ with manipulator

        //effortSolver_.AchieveCartManipulatorConstVel(targetCartPos, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        //std::cout << "Tau result after Manipulator Cart: \n" << tau_result << std::endl;
        //std::cout << "Null space after Manipulator Cart: \n" << Null_space << std::endl;

        /*****************************************************/
        // Controller using pos XYZ with mobile manipulator

        //---effortSolver_.AchieveCartesianManipulator(targetCartPos, targetCartVel, targetCartAccel, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        //effortSolver_.AchieveCartesianConstVel(targetCartPos, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        //std::cout << "Tau result after Mob_mani Cart: \n" << tau_result << std::endl;
        //std::cout << "Null space after Mob_mani Cart: \n" << Null_space << std::endl;

        /*****************************************************/
        // Orientation tasks with mobile manipulator

        //---effortSolver_.AchieveOrientation(targetOrientPos, targetOrientVel, targetOrientAccel, &min_sv_ori, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        //effortSolver_.AchieveOrientationConstVel(targetOrientPos, &min_sv_ori, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space); 
        //---effortSolver_.OrientationImpedance(targetOrientPos, targetOrientVel, targetOrientAccel, &min_sv_ori, tau_ext, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space); 
        //std::cout << "Tau result after achieve orient: \n" << tau_result << std::endl;
        //std::cout << "Null space after achieve orient: \n" << Null_space << std::endl;

        /*****************************************************/
        // Orientation tasks with manipulator

        //---effortSolver_.AchieveOriManipulator(targetOrientPos, targetOrientVel, targetOrientAccel, &min_sv_ori, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        effortSolver_.AchieveOriManipulatorConstVel(targetOrientPos, &min_sv_ori, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space); 
        //std::cout << "Tau result after achieve orient: \n" << tau_result << std::endl;
        //std::cout << "Null space after achieve orient: \n" << Null_space << std::endl;

        /*****************************************************/
        // Controller using pos XYZ with manipulator - To test singularities

        //---effortSolver_.AchieveCartesianManipulator(targetCartPos, targetCartVel, targetCartAccel, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        effortSolver_.AchieveCartManipulatorConstVel(targetCartPos, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        //std::cout << "Tau result after Manipulator Cart: \n" << tau_result << std::endl;
        //std::cout << "Null space after Manipulator Cart: \n" << Null_space << std::endl;
    
        /*****************************************************/
        // Joint tasks

        effortSolver_.AchieveJointConf(q_desired, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
        //std::cout << "Tau result after achieve joint: \n" << tau_result.transpose() << "\n" << std::endl;

        /*****************************************************/
        // Compensation of non-linear effects in joint space

        if(effortSolver_.compensate_jtspace){
            tau_result =  tau_result + C_k + g_k;
        }

        ///std::cout << "Tau Final: \n" << tau_result << "\n" << std::endl;
        
        /******************************/
        // Admittance controller
        // send_vel -> q_dot_result
        
        Eigen::MatrixXd damp_des_     = Eigen::MatrixXd::Identity(3, 3) ;
        damp_des_.topLeftCorner(2, 2) = 800.0*Eigen::MatrixXd::Identity(2, 2); // Liner vel damping (60)
        damp_des_(2,2) = 800.0 ; // Angular vel damping (20.0)

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

        
        /*for (size_t jj = 0; jj < 3; jj++){
            if(abs(q_dot_result(jj))<0.1){
                q_dot_result(jj) = 0.0;
            }
        }*/

        //std::cout << "Vel Command: \n" << q_dot_result << std::endl;

        /******************************/
        // Limit efforts

        for (size_t ii = 0; ii < 9; ii++){
            
            if(isnan(tau_result(ii))) {
                tau_result(ii) = 0.0;
                std::cout << "Nan in efforts." << std::endl;
            }

            if(ii==3 || ii ==6 || ii ==7){ 
                if( abs(tau_result(ii)) > 10.6){
                    //std::cout << "Limited effort of joint: "<< ii << std::endl;  
                    tau_result(ii) = tau_result(ii) * (10.6/abs(tau_result(ii)));
                }
            }
            else if(ii==4 || ii==5 ){    
                if( abs(tau_result(ii)) > 21.2){
                    //std::cout << "Limited effort of joint: "<< ii << std::endl; 
                    tau_result(ii) = tau_result(ii) * (21.2/abs(tau_result(ii)));
                }
            }
            else if(ii==8){    
                if( abs(tau_result(ii)) > 4.1){
                    //std::cout << "Limited effort of joint: "<< ii << std::endl; 
                    tau_result(ii) = tau_result(ii) * (4.1/abs(tau_result(ii)));
                }
            }
        }

        //std::cout << "Limited Efforts: \n" << tau_result << std::endl;

        /******************************/
        // Limit Velocities

        for (size_t ii = 0; ii < 9; ii++){

            if(isnan(q_dot_result(ii))) {
                q_dot_result(ii) = 0.0;
                std::cout << "Nan in Velocities." << std::endl;
            }

            if(ii<2){ // Mobile platform efforts linear vel
                if( abs(q_dot_result(ii)) > 0.5 )  q_dot_result(ii) = q_dot_result(ii) * (0.5/abs(q_dot_result(ii)));
            }
            else if(ii==3){ // Mobile platform angular vel
                if( abs(q_dot_result(ii)) > M_PI_2 )  q_dot_result(ii) = q_dot_result(ii) * (1.75/abs(q_dot_result(ii)));
            }
        }

        //std::cout << "Limited Vel: \n" << q_dot_result << std::endl;

        /******************************/
        // Message data update

        Eigen::Vector3d current_x =  mEndEffector_->getWorldTransform().translation();

        mobile_manipulator_data.position.current_position.x = current_x(0);
        mobile_manipulator_data.position.current_position.y = current_x(1);
        mobile_manipulator_data.position.current_position.z = current_x(2);
        mobile_manipulator_data.position.desired_position.x = targetCartPos(0);
        mobile_manipulator_data.position.desired_position.y = targetCartPos(1);
        mobile_manipulator_data.position.desired_position.z = targetCartPos(2);

        Eigen::Matrix3d eigen_rot_mat = mEndEffector_->getWorldTransform().rotation(); 
        Eigen::Quaterniond eigen_quat(eigen_rot_mat);

        tf2::Quaternion tf2_quat;
        tf2_quat.setX(eigen_quat.x());
        tf2_quat.setY(eigen_quat.y());
        tf2_quat.setZ(eigen_quat.z());
        tf2_quat.setW(eigen_quat.w());

        tf2::Matrix3x3 rot_mat;
        rot_mat.setRotation(tf2_quat);

        double roll, pitch, yaw;
        rot_mat.getEulerZYX(yaw, pitch, roll);

        mobile_manipulator_data.orientation.current_orient.roll = angles::to_degrees(roll) ;
        mobile_manipulator_data.orientation.current_orient.pitch = angles::to_degrees(pitch);
        mobile_manipulator_data.orientation.current_orient.yaw = angles::to_degrees(yaw);

        tf2_quat.setX(mob_man_traj.pose.rotation.x);
        tf2_quat.setY(mob_man_traj.pose.rotation.y);
        tf2_quat.setZ(mob_man_traj.pose.rotation.z);
        tf2_quat.setW(mob_man_traj.pose.rotation.w);
        rot_mat.setRotation(tf2_quat);
        rot_mat.getEulerZYX(yaw, pitch, roll);
        

        mobile_manipulator_data.orientation.desired_orient.roll = angles::to_degrees(roll);
        mobile_manipulator_data.orientation.desired_orient.pitch = angles::to_degrees(pitch);
        mobile_manipulator_data.orientation.desired_orient.yaw = angles::to_degrees(yaw);

        mobile_manipulator_data.singular.min_pos_jacob = min_sv_pos;
        mobile_manipulator_data.singular.min_ori_jacob = min_sv_ori;

        mobile_manipulator_data.torques.torque1 = tau_result(0);
        mobile_manipulator_data.torques.torque2 = tau_result(1);
        mobile_manipulator_data.torques.torque3 = tau_result(2);
        mobile_manipulator_data.torques.torque4 = tau_result(3);
        mobile_manipulator_data.torques.torque5 = tau_result(4);
        mobile_manipulator_data.torques.torque6 = tau_result(5);
        mobile_manipulator_data.torques.torque7 = tau_result(6);
        mobile_manipulator_data.torques.torque8 = tau_result(7);
        mobile_manipulator_data.torques.torque9 = tau_result(8);

        mobile_manipulator_data.joints.actual.mobjoint1  = current_pos(0);
        mobile_manipulator_data.joints.actual.mobjoint2  = current_pos(1);
        mobile_manipulator_data.joints.actual.mobjoint3  = current_pos(2);
        mobile_manipulator_data.joints.actual.joint1     = current_pos(3);
        mobile_manipulator_data.joints.actual.joint2     = current_pos(4);
        mobile_manipulator_data.joints.actual.joint3     = current_pos(5);
        mobile_manipulator_data.joints.actual.joint4     = current_pos(6);
        mobile_manipulator_data.joints.actual.joint5     = current_pos(7);
        mobile_manipulator_data.joints.actual.joint6     = current_pos(8);
        mobile_manipulator_data.joints.desired.mobjoint1 = q_desired(0);
        mobile_manipulator_data.joints.desired.mobjoint2 = q_desired(1);
        mobile_manipulator_data.joints.desired.mobjoint3 = q_desired(2);
        mobile_manipulator_data.joints.desired.joint1    = q_desired(3);
        mobile_manipulator_data.joints.desired.joint2    = q_desired(4);
        mobile_manipulator_data.joints.desired.joint3    = q_desired(5);
        mobile_manipulator_data.joints.desired.joint4    = q_desired(6);
        mobile_manipulator_data.joints.desired.joint5    = q_desired(7);
        mobile_manipulator_data.joints.desired.joint6    = q_desired(8);


        pub_mob_manipulator_data.publish(mobile_manipulator_data);

        /******************************/
        // Publish commands to mobile platform

        mobile_pltfrm_cmd.linear.x  = 0.0;//q_dot_result(0); // 0.0
        mobile_pltfrm_cmd.linear.y  = 0.0;//q_dot_result(1); // 0.0
        mobile_pltfrm_cmd.angular.z = 0.0;//q_dot_result(2); // 0.0

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
            
        //}

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

    /*******************************/
    // Broadcast initial desired pose as original pose

    transformDesiredPos.header.stamp = ros::Time::now();
    transformDesiredPos.header.frame_id = "odom";
    transformDesiredPos.child_frame_id = robot_name + "/" + "desired_pose";

    Eigen::Vector3d orig_pos =  mEndEffector_->getWorldTransform().translation() ;
    
    transformDesiredPos.transform.translation.x = orig_pos(0);
    transformDesiredPos.transform.translation.y = orig_pos(1);
    transformDesiredPos.transform.translation.z = orig_pos(2);

    Eigen::Matrix3d R_orig = mEndEffector_->getWorldTransform().rotation(); 
    Eigen::Quaterniond quat_orig(R_orig);

    transformDesiredPos.transform.rotation.x = quat_orig.x();
    transformDesiredPos.transform.rotation.y = quat_orig.y();
    transformDesiredPos.transform.rotation.z = quat_orig.z();
    transformDesiredPos.transform.rotation.w = quat_orig.w();

    mob_man_traj.pose.translation.x = orig_pos(0);
    mob_man_traj.pose.translation.y = orig_pos(1);
    mob_man_traj.pose.translation.z = orig_pos(2);
    mob_man_traj.vel.linear.x       = 0.0;
    mob_man_traj.vel.linear.y       = 0.0;
    mob_man_traj.vel.linear.z       = 0.0;
    mob_man_traj.accel.linear.x     = 0.0;
    mob_man_traj.accel.linear.y     = 0.0;
    mob_man_traj.accel.linear.z     = 0.0;

    mob_man_traj.pose.rotation.x = quat_orig.x();
    mob_man_traj.pose.rotation.y = quat_orig.y();
    mob_man_traj.pose.rotation.z = quat_orig.z();
    mob_man_traj.pose.rotation.w = quat_orig.w();
    mob_man_traj.vel.angular.x   = 0.0;
    mob_man_traj.vel.angular.y   = 0.0;
    mob_man_traj.vel.angular.z   = 0.0;
    mob_man_traj.accel.angular.x = 0.0;
    mob_man_traj.accel.angular.y = 0.0;
    mob_man_traj.accel.angular.z = 0.0;

    mob_man_traj.joints.mobjoint3 = -10.0;
    mob_man_traj.joints.joint1 = 0.0;
    mob_man_traj.joints.joint2 = 0.0;
    mob_man_traj.joints.joint3 = 0.0;
    mob_man_traj.joints.joint4 = 0.0;
    mob_man_traj.joints.joint5 = 0.0;
    mob_man_traj.joints.joint6 = 0.0;

    broadcaster_.sendTransform(transformDesiredPos);

}

} /* namespace */