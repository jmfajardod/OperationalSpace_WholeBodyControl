#include <osc_hybrid_mob_manipulator/Effort_Tasks.hpp>

namespace effort_tasks {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian orientation 

void EffortTask::AchieveOrientation(Eigen::Matrix3d rot_mat_desired, 
                                    Eigen::Vector3d mTargetVel,
                                    Eigen::Vector3d mTargetAccel,
                                    double *svd_orientation,
                                    Eigen::MatrixXd M,
                                    Eigen::VectorXd C_t,
                                    Eigen::VectorXd g_t,
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector,
                                    Eigen::VectorXd *tau_total,
                                    Eigen::MatrixXd *Null_space_iter){

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate operational space matrices

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    AngularJacobian Normal_Jacob_t = mEndEffector->getAngularJacobian(); // Angular Jacobian
    Eigen::MatrixXd Jacob_t = Normal_Jacob_t * (*Null_space_iter).transpose();
    //std::cout << "Angular Jacob: \n" << Jacob_t << std::endl;

    AngularJacobian Normal_Jacob_dot = mEndEffector->getAngularJacobianDeriv(); // Derivative of jacobian
    Eigen::MatrixXd Jacob_dot = Normal_Jacob_dot * (*Null_space_iter).transpose();

    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Angular Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate SVD for alpha

    Eigen::MatrixXd Alpha_t_inv = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix
    
    Eigen::MatrixXd Alpha_t = calcInertiaMatrix(Alpha_t_inv, svd_orientation);

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Angular Jacobian: \n" << Jacob_dash_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    Eigen::Vector3d error_ori = Eigen::Vector3d::Zero(3);

    switch(ori_error_mode){
        case 1:
            error_ori = ErrorAngleAxis1(rot_mat_desired, mRobot, mEndEffector);
            break;
        case 2:
            error_ori = ErrorAngleAxis2(rot_mat_desired, mRobot, mEndEffector);
            break;
        case 3:
            error_ori = ErrorQuaternion1(rot_mat_desired, mRobot, mEndEffector);
            break;
        case 4:
            error_ori = ErrorQuaternion2(rot_mat_desired, mRobot, mEndEffector);
            break;
        case 5:
            error_ori = ErrorQuaternion3(rot_mat_desired, mRobot, mEndEffector);
            break;
        default:
            error_ori = ErrorQuaternion3(rot_mat_desired, mRobot, mEndEffector);
    }

    Eigen::Vector3d angular_vel =  mEndEffector->getAngularJacobian() * mRobot->getVelocities();
    //std::cout << "Angular velocity vector: \n"  << angular_vel << std::endl;

    Eigen::Matrix3d kp = kp_cartesian_.bottomRightCorner(3, 3);
    Eigen::Matrix3d kd = kd_cartesian_.bottomRightCorner(3, 3);

    Eigen::Vector3d x_star = mTargetAccel + kd *(mTargetVel-angular_vel) + kp * error_ori ; 
    if(compensate_topdown){
        x_star = x_star - Alpha_t_inv * Jacob_dash_t.transpose() * *tau_total;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational force due to task

    Eigen::VectorXd f_t_star = Eigen::VectorXd::Zero(3);
    if(compensate_jtspace){
        f_t_star =  Alpha_t * ( x_star - Jacob_dot * q_dot);
    }
    else{
        Eigen::VectorXd niu_t = Jacob_dash_t.transpose() * C_t  - Alpha_t * Jacob_dot * q_dot; // Operational Coriolis vector  
        //std::cout << "Niu t: \n" << niu_t << std::endl;

        Eigen::VectorXd p_t = Jacob_dash_t.transpose() * g_t; // Operational Gravity vector
        //std::cout << "P t: \n" << p_t << std::endl;

        f_t_star =  Alpha_t * x_star + niu_t + p_t; // Command forces vector for task
    }

    //std::cout << "F star: \n" << f_t_star << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Joint torque due to task

    Eigen::VectorXd tau_star =  Jacob_t.transpose() * f_t_star;
    //std::cout << "Tau star: \n" << tau_star << std::endl;

    // ------------------------------------------//
    // Project torque and add it to the total torque vector

    Eigen::VectorXd tau_projected = *Null_space_iter *  tau_star;

    *tau_total = *tau_total + tau_projected; 

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc TASK null space

    Eigen::MatrixXd Null_space_task =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_t * Jacob_t; // Null space
    //std::cout << "N_t: \n" << Null_space_task << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc LEVEL null space

    *Null_space_iter = *Null_space_iter * Null_space_task.transpose();

}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to go to a orientation with a contant velocity

void EffortTask::AchieveOrientationConstVel(Eigen::Matrix3d rot_mat_desired, 
                                            double *svd_orientation,
                                            Eigen::MatrixXd M,
                                            Eigen::VectorXd C_t,
                                            Eigen::VectorXd g_t,
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector,
                                            Eigen::VectorXd *tau_total,
                                            Eigen::MatrixXd *Null_space_iter){

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate operational space matrices

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    AngularJacobian Normal_Jacob_t = mEndEffector->getAngularJacobian(); // Angular Jacobian
    Eigen::MatrixXd Jacob_t = Normal_Jacob_t * (*Null_space_iter).transpose();
    //std::cout << "Angular Jacob: \n" << Jacob_t << std::endl;

    AngularJacobian Normal_Jacob_dot = mEndEffector->getAngularJacobianDeriv(); // Derivative of jacobian
    Eigen::MatrixXd Jacob_dot = Normal_Jacob_dot * (*Null_space_iter).transpose();

    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Angular Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate SVD for alpha

    Eigen::MatrixXd Alpha_t_inv = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix
    
    Eigen::MatrixXd Alpha_t = calcInertiaMatrix(Alpha_t_inv, svd_orientation);

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Angular Jacobian: \n" << Jacob_dash_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    Eigen::Vector3d error_ori = Eigen::Vector3d::Zero(3);

    switch(ori_error_mode){
        case 1:
            error_ori = ErrorAngleAxis1(rot_mat_desired, mRobot, mEndEffector);
            break;
        case 2:
            error_ori = ErrorAngleAxis2(rot_mat_desired, mRobot, mEndEffector);
            break;
        case 3:
            error_ori = ErrorQuaternion1(rot_mat_desired, mRobot, mEndEffector);
            break;
        case 4:
            error_ori = ErrorQuaternion2(rot_mat_desired, mRobot, mEndEffector);
            break;
        case 5:
            error_ori = ErrorQuaternion3(rot_mat_desired, mRobot, mEndEffector);
            break;
        default:
            error_ori = ErrorQuaternion3(rot_mat_desired, mRobot, mEndEffector);
    }

    Eigen::Vector3d angular_vel =  mEndEffector->getAngularJacobian() * mRobot->getVelocities();
    //std::cout << "Angular velocity vector: \n"  << angular_vel << std::endl;

    Eigen::Matrix3d kp = kp_cartesian_.bottomRightCorner(3, 3);
    Eigen::Matrix3d kd = kd_cartesian_.bottomRightCorner(3, 3);
    
    Eigen::Vector3d x_dot_desired = kp*kd.inverse()*error_ori;

    double scale = std::min(1.0, (max_angular_vel_ / x_dot_desired.norm()));
    //std::cout << "Scale V: \n" << scale << std::endl;

    Eigen::Vector3d x_star =  (-1.0*kd) * (angular_vel - scale*x_dot_desired);

    if(compensate_topdown){
        x_star = x_star - Alpha_t_inv * Jacob_dash_t.transpose() * *tau_total;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational force due to task

    Eigen::VectorXd f_t_star = Eigen::VectorXd::Zero(3);
    if(compensate_jtspace){
        f_t_star =  Alpha_t * ( x_star - Jacob_dot * q_dot);
    }
    else{
        Eigen::VectorXd niu_t = Jacob_dash_t.transpose() * C_t  - Alpha_t * Jacob_dot * q_dot; // Operational Coriolis vector  
        //std::cout << "Niu t: \n" << niu_t << std::endl;

        Eigen::VectorXd p_t = Jacob_dash_t.transpose() * g_t; // Operational Gravity vector
        //std::cout << "P t: \n" << p_t << std::endl;

        f_t_star =  Alpha_t * x_star + niu_t + p_t; // Command forces vector for task
    }

    //std::cout << "F star: \n" << f_t_star << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Joint torque due to task

    Eigen::VectorXd tau_star =  Jacob_t.transpose() * f_t_star;
    //std::cout << "Tau star: \n" << tau_star << std::endl;

    // ------------------------------------------//
    // Project torque and add it to the total torque vector

    Eigen::VectorXd tau_projected = *Null_space_iter *  tau_star;

    *tau_total = *tau_total + tau_projected; 

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc TASK null space

    Eigen::MatrixXd Null_space_task =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_t * Jacob_t; // Null space
    //std::cout << "N_t: \n" << Null_space_task << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc LEVEL null space

    *Null_space_iter = *Null_space_iter * Null_space_task.transpose();

}

/******************************************************************/
/******************************************************************/
// Error functions

// Orientation error Osorio
Eigen::Vector3d EffortTask::ErrorAngleAxis1(Eigen::Matrix3d rot_mat_desired, 
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector){


    Eigen::Matrix3d R_world_EE = mEndEffector->getWorldTransform().rotation();
    //std::cout << "Rotation matrix from W to EE: \n" << R_world_EE << std::endl;

    Eigen::Matrix3d R_EE_desired = R_world_EE.transpose() * rot_mat_desired;
    //std::cout << "Desired rotation matrix in EE: \n" << R_EE_desired << std::endl;

    Eigen::AngleAxisd u_ee (R_EE_desired);

    Eigen::Vector3d axis_desired = R_world_EE * u_ee.axis();
    double angle_desired = u_ee.angle();
    //std::cout << "Axis of vector u_world: \n"  << axis_desired  << std::endl;
    //std::cout << "Angle of vector u_world: \n" << angle_desired << std::endl;

    Eigen::Vector3d angular_vel =  mEndEffector->getAngularJacobian() * mRobot->getVelocities();
    //std::cout << "Angular velocity vector: \n"  << angular_vel << std::endl;

    Eigen::Vector3d error_ori =  axis_desired * angle_desired ; // Orientation error

    return error_ori;                                

}

// Orientation error Caccavale
Eigen::Vector3d EffortTask::ErrorAngleAxis2(Eigen::Matrix3d rot_mat_desired, 
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector){

    Eigen::Matrix3d R_world_EE = mEndEffector->getWorldTransform().rotation();
    //std::cout << "Rotation matrix from W to EE: \n" << R_world_EE << std::endl;

    Eigen::Matrix3d R_EE_desired =  rot_mat_desired * R_world_EE.transpose() ;
    //std::cout << "Desired rotation matrix in EE: \n" << R_EE_desired << std::endl;

    Eigen::AngleAxisd u_ee (R_EE_desired);

    Eigen::Vector3d error_ori =  sin(u_ee.angle())* u_ee.axis() ; // Command force vector
    
    return error_ori; 

}

// Orientation error Yuan
Eigen::Vector3d EffortTask::ErrorQuaternion1(Eigen::Matrix3d rot_mat_desired, 
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector){

    Eigen::Matrix3d R_world_EE = mEndEffector->getWorldTransform().rotation();
    //std::cout << "Rotation matrix from W to EE: \n" << R_world_EE << std::endl;

    Eigen::Quaterniond q_des(rot_mat_desired);
    Eigen::Quaterniond q_act(R_world_EE);
    
    Eigen::Vector3d error_ori = q_des.w()*q_act.vec() - q_act.w() * q_des.vec() + q_des.vec().cross(q_act.vec());
    //std::cout << "Orientation error: \n" << e_ori << std::endl;

    return error_ori;

}

// Orientation error Caccavale
Eigen::Vector3d EffortTask::ErrorQuaternion2(Eigen::Matrix3d rot_mat_desired, 
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector){
    
    Eigen::Matrix3d R_world_EE = mEndEffector->getWorldTransform().rotation();
    //std::cout << "Rotation matrix from W to EE: \n" << R_world_EE << std::endl;

    Eigen::Matrix3d R_EE_desired = R_world_EE.transpose() * rot_mat_desired; //
    //std::cout << "Desired rotation matrix in EE: \n" << R_EE_desired << std::endl;

    Eigen::Quaterniond quat(R_EE_desired);
    Eigen::Vector3d vec_des = R_world_EE * quat.vec();

    Eigen::Vector3d error_ori =   vec_des ; // Orientation error   

    return error_ori;

}

// Orientation error Caccavale
Eigen::Vector3d EffortTask::ErrorQuaternion3(Eigen::Matrix3d rot_mat_desired, 
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector){

    Eigen::Matrix3d R_world_EE = mEndEffector->getWorldTransform().rotation(); 
    //std::cout << "Rotation matrix from W to EE: \n" << R_world_EE << std::endl;

    Eigen::Matrix3d R_EE_desired = rot_mat_desired * R_world_EE.transpose();
    //std::cout << "Desired rotation matrix in EE: \n" << R_EE_desired << std::endl;

    Eigen::Quaterniond quat(R_EE_desired);
    //Eigen::Vector3d vec_des = R_world_EE * quat.vec();

    Eigen::Vector3d error_ori = 2*quat.w()*quat.vec() ; 

    return error_ori;

}

} // end namespace