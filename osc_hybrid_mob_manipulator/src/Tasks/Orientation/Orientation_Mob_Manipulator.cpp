#include <osc_hybrid_mob_manipulator/Effort_Tasks.hpp>

namespace effort_tasks {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to implement a Orientation Impedance 

void EffortTask::OrientationImpedance(Eigen::Matrix3d rot_mat_desired, 
                                    Eigen::Vector3d mTargetVel,
                                    Eigen::Vector3d mTargetAccel,
                                    double *svd_orientation,
                                    Eigen::VectorXd tau_ext,
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

    Eigen::MatrixXd Jacob_t = mEndEffector->getAngularJacobian(); // Angular Jacobian
    if(augmented_projections){
        Jacob_t = Jacob_t * (*Null_space_iter).transpose();
    }
    //std::cout << "Angular Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = mEndEffector->getAngularJacobianDeriv(); // Derivative of jacobian
    if(augmented_projections){
        Jacob_dot = Jacob_dot * (*Null_space_iter).transpose();
    }
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

    // Obtain Desired dynamics matrices
    Eigen::MatrixXd Inertia_Matrix_d = 1.0*Eigen::MatrixXd::Identity(3,3);//Alpha_t;
    Eigen::MatrixXd Stiff_Matrix_d = kp_cartesian_.bottomRightCorner(3, 3);

    Eigen::MatrixXd damp_coeff = kd_cartesian_.bottomRightCorner(3, 3);
    Eigen::MatrixXd Damping_Matrix_d = calcDampingMatrix(Inertia_Matrix_d, Stiff_Matrix_d, damp_coeff); 
    //std::cout << "Damping Ori: \n" << kd << std::endl;

    Eigen::Vector3d x_star = mTargetAccel + Inertia_Matrix_d.inverse() * (Damping_Matrix_d*(mTargetVel-angular_vel) + Stiff_Matrix_d*error_ori - (Jacob_dash_t.transpose() * tau_ext)); // Command force vector
    
    if(compensate_topdown){
        x_star = x_star - Jacob_dash_t.transpose() * *tau_total;
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

    Eigen::MatrixXd Jacob_t = mEndEffector->getAngularJacobian(); // Angular Jacobian
    if(augmented_projections){
        Jacob_t = Jacob_t * (*Null_space_iter).transpose();
    }
    //std::cout << "Angular Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = mEndEffector->getAngularJacobianDeriv(); // Derivative of jacobian
    if(augmented_projections){
        Jacob_dot = Jacob_dot * (*Null_space_iter).transpose();
    }
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

    // Obtain Orientation Gain matrices
    Eigen::MatrixXd kp = kp_cartesian_.bottomRightCorner(3, 3);

    Eigen::MatrixXd damp_coeff = kd_cartesian_.bottomRightCorner(3, 3);
    Eigen::MatrixXd kd = calcDampingMatrix(Eigen::MatrixXd::Identity(3,3), kp, damp_coeff); 
    //std::cout << "Damping Ori: \n" << kd << std::endl;

    Eigen::Vector3d x_star = mTargetAccel + kd *(mTargetVel-angular_vel) + kp * error_ori ; 
    
    if(compensate_topdown){
        x_star = x_star - Jacob_dash_t.transpose() * *tau_total;
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

    Eigen::MatrixXd Jacob_t = mEndEffector->getAngularJacobian(); // Angular Jacobian
    if(augmented_projections){
        Jacob_t = Jacob_t * (*Null_space_iter).transpose();
    }
    //std::cout << "Angular Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = mEndEffector->getAngularJacobianDeriv(); // Derivative of jacobian
    if(augmented_projections){
        Jacob_dot = Jacob_dot * (*Null_space_iter).transpose();
    }
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

    // Obtain Orientation Gain matrices
    Eigen::MatrixXd kp = kp_cartesian_.bottomRightCorner(3, 3);

    Eigen::MatrixXd damp_coeff = kd_cartesian_.bottomRightCorner(3, 3);
    Eigen::MatrixXd kd = calcDampingMatrix(Eigen::MatrixXd::Identity(3,3), kp, damp_coeff); 
    //std::cout << "Damping Ori: \n" << kd << std::endl;
    
    Eigen::Vector3d x_dot_desired = kp*kd.inverse()*error_ori;

    double scale = std::min(1.0, (max_angular_vel_ / x_dot_desired.norm()));
    //std::cout << "Scale V: \n" << scale << std::endl;

    Eigen::Vector3d x_star =  (-1.0*kd) * (angular_vel - scale*x_dot_desired);

    if(compensate_topdown){
        x_star = x_star - Jacob_dash_t.transpose() * *tau_total;
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

} // end namespace