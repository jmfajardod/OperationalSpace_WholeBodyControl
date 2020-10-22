#include <osc_hybrid_mob_manipulator/Effort_Tasks.hpp>

namespace effort_tasks {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void EffortTask::OLD_MakeStraightLine(  Eigen::VectorXd *tau_zero,
                                        Eigen::VectorXd *tau_result,
                                        Eigen::Vector3d mTarget, 
                                        Eigen::MatrixXd M, 
                                        Eigen::VectorXd C_t,
                                        Eigen::VectorXd g_t,
                                        dart::dynamics::SkeletonPtr mRobot,
                                        dart::dynamics::BodyNode* mEndEffector){

    // ------------------------------------------//
    // ------------------------------------------//
    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    LinearJacobian Jacob_t = mEndEffector->getLinearJacobian(); // Jacobian
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Alpha_t = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix
    Alpha_t = Alpha_t.inverse() ;
    //std::cout << "Alpha_t: \n" << Alpha_t << std::endl;

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    LinearJacobian Jacob_dot = mEndEffector->getLinearJacobianDeriv(); // Derivative of jacobian
    Eigen::VectorXd q_dot    = mRobot->getVelocities();                // Derivative of the joints
    //std::cout << "Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    Eigen::VectorXd niu_t = Jacob_dash_t.transpose() * C_t  - Alpha_t * Jacob_dot * q_dot; // Operational Coriolis vector  
    //std::cout << "Niu t: \n" << niu_t << std::endl;

    Eigen::VectorXd p_t = Jacob_dash_t.transpose() * g_t; // Operational Gravity vector
    //std::cout << "P t: \n" << p_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//

    //std::cout<< "Cartesian Target: \n" << mTarget << std::endl;
    //std::cout<< "EE Pos: \n" << mEndEffector->getWorldTransform().translation() << std::endl;

    Eigen::Vector3d x_error =  mTarget - mEndEffector->getWorldTransform().translation(); // Position error
    //std::cout<< "Cartesian error: \n" << x_error << std::endl;

    Eigen::Matrix3d kp = kp_cartesian_.topLeftCorner(3, 3);
    Eigen::Matrix3d kd = kd_cartesian_.topLeftCorner(3, 3);

    Eigen::Vector3d x_dot_desired = kp*kd.inverse()*x_error;

    double scale = std::min(1.0, (max_lineal_vel_ / x_dot_desired.norm()));
    //std::cout << "Scale V: \n" << scale << std::endl;

    Eigen::Vector3d f_t_star =  (-1.0*kd) * (mEndEffector->getLinearVelocity() - scale*x_dot_desired); // Command force vector
    //std::cout << "F star: \n" << f_t_star << std::endl;

    Eigen::VectorXd tau_star =  Alpha_t * f_t_star + niu_t + p_t; // Command torques vector for task
    tau_star =  Jacob_t.transpose() * tau_star;
    //std::cout << "Tau star: \n" << tau_star << std::endl;

    Eigen::MatrixXd N_t =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_t.transpose() * Jacob_dash_t.transpose(); // Null space
    //std::cout << "N_t: \n" << N_t << std::endl;

    *tau_result = tau_star + N_t * *tau_zero;  
    *tau_zero   = *tau_result;
    //std::cout << "Forces: \n" << tau_result << std::endl;
}



////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian orientation Caccavale

void EffortTask::OLD_AchieveOrientationQuat3(Eigen::VectorXd *tau_zero, 
                                            Eigen::VectorXd *tau_result,
                                            Eigen::Matrix3d rot_mat_desired, 
                                            Eigen::MatrixXd M,
                                            Eigen::VectorXd C_t,
                                            Eigen::VectorXd g_t,
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector){

    // ------------------------------------------//
    // ------------------------------------------//
    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    AngularJacobian Jacob_t = mEndEffector->getAngularJacobian(); // Angular Jacobian
    //std::cout << "Angular Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Alpha_t = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix
    Alpha_t = Alpha_t.inverse() ;
    //std::cout << "Alpha_t: \n" << Alpha_t << std::endl;

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Angular Jacobian: \n" << Jacob_dash_t << std::endl;

    AngularJacobian Jacob_dot = mEndEffector->getAngularJacobianDeriv(); // Derivative of jacobian
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Angular Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    Eigen::VectorXd niu_t = Jacob_dash_t.transpose() * C_t  - Alpha_t * Jacob_dot * q_dot; // Operational Coriolis vector  
    //std::cout << "Niu t: \n" << niu_t << std::endl;

    Eigen::VectorXd p_t = Jacob_dash_t.transpose() * g_t; // Operational Gravity vector
    //std::cout << "P t: \n" << p_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//

    Eigen::Matrix3d R_world_EE = mEndEffector->getWorldTransform().rotation(); 
    //std::cout << "Rotation matrix from W to EE: \n" << R_world_EE << std::endl;

    Eigen::Matrix3d R_EE_desired = rot_mat_desired * R_world_EE.transpose();
    //std::cout << "Desired rotation matrix in EE: \n" << R_EE_desired << std::endl;

    Eigen::Quaterniond quat(R_EE_desired);
    //Eigen::Vector3d vec_des = R_world_EE * quat.vec();

    Eigen::Vector3d angular_vel =  mEndEffector->getAngularJacobian() * mRobot->getVelocities();
    //std::cout << "Angular velocity vector: \n"  << angular_vel << std::endl;

    Eigen::Matrix3d kp = kp_cartesian_.bottomRightCorner(3, 3);
    Eigen::Matrix3d kd = kd_cartesian_.bottomRightCorner(3, 3);

    Eigen::Vector3d f_t_star =   kd *(-angular_vel) + kp * (2*quat.w()*quat.vec()) ; // Command force vector
    //std::cout << "F star: \n" << f_t_star << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//

    Eigen::VectorXd tau_star =  Alpha_t * f_t_star + niu_t + p_t; // Command torques vector for task
    tau_star =  Jacob_t.transpose() * tau_star;
    //std::cout << "Tau star: \n" << tau_star << std::endl;

    Eigen::MatrixXd N_t =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_t.transpose() * Jacob_dash_t.transpose(); // Null space
    //std::cout << "N_t: \n" << N_t << std::endl;

    *tau_result = tau_star + N_t * *tau_zero;  
    *tau_zero   = *tau_result;
    //std::cout << "Forces: \n" << tau_result << std::endl;
}


////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a joint configuration

void EffortTask::OLD_AchieveJointConf(  Eigen::VectorXd *tau_zero,
                                        Eigen::VectorXd *tau_result,
                                        Eigen::VectorXd q_desired, 
                                        Eigen::MatrixXd M, 
                                        Eigen::VectorXd C_t,
                                        Eigen::VectorXd g_t,
                                        dart::dynamics::SkeletonPtr mRobot,
                                        dart::dynamics::BodyNode* mEndEffector){

    // ------------------------------------------//
    // ------------------------------------------//
    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    Eigen::VectorXd q_dot_desired = Eigen::MatrixXd::Zero(dofs,1); 
    //std::cout << "q desired: \n" << q_desired << std::endl;
    //std::cout << "q dot desired: \n" << q_dot_desired << std::endl;

    Eigen::VectorXd e = q_desired - mRobot->getPositions(); // Position error
    Eigen::VectorXd de = q_dot_desired - mRobot->getVelocities();  // Velocity error (Target velocity is zero)

    Eigen::VectorXd f_t_star = kd_joints_ * de; // kp_joints_ * e + kd_joints_ * de
    //std::cout << "F star: \n" << f_t_star << std::endl;

    Eigen::VectorXd tau_star = M*f_t_star + C_t + g_t;  // Command torques vector for task
    //std::cout << "Tau star: \n" << tau_star << std::endl;

    Eigen::MatrixXd N_t =  Eigen::MatrixXd::Zero(dofs, dofs);

    *tau_result = tau_star + N_t * *tau_zero;  
    *tau_zero   = *tau_result;
    //std::cout << "Forces: \n" << mForces << std::endl;

}

} // End namespace