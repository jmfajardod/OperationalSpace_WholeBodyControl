#include <osc_hybrid_mob_manipulator/Effort_Tasks.hpp>

namespace effort_tasks {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position
// without the trajectory

void EffortTask::AchieveCartesian(  Eigen::Vector3d mTarget, 
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

    LinearJacobian Jacob_t = mEndEffector->getLinearJacobian(); // Jacobian
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Alpha_t = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix
    Alpha_t = Alpha_t.inverse() ;
    //std::cout << "Alpha_t: \n" << Alpha_t << std::endl;

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    LinearJacobian Jacob_dot = mEndEffector->getLinearJacobianDeriv(); // Derivative of jacobian
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    Eigen::VectorXd niu_t = Jacob_dash_t.transpose() * C_t  - Alpha_t * Jacob_dot * q_dot; // Operational Coriolis vector  
    //std::cout << "Niu t: \n" << niu_t << std::endl;

    Eigen::VectorXd p_t = Jacob_dash_t.transpose() * g_t; // Operational Gravity vector
    //std::cout << "P t: \n" << p_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational Force due to task

    //std::cout<< "Cartesian Target: \n" << mTarget << std::endl;
    //std::cout<< "EE Pos: \n" << mEndEffector->getWorldTransform().translation() << std::endl;

    Eigen::Vector3d e =  mTarget - mEndEffector->getWorldTransform().translation() ; // Position error
    //std::cout<< "Error : \n" << e << std::endl;

    Eigen::Vector3d de = Eigen::Vector3d::Zero(3) - mEndEffector->getLinearVelocity();  // Velocity error (Target velocity is zero)

    // Obtain Position Gain matrices
    Eigen::Matrix3d kp = kp_cartesian_.topLeftCorner(3, 3);
    Eigen::Matrix3d kd = kd_cartesian_.topLeftCorner(3, 3);

    Eigen::Vector3d x_star =  kd*de + kp*e ; // Command force vector
    //std::cout << "F star: \n" << f_t_star << std::endl;

    Eigen::VectorXd f_t_star =  Alpha_t * x_star + niu_t + p_t; // Command forces vector for task

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
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void EffortTask::MakeStraightLine(  Eigen::Vector3d mTarget, 
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

    LinearJacobian Jacob_t = mEndEffector->getLinearJacobian(); // Jacobian
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Alpha_t = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix
    Alpha_t = Alpha_t.inverse() ;
    //std::cout << "Alpha_t: \n" << Alpha_t << std::endl;

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    LinearJacobian Jacob_dot = mEndEffector->getLinearJacobianDeriv(); // Derivative of jacobian
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    Eigen::VectorXd niu_t = Jacob_dash_t.transpose() * C_t  - Alpha_t * Jacob_dot * q_dot; // Operational Coriolis vector  
    //std::cout << "Niu t: \n" << niu_t << std::endl;

    Eigen::VectorXd p_t = Jacob_dash_t.transpose() * g_t; // Operational Gravity vector
    //std::cout << "P t: \n" << p_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational Force due to task

    //std::cout<< "Cartesian Target: \n" << mTarget << std::endl;
    //std::cout<< "EE Pos: \n" << mEndEffector->getWorldTransform().translation() << std::endl;

    Eigen::Vector3d x_error =  mTarget - mEndEffector->getWorldTransform().translation(); // Position error
    //std::cout<< "Cartesian error: \n" << x_error << std::endl;

    // Obtain Position Gain matrices
    Eigen::Matrix3d kp = kp_cartesian_.topLeftCorner(3, 3);
    Eigen::Matrix3d kd = kd_cartesian_.topLeftCorner(3, 3);

    Eigen::Vector3d x_dot_desired = kp*kd.inverse()*x_error;

    double scale = std::min(1.0, (max_vel_ / x_dot_desired.norm()));
    //std::cout << "Scale V: \n" << scale << std::endl;

    Eigen::Vector3d x_star =  (-1.0*kd) * (mEndEffector->getLinearVelocity() - scale*x_dot_desired); // Command force vector
    
    Eigen::VectorXd f_t_star =  Alpha_t * x_star + niu_t + p_t; // Command forces vector for task
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
