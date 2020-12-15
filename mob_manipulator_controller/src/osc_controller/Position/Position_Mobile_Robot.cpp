#include <osc_controller/OSC_Controller.hpp>

namespace osc_controller {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void OSC_Controller::AchieveCartesianMobilRob( Eigen::Vector3d mTargetPos, 
                                            Eigen::Vector3d mTargetVel,
                                            Eigen::Vector3d mTargetAccel, 
                                            double *svd_position,
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

    Eigen::MatrixXd Jacob_t = mEndEffector->getLinearJacobian().topRows(2); // Jacobian
    Jacob_t.bottomRightCorner(2,6) = Eigen::MatrixXd::Zero(2,6);
    if(augmented_projections){ 
        Jacob_t = Jacob_t * (*Null_space_iter).transpose();
    }
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = mEndEffector->getLinearJacobianDeriv().topRows(2); // Derivative of jacobian
    Jacob_dot.bottomRightCorner(2,6) = Eigen::MatrixXd::Zero(2,6);
    if(augmented_projections){ 
        Jacob_dot = Jacob_dot * (*Null_space_iter).transpose();
    }
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate SVD for alpha

    Eigen::MatrixXd Alpha_t_inv = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix

    //std::cout << "Inverse Inertia Matrix: \n" << Alpha_t_inv << std::endl;
    
    Eigen::MatrixXd Alpha_t = Alpha_t_inv.inverse(); // Mobile base do not suffer from singularities in XY position task
    //Eigen::MatrixXd Alpha_t = calcInertiaMatrix(Alpha_t_inv, svd_position);

    //std::cout << "Inertia Matrix: \n" << Alpha_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    Eigen::VectorXd e =  mTargetPos - mEndEffector->getWorldTransform().translation(); // Position error
    //std::cout<< "Error : \n" << e << std::endl;

    Eigen::VectorXd de = mTargetVel - mEndEffector->getLinearVelocity();  // Velocity error (Target velocity is zero)

    // Obtain Position Gain matrices
    Eigen::MatrixXd kp = kp_cartesian_.topLeftCorner(2, 2);

    Eigen::MatrixXd damp_coeff = kd_cartesian_.topLeftCorner(2, 2);
    Eigen::MatrixXd kd = calcDampingMatrix(Eigen::MatrixXd::Identity(2,2), kp, damp_coeff); 

    Eigen::VectorXd x_star = mTargetAccel.topRows(2) + kd*de.topRows(2) + kp*e.topRows(2) ; // Command force vector
    //std::cout << "Command Accel: \n" << x_star << std::endl;

    if(compensate_topdown){
        x_star = x_star - Jacob_dash_t.transpose() * *tau_total;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational force due to task

    Eigen::VectorXd f_t_star = Eigen::VectorXd::Zero(2);
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
    //std::cout << "Tau projected: \n" << tau_projected << std::endl;

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

void OSC_Controller::AchieveCartesianMobilRobConstVel(  Eigen::Vector3d mTargetPos,
                                                    double *svd_position,
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

    Eigen::MatrixXd Jacob_t = mEndEffector->getLinearJacobian().topRows(2); // Jacobian
    Jacob_t.bottomRightCorner(2,6) = Eigen::MatrixXd::Zero(2,6);
    if(augmented_projections){
        Jacob_t = Jacob_t * (*Null_space_iter).transpose();
    }
    //std::cout << "Projected Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = mEndEffector->getLinearJacobianDeriv().topRows(2); // Derivative of jacobian
    Jacob_dot.bottomRightCorner(2,6) = Eigen::MatrixXd::Zero(2,6);
    if(augmented_projections){
        Jacob_dot = Jacob_dot * (*Null_space_iter).transpose();
    }
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate SVD for alpha

    Eigen::MatrixXd Alpha_t_inv = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix
    //std::cout << "Inverse Inertia Matrix: \n" << Alpha_t_inv << std::endl;
    
    Eigen::MatrixXd Alpha_t = Alpha_t_inv.inverse(); // Mobile base do not suffer from singularities in XY position task
    //Eigen::MatrixXd Alpha_t = calcInertiaMatrix(Alpha_t_inv, svd_position);

    //std::cout << "Inertia Matrix: \n" << Alpha_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    Eigen::VectorXd x_error =  (mTargetPos - mEndEffector->getWorldTransform().translation()).topRows(2); // Position error
    //std::cout<< "Error : \n" << x_error << std::endl;

    // Obtain Position Gain matrices
    Eigen::MatrixXd kp = kp_cartesian_.topLeftCorner(2, 2);

    Eigen::MatrixXd damp_coeff = kd_cartesian_.topLeftCorner(2, 2);
    Eigen::MatrixXd kd = calcDampingMatrix(Eigen::MatrixXd::Identity(2,2), kp, damp_coeff); 

    Eigen::VectorXd x_dot_desired = kp*kd.inverse()*x_error;
    double scale = std::min(1.0, (10*max_lineal_vel_ / x_dot_desired.norm()));
    Eigen::VectorXd x_star =  (-1.0*kd) * ((mEndEffector->getLinearVelocity()).topRows(2) - scale*x_dot_desired); // Command force vector
    //std::cout << "Scale V: \n" << scale << std::endl;
    //std::cout << "Command Accel: \n" << x_star << std::endl;

    if(compensate_topdown){
        x_star = x_star - Jacob_dash_t.transpose() * *tau_total;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational force due to task

    Eigen::VectorXd f_t_star = Eigen::VectorXd::Zero(2);
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
    //std::cout << "Tau projected: \n" << tau_projected << std::endl;

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
