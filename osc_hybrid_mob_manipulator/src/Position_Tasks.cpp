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
    // Calculate operational space matrices 

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    LinearJacobian Jacob_t = mEndEffector->getLinearJacobian(); // Jacobian
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    LinearJacobian Jacob_dot = mEndEffector->getLinearJacobianDeriv(); // Derivative of jacobian
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate SVD for alpha

    Eigen::MatrixXd Alpha_t_inv = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix

    Eigen::MatrixXd Alpha_t = calcInertiaMatrix(Alpha_t_inv);

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    //std::cout<< "Cartesian Target: \n" << mTarget << std::endl;
    //std::cout<< "EE Pos: \n" << mEndEffector->getWorldTransform().translation() << std::endl;

    Eigen::Vector3d e =  mTarget - mEndEffector->getWorldTransform().translation() ; // Position error
    //std::cout<< "Error : \n" << e << std::endl;

    Eigen::Vector3d de = Eigen::Vector3d::Zero(3) - mEndEffector->getLinearVelocity();  // Velocity error (Target velocity is zero)

    // Obtain Position Gain matrices
    Eigen::Matrix3d kp = kp_cartesian_.topLeftCorner(3, 3);
    Eigen::Matrix3d kd = kd_cartesian_.topLeftCorner(3, 3);

    Eigen::Vector3d x_star =  kd*de + kp*e ; // Command force vector
    if(compensate_topdown){
        x_star = x_star + Jacob_dash_t.transpose() * *tau_total;
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
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void EffortTask::AchieveCartesianConstVel(  Eigen::Vector3d mTarget, 
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

    LinearJacobian Jacob_dot = mEndEffector->getLinearJacobianDeriv(); // Derivative of jacobian
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate SVD for alpha

    Eigen::MatrixXd Alpha_t_inv = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix

    Eigen::MatrixXd Alpha_t = calcInertiaMatrix(Alpha_t_inv);

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    //std::cout<< "Cartesian Target: \n" << mTarget << std::endl;
    //std::cout<< "EE Pos: \n" << mEndEffector->getWorldTransform().translation() << std::endl;

    Eigen::Vector3d x_error =  mTarget - mEndEffector->getWorldTransform().translation(); // Position error
    //std::cout<< "Cartesian error: \n" << x_error << std::endl;

    // Obtain Position Gain matrices
    Eigen::Matrix3d kp = kp_cartesian_.topLeftCorner(3, 3);
    Eigen::Matrix3d kd = kd_cartesian_.topLeftCorner(3, 3);

    Eigen::Vector3d x_dot_desired = kp*kd.inverse()*x_error;

    double scale = std::min(1.0, (max_lineal_vel_ / x_dot_desired.norm()));
    //std::cout << "Scale V: \n" << scale << std::endl;

    Eigen::Vector3d x_star =  (-1.0*kd) * (mEndEffector->getLinearVelocity() - scale*x_dot_desired); // Command force vector
    if(compensate_topdown){
        x_star = x_star + Jacob_dash_t.transpose() * *tau_total;
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
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void EffortTask::AchieveCartesianMobilRob(  Eigen::Vector3d mTarget, 
                                            Eigen::MatrixXd Full_M, 
                                            Eigen::VectorXd Full_C_t,
                                            Eigen::VectorXd Full_g_t,
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector,
                                            Eigen::VectorXd *tau_total,
                                            Eigen::MatrixXd *Null_space_iter){

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate operational space matrices 

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    Eigen::MatrixXd Jacob_t = (mEndEffector->getLinearJacobian()).topLeftCorner(2,2); // Jacobian
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = (mEndEffector->getLinearJacobianDeriv()).topLeftCorner(2,2);// Derivative of jacobian
    Eigen::VectorXd q_dot = mRobot->getVelocities().topRows(2);            // Derivative of the joints
    //std::cout << "Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate SVD for alpha

    Eigen::MatrixXd M = Full_M.topLeftCorner(2,2);
    Eigen::VectorXd C_t = Full_C_t.topRows(2);
    Eigen::VectorXd g_t = Full_g_t.topRows(2);

    //std::cout<< "Joint Inertia matrix: \n" << M << std::endl;

    Eigen::MatrixXd Alpha_t_inv = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix

    //std::cout << "Inverse Inertia matrix: \n" << Alpha_t_inv << std::endl;

    Eigen::MatrixXd Alpha_t = Alpha_t_inv.inverse();

    //std::cout << "Inertia matrix: \n" << Alpha_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    //std::cout<< "Cartesian Target: \n" << mTarget << std::endl;
    //std::cout<< "EE Pos: \n" << mEndEffector->getWorldTransform().translation() << std::endl;

    Eigen::VectorXd x_error =  (mTarget - mEndEffector->getWorldTransform().translation()).topRows(2); // Position error
    //std::cout<< "Cartesian error: \n" << x_error << std::endl;

    // Obtain Position Gain matrices
    Eigen::MatrixXd kp = 0.003*(kp_cartesian_.topLeftCorner(2, 2));
    Eigen::MatrixXd kd = 0.005*(kd_cartesian_.topLeftCorner(2, 2));

    //Eigen::VectorXd x_dot_desired = kp*kd.inverse()*x_error;
    //double scale = std::min(1.0, (max_lineal_vel_ / x_dot_desired.norm()));
    //std::cout << "Scale V: \n" << scale << std::endl;
    //Eigen::VectorXd x_star =  (-1.0*kd) * ((mEndEffector->getLinearVelocity()).topRows(2) - scale*x_dot_desired); // Command force vector
    
    Eigen::VectorXd de = Eigen::VectorXd::Zero(2) - (mEndEffector->getLinearVelocity()).topRows(2);
    Eigen::VectorXd x_star =  kd*de + kp*x_error ; // Command force vector
    
    //std::cout << "Ref accel: \n" << x_star << std::endl; 

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

    (*tau_total)(0) = (*tau_total)(0) + tau_star(0);
    (*tau_total)(1) = (*tau_total)(1) + tau_star(1);

    //std::cout << "Tau total: \n" << *tau_total << std::endl; 

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc TASK null space

    Eigen::MatrixXd Null_space_task =  Eigen::MatrixXd::Identity(dofs, dofs); // Null space
    Null_space_task.topLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    //std::cout << "N_t: \n" << Null_space_task << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc LEVEL null space

    *Null_space_iter = *Null_space_iter * Null_space_task.transpose();

}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void EffortTask::AchieveHeightConstVel( Eigen::Vector3d mTarget, 
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

    Eigen::MatrixXd Jacob_t = (mEndEffector->getLinearJacobian()).bottomRightCorner(1,dofs); // Jacobian
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = (mEndEffector->getLinearJacobianDeriv()).bottomRightCorner(1,dofs); // Derivative of jacobian
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate SVD for alpha

    Eigen::MatrixXd Alpha_t_inv = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix

    Eigen::MatrixXd Alpha_t = calcInertiaMatrix(Alpha_t_inv);

    //std::cout << "Inertia matrix: \n" << Alpha_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    //std::cout<< "Cartesian Target: \n" << mTarget << std::endl;
    //std::cout<< "EE Pos: \n" << mEndEffector->getWorldTransform().translation() << std::endl;

    Eigen::VectorXd x_error =  (mTarget - mEndEffector->getWorldTransform().translation()).bottomRows(1); // Position error
    //std::cout<< "Cartesian error: \n" << x_error << std::endl;

    // Obtain Position Gain matrices
    Eigen::MatrixXd kp = (kp_cartesian_.topLeftCorner(3,3)).bottomRightCorner(1,1);
    Eigen::MatrixXd kd = (kd_cartesian_.topLeftCorner(3,3)).bottomRightCorner(1,1);

    Eigen::VectorXd x_dot_desired = kp*kd.inverse()*x_error;

    double scale = std::min(1.0, (max_lineal_vel_ / x_dot_desired.norm()));
    //std::cout << "Scale V: \n" << scale << std::endl;

    Eigen::VectorXd x_star =  (-1.0*kd) * ( (mEndEffector->getLinearVelocity()).bottomRows(1) - scale*x_dot_desired); // Command force vector
    
    //std::cout << "Ref Accel: \n" << x_star << std::endl;
    
    if(compensate_topdown){
        x_star = x_star + Jacob_dash_t.transpose() * *tau_total;
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
