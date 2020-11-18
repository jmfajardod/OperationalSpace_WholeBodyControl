#include <osc_hybrid_mob_manipulator/Effort_Tasks.hpp>

namespace effort_tasks {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position
// without the trajectory

void EffortTask::CartesianImpedance(  Eigen::Vector3d mTargetPos, 
                                    Eigen::Vector3d mTargetVel,
                                    Eigen::Vector3d mTargetAccel, 
                                    double *svd_position,
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

    Eigen::MatrixXd Jacob_t = mEndEffector->getLinearJacobian(); // Jacobian
    if(augmented_projections){
        Jacob_t = Jacob_t * (*Null_space_iter).transpose();
    }
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = mEndEffector->getLinearJacobianDeriv(); // Derivative of jacobian
    if(augmented_projections){
        Jacob_dot = Jacob_dot * (*Null_space_iter).transpose();
    }
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate SVD for alpha

    singularity_thres_high_ = singularity_thres_high_pos_;
    singularity_thres_low_  = singularity_thres_low_pos_;

    Eigen::MatrixXd Alpha_t_inv = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix

    Eigen::MatrixXd Alpha_ns;
    Eigen::MatrixXd Alpha_s;
    Eigen::MatrixXd Alpha_s_dummy;
    double act_param = 0;

    calcInertiaMatrixHandling( Alpha_t_inv, svd_position, &act_param, &Alpha_ns, &Alpha_s, &Alpha_s_dummy);
    //std::cout << "Inertia Matrix: " << Alpha_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Base_Jacob_dash = M.inverse() * Jacob_t.transpose();
    Eigen::MatrixXd Jacob_dash_ns = Base_Jacob_dash * Alpha_ns; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_s = Base_Jacob_dash * Alpha_s; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_dummy = Base_Jacob_dash * Alpha_s_dummy; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    Eigen::Vector3d e =  mTargetPos - mEndEffector->getWorldTransform().translation() ; // Position error
    //std::cout<< "Error : \n" << e << std::endl;

    Eigen::Vector3d de = mTargetVel - mEndEffector->getLinearVelocity();  // Velocity error (Target velocity is zero)

    // Obtain Desired dynamics matrices
    Eigen::MatrixXd Inertia_Matrix_d = 1.0*Eigen::MatrixXd::Identity(3,3);//Alpha_t;
    Eigen::MatrixXd Stiff_Matrix_d = kp_cartesian_.topLeftCorner(3, 3);

    Eigen::MatrixXd damp_coeff = kd_cartesian_.topLeftCorner(3, 3);
    Eigen::MatrixXd Damping_Matrix_d = calcDampingMatrix(Inertia_Matrix_d, Stiff_Matrix_d, damp_coeff); 

    Eigen::Vector3d x_star = mTargetAccel + Inertia_Matrix_d.inverse() * (Damping_Matrix_d*de + Stiff_Matrix_d*e - (Jacob_dash_ns.transpose() * tau_ext)); // Command force vector
    
    if(compensate_topdown){
        x_star = x_star - Jacob_dash_ns.transpose() * *tau_total;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational force due to task

    Eigen::VectorXd f_star_ns = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd f_star_s = Eigen::VectorXd::Zero(3);

    if(compensate_jtspace){
        f_star_ns =  Alpha_ns * ( x_star - Jacob_dot * q_dot);
        f_star_s  =  Alpha_s  * ( x_star - Jacob_dot * q_dot);
    }
    else{
        Eigen::VectorXd niu_ns = Jacob_dash_ns.transpose() * C_t  - Alpha_ns * Jacob_dot * q_dot; // Operational Coriolis vector  
        Eigen::VectorXd p_ns = Jacob_dash_ns.transpose() * g_t; // Operational Gravity vector
        f_star_ns =  Alpha_ns * x_star + niu_ns + p_ns; // Command forces vector for task

        Eigen::VectorXd niu_s = Jacob_dash_s.transpose() * C_t  - Alpha_s * Jacob_dot * q_dot; // Operational Coriolis vector  
        Eigen::VectorXd p_s = Jacob_dash_s.transpose() * g_t; // Operational Gravity vector
        f_star_s =  Alpha_s * x_star + niu_s + p_s; // Command forces vector for task

    }

    f_star_s = act_param * f_star_s; // Scale Singular task by activation parameter

    //std::cout << "F star Pos Non-singular: \n" << f_star_ns << std::endl;
    //std::cout << "F star Pos Singular: \n" << f_star_s << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Joint torque due to task

    Eigen::VectorXd tau_star =  Jacob_t.transpose() * (f_star_ns + f_star_s);
    //std::cout << "Tau star: \n" << tau_star << std::endl;

    // ------------------------------------------//
    // Project torque and add it to the total torque vector

    Eigen::VectorXd tau_projected = *Null_space_iter *  tau_star;
    //std::cout << "Tau projected: \n" << tau_projected << std::endl;

    *tau_total = *tau_total + tau_projected; 

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc null space

    Eigen::MatrixXd Null_space_ns =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_ns * Jacob_t; // Null space
    *Null_space_iter = *Null_space_iter * Null_space_ns.transpose();

    Eigen::MatrixXd Null_space_s =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_dummy * Jacob_t; // Null space
    *Null_space_iter = *Null_space_iter * Null_space_s.transpose();

}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void EffortTask::AchieveCartesian(  Eigen::Vector3d mTargetPos, 
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

    Eigen::MatrixXd Jacob_t = mEndEffector->getLinearJacobian(); // Jacobian
    if(augmented_projections){
        Jacob_t = Jacob_t * (*Null_space_iter).transpose();
    }
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = mEndEffector->getLinearJacobianDeriv(); // Derivative of jacobian
    if(augmented_projections){
        Jacob_dot = Jacob_dot * (*Null_space_iter).transpose();
    }
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate SVD for alpha

    singularity_thres_high_ = singularity_thres_high_pos_;
    singularity_thres_low_  = singularity_thres_low_pos_;

    Eigen::MatrixXd Alpha_t_inv = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix

    Eigen::MatrixXd Alpha_ns;
    Eigen::MatrixXd Alpha_s;
    Eigen::MatrixXd Alpha_s_dummy;
    double act_param = 0;

    calcInertiaMatrixHandling( Alpha_t_inv, svd_position, &act_param, &Alpha_ns, &Alpha_s, &Alpha_s_dummy);
    //std::cout << "Inertia Matrix: " << Alpha_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Base_Jacob_dash = M.inverse() * Jacob_t.transpose();
    Eigen::MatrixXd Jacob_dash_ns = Base_Jacob_dash * Alpha_ns; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_s = Base_Jacob_dash * Alpha_s; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_dummy = Base_Jacob_dash * Alpha_s_dummy; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    Eigen::Vector3d e =  mTargetPos - mEndEffector->getWorldTransform().translation() ; // Position error
    //std::cout<< "Error : \n" << e << std::endl;

    Eigen::Vector3d de = mTargetVel - mEndEffector->getLinearVelocity();  // Velocity error (Target velocity is zero)

    // Obtain Position Gain matrices
    Eigen::MatrixXd kp = kp_cartesian_.topLeftCorner(3, 3);

    Eigen::MatrixXd damp_coeff = kd_cartesian_.topLeftCorner(3, 3);
    Eigen::MatrixXd kd = calcDampingMatrix(Eigen::MatrixXd::Identity(3,3), kp, damp_coeff); 

    Eigen::Vector3d x_star = mTargetAccel + kd*de + kp*e ; // Command force vector
    if(compensate_topdown){
        x_star = x_star - Jacob_dash_ns.transpose() * *tau_total;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational force due to task

    Eigen::VectorXd f_star_ns = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd f_star_s = Eigen::VectorXd::Zero(3);

    if(compensate_jtspace){
        f_star_ns =  Alpha_ns * ( x_star - Jacob_dot * q_dot);
        f_star_s  =  Alpha_s  * ( x_star - Jacob_dot * q_dot);
    }
    else{
        Eigen::VectorXd niu_ns = Jacob_dash_ns.transpose() * C_t  - Alpha_ns * Jacob_dot * q_dot; // Operational Coriolis vector  
        Eigen::VectorXd p_ns = Jacob_dash_ns.transpose() * g_t; // Operational Gravity vector
        f_star_ns =  Alpha_ns * x_star + niu_ns + p_ns; // Command forces vector for task

        Eigen::VectorXd niu_s = Jacob_dash_s.transpose() * C_t  - Alpha_s * Jacob_dot * q_dot; // Operational Coriolis vector  
        Eigen::VectorXd p_s = Jacob_dash_s.transpose() * g_t; // Operational Gravity vector
        f_star_s =  Alpha_s * x_star + niu_s + p_s; // Command forces vector for task

    }

    f_star_s = act_param * f_star_s; // Scale Singular task by activation parameter

    //std::cout << "F star Pos Non-singular: \n" << f_star_ns << std::endl;
    //std::cout << "F star Pos Singular: \n" << f_star_s << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Joint torque due to task

    Eigen::VectorXd tau_star =  Jacob_t.transpose() * (f_star_ns + f_star_s);
    //std::cout << "Tau star: \n" << tau_star << std::endl;

    // ------------------------------------------//
    // Project torque and add it to the total torque vector

    Eigen::VectorXd tau_projected = *Null_space_iter *  tau_star;
    //std::cout << "Tau projected: \n" << tau_projected << std::endl;

    *tau_total = *tau_total + tau_projected; 

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc null space

    Eigen::MatrixXd Null_space_ns =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_ns * Jacob_t; // Null space
    *Null_space_iter = *Null_space_iter * Null_space_ns.transpose();

    Eigen::MatrixXd Null_space_s =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_dummy * Jacob_t; // Null space
    *Null_space_iter = *Null_space_iter * Null_space_s.transpose();


}


////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void EffortTask::AchieveCartesianConstVel(  Eigen::Vector3d mTarget, 
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

    Eigen::MatrixXd Jacob_t = mEndEffector->getLinearJacobian(); // Jacobian
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;
    if(augmented_projections){
        Jacob_t = Jacob_t * (*Null_space_iter).transpose();
    }
    //std::cout << "Projected Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = mEndEffector->getLinearJacobianDeriv(); // Derivative of jacobian
    if(augmented_projections){
        Jacob_dot = Jacob_dot * (*Null_space_iter).transpose();
    }
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
    //std::cout << "Projected Jacobian dot: \n" << Jacob_dot << std::endl;
    //std::cout << "Q dot: \n" << q_dot << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate SVD for alpha

    singularity_thres_high_ = singularity_thres_high_pos_;
    singularity_thres_low_  = singularity_thres_low_pos_;

    Eigen::MatrixXd Alpha_t_inv = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix

    Eigen::MatrixXd Alpha_ns;
    Eigen::MatrixXd Alpha_s;
    Eigen::MatrixXd Alpha_s_dummy;
    double act_param = 0;

    calcInertiaMatrixHandling( Alpha_t_inv, svd_position, &act_param, &Alpha_ns, &Alpha_s, &Alpha_s_dummy);
    //std::cout << "Inertia Matrix: " << Alpha_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Base_Jacob_dash = M.inverse() * Jacob_t.transpose();
    Eigen::MatrixXd Jacob_dash_ns = Base_Jacob_dash * Alpha_ns; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_s = Base_Jacob_dash * Alpha_s; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_dummy = Base_Jacob_dash * Alpha_s_dummy; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    Eigen::Vector3d x_error =  mTarget - mEndEffector->getWorldTransform().translation(); // Position error
    //std::cout<< "Cartesian error: \n" << x_error << std::endl;

    // Obtain Position Gain matrices
    Eigen::MatrixXd kp = kp_cartesian_.topLeftCorner(3, 3);

    Eigen::MatrixXd damp_coeff = kd_cartesian_.topLeftCorner(3, 3);
    Eigen::MatrixXd kd = calcDampingMatrix(Eigen::MatrixXd::Identity(3,3), kp, damp_coeff); 

    Eigen::Vector3d x_dot_desired = kp*kd.inverse()*x_error;

    double scale = std::min(1.0, (max_lineal_vel_ / x_dot_desired.norm()));
    //std::cout << "Scale V: \n" << scale << std::endl;

    Eigen::Vector3d x_star =  (-1.0*kd) * (mEndEffector->getLinearVelocity() - scale*x_dot_desired); // Command force vector
    if(compensate_topdown){
        x_star = x_star - Jacob_dash_ns.transpose() * *tau_total;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational force due to task

    Eigen::VectorXd f_star_ns = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd f_star_s = Eigen::VectorXd::Zero(3);

    if(compensate_jtspace){
        f_star_ns =  Alpha_ns * ( x_star - Jacob_dot * q_dot);
        f_star_s  =  Alpha_s  * ( x_star - Jacob_dot * q_dot);
    }
    else{
        Eigen::VectorXd niu_ns = Jacob_dash_ns.transpose() * C_t  - Alpha_ns * Jacob_dot * q_dot; // Operational Coriolis vector  
        Eigen::VectorXd p_ns = Jacob_dash_ns.transpose() * g_t; // Operational Gravity vector
        f_star_ns =  Alpha_ns * x_star + niu_ns + p_ns; // Command forces vector for task

        Eigen::VectorXd niu_s = Jacob_dash_s.transpose() * C_t  - Alpha_s * Jacob_dot * q_dot; // Operational Coriolis vector  
        Eigen::VectorXd p_s = Jacob_dash_s.transpose() * g_t; // Operational Gravity vector
        f_star_s =  Alpha_s * x_star + niu_s + p_s; // Command forces vector for task

    }

    f_star_s = act_param * f_star_s; // Scale Singular task by activation parameter

    //std::cout << "F star Pos Non-singular: \n" << f_star_ns << std::endl;
    //std::cout << "F star Pos Singular: \n" << f_star_s << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Joint torque due to task

    Eigen::VectorXd tau_star =  Jacob_t.transpose() * (f_star_ns + f_star_s);
    //std::cout << "Tau star: \n" << tau_star << std::endl;

    // ------------------------------------------//
    // Project torque and add it to the total torque vector

    Eigen::VectorXd tau_projected = *Null_space_iter *  tau_star;
    //std::cout << "Tau projected: \n" << tau_projected << std::endl;

    *tau_total = *tau_total + tau_projected; 

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc null space

    Eigen::MatrixXd Null_space_ns =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_ns * Jacob_t; // Null space
    *Null_space_iter = *Null_space_iter * Null_space_ns.transpose();

    Eigen::MatrixXd Null_space_s =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_dummy * Jacob_t; // Null space
    *Null_space_iter = *Null_space_iter * Null_space_s.transpose();


}


} // end namespace
