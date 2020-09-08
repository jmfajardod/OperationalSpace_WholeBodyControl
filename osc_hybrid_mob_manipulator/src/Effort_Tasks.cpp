#include <osc_hybrid_mob_manipulator/Effort_Tasks.hpp>

namespace effort_tasks {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Constructor
EffortTask::EffortTask(){
    kp_cartesian_ = 400.0;
    kd_cartesian_ = 40.0;
    kp_joints_    = 400.0;
    kd_joints_    = 40.0;

    max_vel_ = 0.7; // m/s

    joint_margin_ = 0.175; // 10 deg
    eta_firas_    = 0.01;

    singularity_thres_high_ = 0.1;
    singularity_thres_low_  = 0.05;
}
////////////////////////////////////////////////////////////////////////////////
// Destructor
EffortTask::~EffortTask()
{
}

////////////////////////////////////////////////////////////////////////////////
// Function to change the Gains
void EffortTask::changeGains(double kp_c, double kd_c, double kp_j, double kd_j){
    kp_cartesian_ = kp_c;
    kd_cartesian_ = kd_c;
    kp_joints_    = kp_j;
    kd_joints_    = kd_j;
}

////////////////////////////////////////////////////////////////////////////////
// Function to change the maximum velocity of the end effector
void EffortTask::changeMaxVel(double new_max_vel){
    max_vel_ = new_max_vel;
}

////////////////////////////////////////////////////////////////////////////////
// Function to change the joint margin for avoding joint limits
void EffortTask::changeJointMargin(double new_margin){
    joint_margin_ = new_margin;
}

////////////////////////////////////////////////////////////////////////////////
// Function to change the eta in FIRAS function for avoding joint limits
void EffortTask::change_etaFIRAS(double new_eta){
    eta_firas_ = new_eta;
}

////////////////////////////////////////////////////////////////////////////////
// Function to change the higher threshold for singularities
void EffortTask::changeHighThSing(double new_high_thr){
    singularity_thres_high_ = new_high_thr;
}

////////////////////////////////////////////////////////////////////////////////
// Function to change the lower threshold for singularities
void EffortTask::changeLowThSing(double new_low_thr){
    singularity_thres_low_ = new_low_thr;
}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void EffortTask::AchieveCartesian(  Eigen::VectorXd *tau_zero,
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

    LinearJacobian Jacob_dot    = mEndEffector->getLinearJacobianDeriv(); // Derivative of jacobian
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
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

    Eigen::Vector3d e =  mTarget - mEndEffector->getWorldTransform().translation() ; // Position error
    //std::cout<< "Error : \n" << e << std::endl;

    Eigen::Vector3d de = -mEndEffector->getLinearVelocity();  // Velocity error (Target velocity is zero)

    Eigen::Vector3d f_t_star =  kp_cartesian_ * e + kd_cartesian_ * de; // Command force vector
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
// Function to calculate the efforts required to Hold/Achieve a joint configuration

void EffortTask::AchieveJointConf(  Eigen::VectorXd *tau_zero,
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

    Eigen::VectorXd f_t_star =  kp_joints_ * e + kd_joints_ * de; // Command force vector
    //std::cout << "F star: \n" << f_t_star << std::endl;

    Eigen::VectorXd tau_star = M*f_t_star + C_t + g_t;  // Command torques vector for task
    //std::cout << "Tau star: \n" << tau_star << std::endl;

    Eigen::MatrixXd N_t =  Eigen::MatrixXd::Zero(dofs, dofs);

    *tau_result = tau_star + N_t * *tau_zero;  
    *tau_zero   = *tau_result;
    //std::cout << "Forces: \n" << mForces << std::endl;

}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a joint configuration

void EffortTask::AchieveOrientation(Eigen::VectorXd *tau_zero, 
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

    Eigen::Matrix3d R_EE_desired = R_world_EE.transpose() * rot_mat_desired;
    //std::cout << "Desired rotation matrix in EE: \n" << R_EE_desired << std::endl;

    Eigen::AngleAxisd u_ee (R_EE_desired);

    Eigen::Vector3d axis_desired = R_world_EE * u_ee.axis();
    double angle_desired = u_ee.angle();
    //std::cout << "Axis of vector u_world: \n"  << axis_desired  << std::endl;
    //std::cout << "Angle of vector u_world: \n" << angle_desired << std::endl;

    Eigen::VectorXd angular_vel =  mEndEffector->getAngularJacobian() * mRobot->getVelocities();
    //std::cout << "Angular velocity vector: \n"  << angular_vel << std::endl;

    Eigen::Vector3d f_t_star =  axis_desired * kp_cartesian_ * angle_desired - kd_cartesian_ * angular_vel; // Command force vector
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
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void EffortTask::MakeStraightLine(  Eigen::VectorXd *tau_zero,
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

    LinearJacobian Jacob_dot    = mEndEffector->getLinearJacobianDeriv(); // Derivative of jacobian
    Eigen::VectorXd q_dot = mRobot->getVelocities();            // Derivative of the joints
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

    Eigen::Vector3d x_dot_desired = (kp_cartesian_/kd_cartesian_)*x_error;

    double scale = std::min(1.0, (max_vel_ / x_dot_desired.norm()));
    //std::cout << "Scale V: \n" << scale << std::endl;

    Eigen::Vector3d f_t_star =  -kd_cartesian_ * (mEndEffector->getLinearVelocity() - scale*x_dot_desired); // Command force vector
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
// Function to calculate the efforts required to avoid joint limits

void EffortTask::AvoidJointLimits(  Eigen::VectorXd *tau_zero,
                                    Eigen::VectorXd *tau_result,
                                    Eigen::MatrixXd M, 
                                    Eigen::VectorXd C_t,
                                    Eigen::VectorXd g_t,
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector){

    // ------------------------------------------//
    // ------------------------------------------//
    // Define Limits

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    Eigen::VectorXd q_current = mRobot->getPositions();

    Eigen::VectorXd f_star = Eigen::MatrixXd::Zero(dofs,1); 
    Eigen::VectorXd q_close = Eigen::MatrixXd::Zero(dofs,1); 

    Eigen::VectorXd lower_limits = Eigen::MatrixXd::Zero(dofs,1); 
    Eigen::VectorXd upper_limits = Eigen::MatrixXd::Zero(dofs,1); 

    // Limit for joint 4
    lower_limits(3) = - M_PI;
    upper_limits(3) =   M_PI;
    // Limit for joint 5
    lower_limits(4) =  (-106.0/180.0) * M_PI;
    upper_limits(4) =  (101.0/180.0) * M_PI;
    // Limit for joint 6
    lower_limits(5) =  (-92.0/180.0) * M_PI;
    upper_limits(5) =  (101.0/180.0) * M_PI;
    // Limit for joint 7
    lower_limits(6) = - M_PI;
    upper_limits(6) =   M_PI;
    // Limit for joint 8
    lower_limits(7) =  (-130.0/180.0) * M_PI;
    upper_limits(7) =  (107.0/180.0) * M_PI;
    // Limit for joint 9
    lower_limits(8) = - M_PI;
    upper_limits(8) =   M_PI;

    //std::cout << "Lower limits : \n" << lower_limits << std::endl;
    //std::cout << "Upper limits : \n" << upper_limits << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Check all joints of the manipulator to see if they are close to the limit

    for (size_t ii = 3; ii < dofs; ii++)
    {
        double rho_low = q_current(ii) - lower_limits(ii);
        double rho_up  = upper_limits(ii) - q_current(ii);

        bool close2limit = false;
        
        double gamma_low = 0.0;
        double gamma_up  = 0.0;

        if(rho_low <= joint_margin_){
            gamma_low = eta_firas_ * ( (1/rho_low) - (1/joint_margin_) ) * (1/(rho_low*rho_low));
            close2limit = true;
            std::cout << "Joint " << ii << " is close LOW to limit, gamma: " << gamma_low << std::endl; 
        } 
        if(rho_up <= joint_margin_){
            gamma_up = -eta_firas_ * ( (1/rho_up) - (1/joint_margin_) ) * (1/(rho_up*rho_up));
            close2limit = true;
            std::cout << "Joint " << ii << " is close HIGH to limit, gamma: " << gamma_up << std::endl; 
        } 

        f_star(ii) = gamma_low + gamma_up;
        if(close2limit)   q_close(ii) = 1.0;
    }

    //std::cout << "F star: \n" << f_star << std::endl;
    
    // ------------------------------------------//
    // ------------------------------------------//
    // If a least one joint is close to the limits 
    //create the jacobian and calculate the torques
    
    if(q_close.sum()>0.0){
        Eigen::MatrixXd Jacob_t = Eigen::MatrixXd::Zero(int(q_close.sum()),dofs);
        Eigen::VectorXd gamma_vector = Eigen::MatrixXd::Zero(int(q_close.sum()),1);
        int aux_counter = 0;
        for (size_t ii = 0; ii < dofs; ii++)
        {
            if(q_close(ii) == 1.0){
                Jacob_t(aux_counter,ii) = 1.0;
                gamma_vector(aux_counter) = f_star(ii);
                aux_counter++;
            }
        }
        //std::cout << "Gamma vector: \n" << gamma_vector << std::endl;
        //std::cout << "Jacobian: \n" << Jacob_t << std::endl;

        Eigen::MatrixXd Alpha_t = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix
        Alpha_t = Alpha_t.inverse() ;
        //std::cout << "Alpha_t: \n" << Alpha_t << std::endl;

        Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
        //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

        Eigen::VectorXd niu_t = Jacob_dash_t.transpose() * C_t ; // Operational Coriolis vector  
        //std::cout << "Niu t: \n" << niu_t << std::endl;

        Eigen::VectorXd p_t = Jacob_dash_t.transpose() * g_t; // Operational Gravity vector
        //std::cout << "P t: \n" << p_t << std::endl;

        Eigen::VectorXd tau_star =  Alpha_t * gamma_vector + niu_t + p_t; // Command torques vector for task
        tau_star =  Jacob_t.transpose() * tau_star;
        //std::cout << "Tau star: \n" << tau_star << std::endl;
        
        Eigen::MatrixXd N_t =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_t.transpose() * Jacob_dash_t.transpose(); // Null space
        //std::cout << "N_t: \n" << N_t << std::endl;

        *tau_result = tau_star + N_t * *tau_zero;  
        *tau_zero   = *tau_result;
        //std::cout << "Forces: \n" << tau_result << std::endl;
    }

}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position
// while avoiding singularities

void EffortTask::CartesianAvoidSing(Eigen::VectorXd *tau_zero,
                                    Eigen::VectorXd *tau_result,
                                    Eigen::Vector3d mTarget, 
                                    Eigen::MatrixXd M, 
                                    Eigen::VectorXd C_t,
                                    Eigen::VectorXd g_t,
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector){

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate alpha

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    LinearJacobian Jacob_t = mEndEffector->getLinearJacobian(); // Jacobian
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Alpha_t = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix
    Eigen::MatrixXd Alpha_inv = Alpha_t; 
    Alpha_t = Alpha_t.inverse() ; 
    //std::cout << "Alpha_t: \n" << Alpha_t << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate SVD for alpha

    //Eigen::JacobiSVD<Eigen::MatrixXd> svd(Alpha_t, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Alpha_inv, Eigen::ComputeFullU | Eigen::ComputeFullV);

    std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
    std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd.matrixU() << std::endl;
    //std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << svd.matrixV() << std::endl;

    Eigen::VectorXd svd_values = svd.singularValues();
    Eigen::MatrixXd Full_U = svd.matrixU();

    // ------------------------------------------//
    // ------------------------------------------//
    // Compare SVD to threshold

    Eigen::VectorXd small_svd = Eigen::MatrixXd::Zero(3,1);

    for (size_t ii = 0; ii < 3; ii++){
        if(abs(svd_values(ii))<singularity_thres_high_)   small_svd(ii) = 1.0;
    }

    // Variable to check if the manipulator is close to a singularity
    bool close_singularity = false;

    // Non-singular U
    Eigen::MatrixXd U_ns = Eigen::MatrixXd::Zero(3,3 - int(small_svd.sum()));
    Eigen::MatrixXd svd_values_ns = Eigen::MatrixXd::Zero(3 - int(small_svd.sum()),3 - int(small_svd.sum()));

    // Singular U
    Eigen::MatrixXd U_s;
    if(int(small_svd.sum()) > 0.0){
        U_s = Eigen::MatrixXd::Zero(3,int(small_svd.sum()));
        close_singularity = true;
    }    
    else{
        U_s = Eigen::MatrixXd::Zero(3,1);
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Fill U_ns , U_s and SVD_ns

    double aux_counter_s = 0;
    double aux_counter_ns = 0;
    for (size_t ii = 0; ii < 3; ii++){

        // If the SVD is low enough append column to U_s
        if(small_svd(ii) == 1.0){
            U_s(0,aux_counter_s) = Full_U(0,ii);
            U_s(1,aux_counter_s) = Full_U(1,ii);
            U_s(2,aux_counter_s) = Full_U(2,ii);
            aux_counter_s++;
        }
        // Else append the column to U_ns and the svd_value to the svd_values_ns matrix
        else{
            U_ns(0,aux_counter_ns) = Full_U(0,ii);
            U_ns(1,aux_counter_ns) = Full_U(1,ii);
            U_ns(2,aux_counter_ns) = Full_U(2,ii);

            svd_values_ns(aux_counter_ns,aux_counter_ns) = svd_values(ii);
            aux_counter_ns++;
        }
    }
    
    std::cout << "U_singular: \n" << U_s << std::endl;
    std::cout << "U_non-singular: \n" << U_ns << std::endl;
    std::cout << "Sigma_non-singular: \n" << svd_values_ns << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Compute the command force vector for the task Hold/Achieve a cartesian position

    Eigen::Vector3d e = mTarget - mEndEffector->getWorldTransform().translation(); // Position error

    Eigen::Vector3d de = -mEndEffector->getLinearVelocity();  // Velocity error (Target velocity is zero)

    Eigen::Vector3d f_t_star =  kp_cartesian_ * e + kd_cartesian_ * de; // Command force vector
    std::cout << "F star: \n" << f_t_star << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Project the command vector to the singular and non singular directions

    Eigen::VectorXd f_ns = U_ns.transpose() * f_t_star;
    Eigen::VectorXd f_s  = U_s.transpose()  * f_t_star;

    Eigen::MatrixXd Jacob_ns =  U_ns.transpose() * Jacob_t;
    Eigen::MatrixXd Jacob_s  =  U_s.transpose()  * Jacob_t;

    std::cout << "F_singular: \n" << f_s << std::endl;
    std::cout << "F_non-singular: \n" << f_ns << std::endl;
    std::cout << "Jacobian_singular: \n" << Jacob_s << std::endl;
    std::cout << "Jacobian_non-singular: \n" << Jacob_ns << std::endl;
    
    // ------------------------------------------//
    // ------------------------------------------//
    // Find minimum singular value 

    double min_svd = 1.0e3;
    for (size_t ii = 0; ii < 3; ii++){
        if(abs(svd_values(ii))<min_svd)   min_svd = abs(svd_values(ii));
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Find Alpha non-singular
    Eigen::MatrixXd Alpha_ns;

    if (abs(min_svd)<singularity_thres_low_){
        Alpha_ns = U_ns * svd_values_ns * U_ns.transpose() ;
    }
    else{
        Alpha_ns = Jacob_ns * M.inverse() * Jacob_ns.transpose(); 
        Alpha_ns = Alpha_ns.inverse() ;
    }
    std::cout << "Alpha_non-singular: \n" << Alpha_ns << std::endl;


    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate the activation parameter h
    
    double h = 0.0;
    if (abs(min_svd)>=singularity_thres_high_){
        h = 1.0;
    }
    else if (abs(min_svd)<singularity_thres_low_){
        h = 0.0;
    }
    else
    {
        h = 0.5 + 0.5*sin( (M_PI/(singularity_thres_high_-singularity_thres_low_))*(abs(min_svd)-singularity_thres_low_) - M_PI_2);
    }
    std::cout << "Activation parameters: \n" << h << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate the corrected forces

    f_ns = f_ns;
    f_s  = h*f_s + (1-h) * Jacob_s * M.inverse() * Jacob_ns.transpose() * h * Alpha_ns * f_ns;

    std::cout << "Corrected F_non-singular: \n" << f_ns << std::endl;
    std::cout << "Corrected F_singular: \n" << f_s << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Create the complete force vector

    Eigen::VectorXd f_complete(3);
    if(close_singularity)   f_complete << f_ns, f_s;
    else                    f_complete << f_ns;

    // ------------------------------------------//
    // ------------------------------------------//
    // Decouple forces and map to joint space

    Eigen::MatrixXd Jacob_complete = Eigen::MatrixXd::Zero(3,dofs);
    if(close_singularity)   Jacob_complete << Jacob_ns, Jacob_s;
    else                    Jacob_complete << Jacob_ns;

    //std::cout << "Complete Jacobian: \n" << Jacob_complete << std::endl;

    Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_complete.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    Eigen::VectorXd niu_t = Jacob_dash_t.transpose() * C_t ; // Operational Coriolis vector  
    //std::cout << "Niu t: \n" << niu_t << std::endl;

    Eigen::VectorXd p_t = Jacob_dash_t.transpose() * g_t; // Operational Gravity vector
    //std::cout << "P t: \n" << p_t << std::endl;

    Eigen::VectorXd niu_ns = U_ns.transpose() * niu_t;
    Eigen::VectorXd niu_s  = U_s.transpose()  * niu_t;
    Eigen::VectorXd niu_complete(3);
    if(close_singularity)   niu_complete << niu_ns, niu_s;
    else                    niu_complete << niu_ns;

    //std::cout << "Complete Niu vector: \n" << niu_complete << std::endl;

    Eigen::VectorXd p_ns = U_ns.transpose() * p_t;
    Eigen::VectorXd p_s  = U_s.transpose()  * p_t;
    Eigen::VectorXd p_complete(3);
    if(close_singularity)   p_complete << p_ns, p_s;
    else                    p_complete << p_ns;

    //std::cout << "Complete P vector: \n" << p_complete << std::endl;
    

    Eigen::VectorXd tau_star =  Alpha_t * f_complete + niu_complete + p_complete; // Command torques vector for task
    tau_star =  Jacob_complete.transpose() * tau_star;
    //std::cout << "Tau star: \n" << tau_star << std::endl;
    
    Eigen::MatrixXd N_t =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_complete.transpose() * Jacob_dash_t.transpose(); // Null space
    //std::cout << "N_t: \n" << N_t << std::endl;

    *tau_result = tau_star + N_t * *tau_zero;  
    *tau_zero   = *tau_result;
    //std::cout << "Forces: \n" << tau_result << std::endl;

}

} /* namespace */