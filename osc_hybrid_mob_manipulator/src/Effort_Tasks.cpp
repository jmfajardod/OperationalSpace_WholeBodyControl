#include <osc_hybrid_mob_manipulator/Effort_Tasks.hpp>

namespace effort_tasks {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Constructor
EffortTask::EffortTask(){

    compensate_topdown  = true;
    compensate_jtspace  = true;

    // Gain Matrices definition
    kp_cartesian_ = Eigen::MatrixXd::Identity(6, 6);
    kp_cartesian_.topLeftCorner(3, 3) = 200.0*Eigen::MatrixXd::Identity(3, 3); // Position gains
    kp_cartesian_.bottomRightCorner(3, 3) = 200.0*Eigen::MatrixXd::Identity(3, 3); // Orientation gains

    kd_cartesian_ = Eigen::MatrixXd::Identity(6, 6);
    kd_cartesian_.topLeftCorner(3, 3) = 10.0*Eigen::MatrixXd::Identity(3, 3); // Position gains
    kd_cartesian_.bottomRightCorner(3, 3) = 20.0*Eigen::MatrixXd::Identity(3, 3); // Orientation gains

    kp_joints_ = Eigen::MatrixXd::Identity(9, 9);
    kp_joints_.topLeftCorner(3, 3) = 0.1*Eigen::MatrixXd::Identity(3, 3); // Mobile base gains
    kp_joints_.bottomRightCorner(6, 6) = 0.5*Eigen::MatrixXd::Identity(6, 6); // Manipulator gains

    kd_joints_ = Eigen::MatrixXd::Identity(9, 9);
    kd_joints_.topLeftCorner(3, 3) = 2.0*Eigen::MatrixXd::Identity(3, 3); // Mobile base gains
    kd_joints_.bottomRightCorner(6, 6) = 20.0*Eigen::MatrixXd::Identity(6, 6); // Manipulator gains

    // Max vel for straight line task
    max_vel_ = 0.3; // m/s

    // Parameters for avoid joint limits task
    joint_margin_ = 0.175; // 10 deg
    eta_firas_    = 0.01;
}
////////////////////////////////////////////////////////////////////////////////
// Destructor
EffortTask::~EffortTask()
{
}

////////////////////////////////////////////////////////////////////////////////
// Function to change the Gains
/*
void EffortTask::changeGains(double kp_c, double kd_c, double kp_j, double kd_j){
    kp_cartesian_ = kp_c;
    kd_cartesian_ = kd_c;
    kp_joints_    = kp_j;
    kd_joints_    = kd_j;
}
*/

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
// Function to calculate the efforts required to Hold/Achieve a joint configuration

void EffortTask::AchieveJointConf(  Eigen::VectorXd q_desired, 
                                    Eigen::MatrixXd M, 
                                    Eigen::VectorXd C_t,
                                    Eigen::VectorXd g_t,
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector,
                                    Eigen::VectorXd *tau_total,
                                    Eigen::MatrixXd *Null_space_iter){

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate desired task

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    Eigen::VectorXd q_dot_desired = Eigen::MatrixXd::Zero(dofs,1); 
    //std::cout << "q desired: \n" << q_desired << std::endl;
    //std::cout << "q dot desired: \n" << q_dot_desired << std::endl;

    Eigen::VectorXd e = q_desired - mRobot->getPositions(); // Position error
    Eigen::VectorXd de = q_dot_desired - mRobot->getVelocities();  // Velocity error (Target velocity is zero)

    Eigen::VectorXd q_star = kd_joints_ * de ;//+ kp_joints_ * e; 

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Joint torque due to task
    
    Eigen::VectorXd tau_star = Eigen::VectorXd::Zero(dofs);
    if(compensate_jtspace){
        tau_star = M * q_star ;  // Command torques vector for task
    }
    else{
        tau_star = M * q_star + C_t + g_t;  // Command torques vector for task
    }

    //std::cout << "Tau star: \n" << tau_star << std::endl;

    // ------------------------------------------//
    // Project torque and add it to the total torque vector

    Eigen::VectorXd tau_projected = *Null_space_iter *  tau_star;

    *tau_total = *tau_total + tau_projected; 
    //std::cout << "Forces: \n" << tau_projected << std::endl;

}


////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to avoid joint limits

void EffortTask::AvoidJointLimits(Eigen::MatrixXd M, 
                                Eigen::VectorXd C_t,
                                Eigen::VectorXd g_t,
                                dart::dynamics::SkeletonPtr mRobot,
                                dart::dynamics::BodyNode* mEndEffector,
                                Eigen::VectorXd *tau_total,
                                Eigen::MatrixXd *Null_space_iter){

    // ------------------------------------------//
    // ------------------------------------------//
    // Define Limits

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

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

    Eigen::VectorXd q_current = mRobot->getPositions();

    Eigen::VectorXd f_star = Eigen::MatrixXd::Zero(dofs,1); 
    Eigen::VectorXd q_close = Eigen::MatrixXd::Zero(dofs,1); 

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

            //std::cout << "Joint " << ii << " is close LOW to limit, gamma: " << gamma_low << std::endl; 
        } 
        if(rho_up <= joint_margin_){
            gamma_up = -eta_firas_ * ( (1/rho_up) - (1/joint_margin_) ) * (1/(rho_up*rho_up));
            close2limit = true;

            //std::cout << "Joint " << ii << " is close HIGH to limit, gamma: " << gamma_up << std::endl; 
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

        // ------------------------------------------//
        // ------------------------------------------//
        // Calculate operational space matrices 

        Eigen::MatrixXd Alpha_t = Jacob_t * M.inverse() * Jacob_t.transpose(); // Symmetric Inertia Matrix
        Alpha_t = Alpha_t.inverse() ;
        //std::cout << "Alpha_t: \n" << Alpha_t << std::endl;

        Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
        //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

        // ------------------------------------------//
        // ------------------------------------------//
        // Calc Operational acceleration due to task

        if(compensate_topdown){
            gamma_vector = gamma_vector - Jacob_dash_t.transpose() * *tau_total;
        }

        // ------------------------------------------//
        // ------------------------------------------//
        // Calc Operational force due to task

        Eigen::VectorXd f_t_star = Eigen::VectorXd::Zero(3);
        if(compensate_jtspace){
            f_t_star =  Alpha_t * gamma_vector;
        }
        else{
            Eigen::VectorXd niu_t = Jacob_dash_t.transpose() * C_t ; // Operational Coriolis vector  
            //std::cout << "Niu t: \n" << niu_t << std::endl;

            Eigen::VectorXd p_t = Jacob_dash_t.transpose() * g_t; // Operational Gravity vector
            //std::cout << "P t: \n" << p_t << std::endl;

            f_t_star =  Alpha_t * gamma_vector + niu_t + p_t; // Command forces vector for task
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

}


} /* namespace */