#include <osc_hybrid_mob_manipulator/Effort_Tasks.hpp>

namespace effort_tasks {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to avoid joint limits

void EffortTask::AvoidJointLimitsPotentials(Eigen::MatrixXd M, 
                                            Eigen::VectorXd C_t,
                                            Eigen::VectorXd g_t,
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector,
                                            Eigen::VectorXd *tau_total,
                                            Eigen::MatrixXd *Null_space_iter){

    // ------------------------------------------//
    // ------------------------------------------//
    // Get number of DOFS

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    //std::cout << "Lower limits : \n" << Lower_limits << std::endl;
    //std::cout << "Upper limits : \n" << Upper_limits << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Check all joints of the manipulator to see if they are close to the limit

    Eigen::VectorXd q_current = mRobot->getPositions();

    Eigen::VectorXd f_star = Eigen::VectorXd::Zero(dofs); 
    Eigen::VectorXd q_close = Eigen::VectorXd::Zero(dofs); 

    // Check all joints except the first three (Only manipulator)
    for (size_t ii = 3; ii < dofs; ii++)
    {
        double rho_low = q_current(ii) - Lower_limits(ii);
        double rho_up  = Upper_limits(ii) - q_current(ii);

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

    
    // ------------------------------------------//
    // ------------------------------------------//
    // If a least one joint is close to the limits 
    //create the jacobian and calculate the torques
    
    if(q_close.sum()>0.0){
        
        //std::cout << "Limits: \n" << q_close.transpose() << std::endl;

        Eigen::MatrixXd Jacob_t = Eigen::MatrixXd::Zero(int(q_close.sum()),dofs);
        Eigen::VectorXd gamma_vector = Eigen::VectorXd::Zero(int(q_close.sum()));
        
        // Create Jacobian of joint limit task and form force vector from individual forces
        int aux_counter = 0; // Counter for rows of Jacobian
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
        Alpha_t = Alpha_t.inverse() ; // This matrix does not suffer from singularities and can be inverted
        //std::cout << "Alpha_t: \n" << Alpha_t << std::endl;

        Eigen::MatrixXd Jacob_dash_t = M.inverse() * Jacob_t.transpose() * Alpha_t; // Dynamically consistent inverse jacobian
        //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

        // ------------------------------------------//
        // ------------------------------------------//
        // Compensate top-down effects

        if(compensate_topdown){
            gamma_vector = gamma_vector - Jacob_dash_t.transpose() * *tau_total;
        }

        // ------------------------------------------//
        // ------------------------------------------//
        // Calc Operational force due to task

        Eigen::VectorXd f_t_star = Eigen::VectorXd::Zero(int(q_close.sum()));
        if(compensate_jtspace){
            f_t_star =  Alpha_t * gamma_vector; // The Jacobian is assumed constant and its derivative is zero
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

        //Eigen::VectorXd tau_projected = *Null_space_iter *  tau_star;
        //*tau_total = *tau_total + tau_projected; 

        *tau_total = *tau_total + tau_star;

        // ------------------------------------------//
        // ------------------------------------------//
        // Calc null space

        Eigen::MatrixXd Null_space_task =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_t * Jacob_t; // Null space
        //std::cout << "N_t: \n" << Null_space_task << std::endl;

        // *Null_space_iter = *Null_space_iter * Null_space_task.transpose();

    }

}


} /* namespace */