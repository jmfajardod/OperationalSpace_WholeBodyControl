#include <osc_controller/OSC_Controller.hpp>

namespace osc_controller {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to avoid joint limits

void OSC_Controller::AvoidJointLimitsIntermValue(Eigen::MatrixXd M, 
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
    Eigen::VectorXd act_param_aux = Eigen::VectorXd::Zero(dofs); 
    Eigen::VectorXd joint_force = Eigen::VectorXd::Zero(dofs); 
    Eigen::VectorXd q_close = Eigen::VectorXd::Zero(dofs); 

    // Check all joints except the first three (Only manipulator)
    for (size_t ii = 3; ii < dofs; ii++)
    {
        double margin_up  = Upper_limits(ii) - joint_margin_(ii);
        double margin_low = Lower_limits(ii) + joint_margin_(ii);
        
        double thres_up  = margin_up  - joint_limit_buffer(ii);
        double thres_low = margin_low + joint_limit_buffer(ii);

        bool close2limit = false;

        if( q_current(ii) >= margin_up ){
            act_param_aux(ii) = 1.0;
            joint_force(ii) = gain_limit_avoidance*(thres_up - q_current(ii));
            close2limit = true;
            //std::cout << "Joint " << ii << " too HIGH" << std::endl;
        }
        else if (  q_current(ii) > thres_up ){
            act_param_aux(ii) = 0.5 + 0.5*sin( (M_PI/joint_limit_buffer(ii))*( q_current(ii) - thres_up ) - M_PI_2 );
            joint_force(ii) = gain_limit_avoidance*(thres_up - q_current(ii));
            close2limit = true;
            //std::cout << "Joint " << ii << " in HIGH buffer region" << std::endl;
        }

        if(q_current(ii) <= margin_low){
            act_param_aux(ii) = 1.0;
            joint_force(ii) = gain_limit_avoidance*(thres_low - q_current(ii));
            close2limit = true;
            //std::cout << "Joint " << ii << " too LOW" << std::endl;
        } 
        else if(q_current(ii) < thres_low){
            act_param_aux(ii) = 0.5 + 0.5*sin( (M_PI/joint_limit_buffer(ii))*( q_current(ii) - thres_low ) + M_PI_2 );
            joint_force(ii) = gain_limit_avoidance*(thres_low - q_current(ii));
            close2limit = true;
            //std::cout << "Joint " << ii << " in LOW buffer region" << std::endl;
        } 

        if(close2limit)   q_close(ii) = 1.0;
    }
    
    // ------------------------------------------//
    // ------------------------------------------//
    // If a least one joint is close to the limits 
    //create the jacobian and calculate the torques
    
    if(q_close.sum()>0.0){
        
        //std::cout << "Limits: \n" << q_close.transpose() << std::endl;

        Eigen::MatrixXd Jacob_t = Eigen::MatrixXd::Zero(int(q_close.sum()),dofs);
        Eigen::VectorXd act_param = Eigen::VectorXd::Zero(int(q_close.sum()));
        Eigen::VectorXd x_star = Eigen::VectorXd::Zero(int(q_close.sum()));
        
        // Create Jacobian of joint limit task and form force vector from individual forces
        int aux_counter = 0; // Counter for rows of Jacobian
        for (size_t ii = 0; ii < dofs; ii++)
        {
            if(q_close(ii) == 1.0){
                Jacob_t(aux_counter,ii) = 1.0;
                act_param(aux_counter) = act_param_aux(ii);
                x_star(aux_counter) = joint_force(ii); 
                aux_counter++;
            }
        }
        //std::cout << "Desired accel vector: \n" << x_star << std::endl;
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
            x_star = x_star - Jacob_dash_t.transpose() * *tau_total;
        }

        // ------------------------------------------//
        // ------------------------------------------//
        // Calc Operational force due to task

        Eigen::VectorXd f_t_star = Eigen::VectorXd::Zero(int(q_close.sum()));
        if(compensate_jtspace){
            f_t_star =  Alpha_t * x_star; // The Jacobian is assumed constant and its derivative is zero
        } 
        else{
            Eigen::VectorXd niu_t = Jacob_dash_t.transpose() * C_t ; // Operational Coriolis vector  
            //std::cout << "Niu t: \n" << niu_t << std::endl;

            Eigen::VectorXd p_t = Jacob_dash_t.transpose() * g_t; // Operational Gravity vector
            //std::cout << "P t: \n" << p_t << std::endl;

            f_t_star =  Alpha_t * x_star + niu_t + p_t; // Command forces vector for task
        }

        // Scale task
        for (size_t ii = 0; ii < int(q_close.sum()); ii++){
            f_t_star(ii) = act_param(ii)*f_t_star(ii);
        }

        //std::cout << "After scale F star: \n" << f_t_star << std::endl;

        // ------------------------------------------//
        // ------------------------------------------//
        // Calc Joint torque due to task

        Eigen::VectorXd tau_star =  Jacob_t.transpose() * f_t_star;
        //std::cout << "Tau star: \n" << tau_star << std::endl;

        // ------------------------------------------//
        // Project torque and add it to the total torque vector

        Eigen::VectorXd tau_projected = *Null_space_iter *  tau_star;
        *tau_total = *tau_total + tau_projected; 

        //std::cout << "Tau projected: \n" << tau_projected << std::endl;

        // ------------------------------------------//
        // ------------------------------------------//
        // Calc null space

        if(interm_alg_update_null==3){

            act_param = scale_null_space*act_param; 

            //--//
            //Scale Jacobian to scale Null space 
            Eigen::MatrixXd Jacobian_scaled = Jacob_t.array().colwise() * act_param.array();
            Eigen::MatrixXd Null_space_task =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_t * Jacobian_scaled; // Null space
            //Eigen::MatrixXd Null_space_task_st =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_t * Jacob_t;
            //std::cout << "Standard null space: \n" << Null_space_task_st << std::endl;
            //std::cout << "New null space: \n" << Null_space_task << std::endl; 

            *Null_space_iter = *Null_space_iter * Null_space_task.transpose();

        }
        else if(interm_alg_update_null==2){

            act_param_aux = scale_null_space*act_param_aux;            

            //--//
            // Create diagonal matrix based on activation parameters
            Eigen::MatrixXd occ_dofs = Eigen::MatrixXd::Identity(dofs, dofs).array().rowwise()  * act_param_aux.transpose().array();
            Eigen::MatrixXd Null_space_task =  Eigen::MatrixXd::Identity(dofs, dofs) - occ_dofs; // Null space
            //std::cout << "N_t: \n" << Null_space_task << std::endl;

            *Null_space_iter = *Null_space_iter * Null_space_task.transpose();
        }
        else if (interm_alg_update_null==1){
            Eigen::MatrixXd Null_space_task =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_t * Jacob_t;
            *Null_space_iter = *Null_space_iter * Null_space_task.transpose();
        }
    }

}


} /* namespace */