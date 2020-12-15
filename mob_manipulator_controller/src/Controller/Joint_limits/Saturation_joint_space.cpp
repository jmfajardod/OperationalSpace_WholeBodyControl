#include <mob_manipulator_controller/OSC_Controller.hpp>

namespace effort_tasks {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Function to update SJS constraints
void EffortTask::updateSJSConstraints(dart::dynamics::SkeletonPtr mRobot, double sampling_time){

    Eigen::VectorXd current_pos = mRobot->getPositions();
    Eigen::VectorXd current_vel = mRobot->getVelocities();

    //std::cout << "Current joint positions:\n" << current_pos.transpose() << std::endl;
    //std::cout << "Current joint velocities:\n" << current_vel.transpose() << std::endl; 

    for(size_t yy = 0; yy<9 ; yy++){
        if(yy<2){
            Max_joint_accel(yy) =  10.0; // m/s^2
            Min_joint_accel(yy) = -10.0; // m/s^2
            Max_joint_vel(yy)   =  0.2; // m/s
            Min_joint_vel(yy)   = -0.2; // m/s
            Max_joint_pos(yy)   =  10e10;
            Min_joint_pos(yy)   = -10e10;
        }
        if(yy==2){
            Max_joint_accel(yy) =  M_PI*2; // rad/s^2
            Min_joint_accel(yy) = -M_PI*2; // rad/s^2
            Max_joint_vel(yy)   =  3.0; // rad/s
            Min_joint_vel(yy)   = -3.0; // rad/s
            Max_joint_pos(yy)   =  M_PI_2-0.1; // rad
            Min_joint_pos(yy)   = -M_PI_2+0.1; // rad
        }
        if(yy>2){
            Max_joint_accel(yy) =  5*M_PI; // rad/s^2 default:  2*M_PI
            Min_joint_accel(yy) = -5*M_PI; // rad/s^2 default: -2*M_PI
            Max_joint_vel(yy)   =  2.0; // rad/s      default:  3.0
            Min_joint_vel(yy)   = -2.0; // rad/s      default: -3.0
            Max_joint_pos(yy)   =  Upper_limits(yy)-joint_margin_(yy); // rad
            Min_joint_pos(yy)   =  Lower_limits(yy)+joint_margin_(yy); // rad
        }
    }
    double sampling_T = 0.01; // Sampling time for velocity predicition

    Eigen::VectorXd Joint_accel_max = Eigen::VectorXd::Zero(9);
    Eigen::VectorXd Joint_accel_min = Eigen::VectorXd::Zero(9);
    Eigen::VectorXd vector_ones = Eigen::VectorXd::Constant(9,1);

    Eigen::VectorXd max_vel = ( 1/sampling_T)*(Max_joint_vel-current_vel);
    Eigen::VectorXd min_vel = ( 1/sampling_T)*(Min_joint_vel-current_vel);

    sampling_T = 0.1; // Sampling time for position predicition

    Eigen::VectorXd max_pos_joint = (2/(sampling_T*sampling_T))*(Max_joint_pos-sampling_T*current_vel-current_pos);
    Eigen::VectorXd min_pos_joint = (2/(sampling_T*sampling_T))*(Min_joint_pos-sampling_T*current_vel-current_pos);

    for (size_t yy = 0; yy < 9; yy++){
        Joint_accel_max(yy) = std::min( max_pos_joint(yy) , std::min( max_vel(yy) , Max_joint_accel(yy) ) );
        Joint_accel_min(yy) = std::max( min_pos_joint(yy) , std::max( min_vel(yy) , Min_joint_accel(yy) ) );

        /*
        if(current_pos(yy)>Max_joint_pos(yy)){
            Joint_accel_max(yy) = (2/sampling_T)*(-current_vel(yy));
        }
        if(current_pos(yy)<Min_joint_pos(yy)){
            Joint_accel_min(yy) = (2/sampling_T)*(-current_vel(yy));
        }
        */

        if(Joint_accel_max(yy)<Joint_accel_min(yy)){
            double aux_value = Joint_accel_min(yy);
            Joint_accel_min(yy) = Joint_accel_max(yy);
            Joint_accel_max(yy) = aux_value;
        }

        //if(Joint_accel_max(yy)<-max_joint_accel) Joint_accel_max(yy)=-max_joint_accel;
        //if(Joint_accel_min(yy)> max_joint_accel) Joint_accel_min(yy)= max_joint_accel;
    }        

    //std::cout << "Max limits:\n" << Joint_accel_max.transpose() << std::endl;
    //std::cout << "Min limits:\n" << Joint_accel_min.transpose() << std::endl;

    Max_constraint_accel = Joint_accel_max;
    Min_constraint_accel = Joint_accel_min;

}

////////////////////////////////////////////////////////////////////////////////
// Function to check SJS constraints

void EffortTask::checkSJSConstraints(Eigen::VectorXd Joint_Acceleration,
                                    dart::dynamics::BodyNode* mEndEffector,
                                    bool* flag_sjs){

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    double max_diff = 0;
    double tol_sjs = 0.1;
    bool   flag_limits = false;

    for (size_t yy = 3; yy < dofs; yy++){

        double joint_accel_sel = Joint_Acceleration(yy);
        double max_value_sel   = Max_constraint_accel(yy) + tol_sjs;
        double min_value_sel   = Min_constraint_accel(yy) - tol_sjs;

        if(joint_accel_sel>max_value_sel){ 
            //std::cout << "Accel " << joint_accel_sel << " more than Max "<< max_value_sel << std::endl;
            flag_limits=true;
            break;
        }
        if(joint_accel_sel<min_value_sel){ 
            //std::cout << "Accel " << joint_accel_sel << " less than Min "<< min_value_sel << std::endl;
            flag_limits=true;
            break;
        }
    }
    if(flag_limits==true){
        //std::cout << "MAX Accel " << Max_constraint_accel.transpose()  << std::endl;
        //std::cout << "MIN Accel " << Min_constraint_accel.transpose()  << std::endl;
    }

    *flag_sjs = flag_limits;
}

////////////////////////////////////////////////////////////////////////////////
// Function to update SJS Jacobian and task vector

void EffortTask::updateSJSConstraintTask(Eigen::VectorXd Current_joint_accel,
                                        bool* flag_sjs,
                                        Eigen::MatrixXd *Jacobian_constr,
                                        Eigen::VectorXd *Constr_Task_Acceleration){

    Eigen::VectorXd current_joint_accel = Current_joint_accel;
    Eigen::VectorXd Desired_accel_constraints = *Constr_Task_Acceleration;
    Eigen::MatrixXd Jacobian_constraints = *Jacobian_constr;
    bool limits = false;

    for(size_t ii = 3; ii<9;ii++){
        
        double desired_accel = 0;
        bool   limit_exceeded = false;

        double joint_accel_sel = current_joint_accel(ii);
        double max_joint_accel_sel = Max_constraint_accel(ii);
        double min_joint_accel_sel = Min_constraint_accel(ii);

        //std::cout << "Joint " << ii << std::endl;
        //std::cout << "Current accel of joint: " << joint_accel_sel << std::endl;
        //std::cout << "Max accel of joint: " << max_joint_accel_sel << std::endl;
        //std::cout << "Min accel of joint: " << min_joint_accel_sel << std::endl;

        if(joint_accel_sel>max_joint_accel_sel){
            desired_accel = max_joint_accel_sel;
            limit_exceeded = true;
        }
        else if(joint_accel_sel<min_joint_accel_sel){
            desired_accel = min_joint_accel_sel;
            limit_exceeded = true;
        }

        if(limit_exceeded){

            if(Jacobian_constraints.rows()==1 && Jacobian_constraints.row(0).sum()==0.0){
                Jacobian_constraints(0, ii) = 1.0;
                Desired_accel_constraints(0) = desired_accel;
            }
            else{
                if(Jacobian_constraints.col(ii).sum()<1.0){
                    Jacobian_constraints.conservativeResize(Jacobian_constraints.rows()+1, Eigen::NoChange);
                    Jacobian_constraints.row(Jacobian_constraints.rows()-1) = Eigen::VectorXd::Zero(9);
                    Jacobian_constraints(Jacobian_constraints.rows()-1,ii) = 1.0;

                    Desired_accel_constraints.conservativeResize(Desired_accel_constraints.rows()+1);
                    Desired_accel_constraints(Desired_accel_constraints.rows()-1) = desired_accel;
                }/*
                else{
                    std::cout << "------ LIMIT REPEATED------" << std::endl;
                    *flag_sjs = false;
                }*/
            }
        }
    }

    //std::cout << "Jacobian of constraints\n" << Jacobian_constraints << std::endl;
    //std::cout << "Desired accel vector\n"    << Desired_accel_constraints.transpose() << std::endl;

    *Constr_Task_Acceleration = Desired_accel_constraints;
    *Jacobian_constr = Jacobian_constraints;

}

////////////////////////////////////////////////////////////////////////////////
// Function to compute SJS joint task and update Null space

void EffortTask::SaturationJointSpace(Eigen::MatrixXd Jacobian,
                                    Eigen::VectorXd desired_accel,
                                    Eigen::MatrixXd M, 
                                    Eigen::VectorXd C_t,
                                    Eigen::VectorXd g_t,
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector,
                                    Eigen::VectorXd *tau_total,
                                    Eigen::MatrixXd *Null_space_iter){

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    Eigen::MatrixXd Alpha_limit_inv = Jacobian * M.inverse() * Jacobian.transpose(); // Symmetric Inertia Matrix
    Eigen::MatrixXd Alpha_limit = Alpha_limit_inv.inverse();

    Eigen::MatrixXd Jacob_dash_limit = M.inverse() * Jacobian.transpose() * Alpha_limit;
    Eigen::MatrixXd Null_space_limit = Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_limit * Jacobian;
    
    //std::cout << "Desired accel vector in calc\n"    << desired_accel.transpose() << std::endl;

    Eigen::VectorXd Force_limit_task;
    if(compensate_jtspace){
        Force_limit_task =  Alpha_limit * desired_accel;
    }
    else{
        Eigen::VectorXd niu_limit = Jacob_dash_limit.transpose() * C_t; // Operational Coriolis vector  
        Eigen::VectorXd p_limit   = Jacob_dash_limit.transpose() * g_t; // Operational Gravity vector
        Force_limit_task =  Alpha_limit * desired_accel + niu_limit + p_limit; // Command forces vector for task
    }
    
    //std::cout << "Force_limit_task \n" << Force_limit_task.transpose() << std::endl;

    Eigen::VectorXd Torque_limit_task = Jacobian.transpose() * Force_limit_task;
    Eigen::VectorXd Torque_limit_task_projected = Torque_limit_task; 

    *tau_total = *tau_total + Torque_limit_task_projected;; // Update torque
    //std::cout << "Torque dash \n" << (*tau_total).transpose() << std::endl;

    *Null_space_iter = Null_space_limit.transpose(); // Update null space
    //std::cout << "Null space\n" << (*Null_space_iter).transpose() << std::endl;
}

} /* namespace */