#include <mob_manipulator_controller/Mob_Manipulator_Controller.hpp>

namespace mob_manipulator_controller {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

using namespace osc_controller;

////////////////////////////////////////////////////////////////////////////////
// Function where the stack of tasks is defined
void MobManipulatorController::StackTasks(Eigen::MatrixXd *Null_space, Eigen::VectorXd *torque_ns, int cycle){

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
    // Init definition of Stack of Tasks  
    // Uncomment or write the tasks to add to the stack
    // The task first defined has more priority

    /*****************************************************/
    // Controller using pos XY with mobile robot and Z with mobile manipulator

    //---osc_controller_.AchieveCartesianMobilRob(targetCartPos, targetCartVel, targetCartAccel, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, Null_space);
    //osc_controller_.AchieveCartesianMobilRobConstVel(targetCartPos, &min_sv_pos, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, Null_space);

    //---osc_controller_.AchievePosZ(targetCartPos, targetCartVel, targetCartAccel, &min_sv_pos, cycle, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space);
    //osc_controller_.AchievePosZConstVel(targetCartPos, &min_sv_pos, cycle, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space);

    /*****************************************************/
    // Controller using pos XYZ with manipulator

    //---osc_controller_.AchieveCartesianManipulator(targetCartPos, targetCartVel, targetCartAccel, &min_sv_pos, cycle, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space);
    //osc_controller_.AchieveCartManipulatorConstVel(targetCartPos, &min_sv_pos, cycle, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space);

    /*****************************************************/
    // Controller using pos XYZ with mobile manipulator
    
    //---osc_controller_.AchieveCartesianManipulator(targetCartPos, targetCartVel, targetCartAccel, &min_sv_pos, cycle, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space);
    //osc_controller_.AchieveCartesianConstVel(targetCartPos, &min_sv_pos, cycle, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space);

    /*****************************************************/
    // Orientation tasks with mobile manipulator

    //---osc_controller_.AchieveOrientation(targetOrientPos, targetOrientVel, targetOrientAccel, &min_sv_ori, cycle, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space);
    //osc_controller_.AchieveOrientationConstVel(targetOrientPos, &min_sv_ori, cycle, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space); 
    //---osc_controller_.OrientationImpedance(targetOrientPos, targetOrientVel, targetOrientAccel, &min_sv_ori, cycle, tau_ext, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space); 

    /*****************************************************/
    // Orientation tasks with manipulator

    //---osc_controller_.AchieveOriManipulator(targetOrientPos, targetOrientVel, targetOrientAccel, &min_sv_ori, cycle, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space);
    osc_controller_.AchieveOriManipulatorConstVel(targetOrientPos, &min_sv_ori, cycle, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space); 

    /*****************************************************/
    // Controller using pos XYZ with manipulator - To test singularities

    //---osc_controller_.AchieveCartesianManipulator(targetCartPos, targetCartVel, targetCartAccel, &min_sv_pos, cycle, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space);
    osc_controller_.AchieveCartManipulatorConstVel(targetCartPos, &min_sv_pos, cycle, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, torque_ns, Null_space);

    /*****************************************************/
    // Joint tasks
    if(cycle==1){
        osc_controller_.AchieveJointConf(q_desired, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, Null_space);
    }

}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate torque due to tasks define in the stack of tasks
void MobManipulatorController::calcTorqueDueTasks(){

    /*********************************************************************************/
    // Stack of tasks Definition

    Eigen::MatrixXd Null_space;
    Eigen::VectorXd tau_ns;
    
    //--- Number of cycles of algorithm due to singularity handling method
    int cycles_algorithm = 1;
    if(osc_controller_.singularity_handling_method == 2){
        cycles_algorithm = 2;
    }
    
    //--- Compute the constraints of the saturation
    if(osc_controller_.joint_limit_handling_method==3){
        time_actual_sjs = ros::Time::now().toSec();
        double current_sampling_time = time_actual_sjs-time_previous_sjs;
        if(current_sampling_time<=10e-6){
            current_sampling_time = 1.0/500.0;
        }

        osc_controller_.updateSJSConstraints(dart_robotSkeleton, current_sampling_time);

        time_previous_sjs = ros::Time::now().toSec();
    }

    //--- SJS variables
    bool flag_sjs = true;
    Eigen::MatrixXd Jacobian_constraints = Eigen::MatrixXd::Zero(1, robot_dofs);
    Eigen::VectorXd Desired_accel_constraints = Eigen::VectorXd::Zero(1);
    bool task_limited = false;

    //--- SJS main cycle
    while(flag_sjs){
        
        //--- Init non-singular torque
        tau_ns = Eigen::VectorXd::Zero(robot_dofs);

        //-- Loop for third singularity handling method
        for (size_t cycle = 1; cycle <= cycles_algorithm; cycle++){

            Null_space = Eigen::MatrixXd::Identity(robot_dofs,robot_dofs);
            tau_result = Eigen::VectorXd::Zero(robot_dofs);
        
            /*****************************************************/
            // Avoid Joint Limits task

            if(cycle==1){
                if(osc_controller_.joint_limit_handling_method==0){
                    osc_controller_.AvoidJointLimitsPotentials(M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
                }
                if(osc_controller_.joint_limit_handling_method==1){
                    osc_controller_.AvoidJointLimitsIntermValue(M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
                }
                if(osc_controller_.joint_limit_handling_method==3 && task_limited){
                    osc_controller_.SaturationJointSpace( Jacobian_constraints, Desired_accel_constraints, M, C_k, g_k, dart_robotSkeleton, mEndEffector_, &tau_result, &Null_space);
                }
            }

            /*****************************************************/
            // Calc torques due task in stack
            StackTasks(&Null_space, &tau_ns, cycle);

            /*****************************************************/
            // Take torque at the end of first cycle as non-singular torque
            tau_ns = tau_result;
        }

        /*****************************************************/
        // Compensation of non-linear effects in joint space

        if(osc_controller_.compensate_jtspace){
            tau_result =  tau_result + C_k + g_k;
        }

        flag_sjs = false; // Clean SJS flag

        /*****************************************************/
        // IF SJS is selected check the constraints
        if(osc_controller_.joint_limit_handling_method==3){

            Eigen::VectorXd current_joint_accel = M.inverse() * (tau_result - C_k - g_k);

            //--- Check constraints
            int critical_joint = -1;
            osc_controller_.checkSJSConstraints(current_joint_accel, mEndEffector_, &flag_sjs);

            //--- If constraints are exceeded
            if(flag_sjs){
                
                task_limited = true;
                Eigen::MatrixXd prev_jacob = Jacobian_constraints;

                //--- Update Jacobian and task vector
                osc_controller_.updateSJSConstraintTask(current_joint_accel, &flag_sjs, &Jacobian_constraints, &Desired_accel_constraints);
                
                if(prev_jacob.rows()==Jacobian_constraints.rows()){
                    if(prev_jacob.isApprox(Jacobian_constraints)){
                        ROS_INFO("Repeated saturations in SJS");
                        flag_sjs = false;
                    }
                }
            }

        }
    
    }// End SJS cycle

}

} /* namespace */