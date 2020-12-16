/*******************************************************************************
* Copyright 2020 Jose Manuel Fajardo
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <osc_controller/OSC_Controller.hpp>

namespace osc_controller {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void OSC_Controller::AchieveCartesianManipulator(Eigen::Vector3d mTargetPos, 
                                            Eigen::Vector3d mTargetVel,
                                            Eigen::Vector3d mTargetAccel, 
                                            double *svd_position,
                                            int cycle,
                                            Eigen::MatrixXd M, 
                                            Eigen::VectorXd C_t,
                                            Eigen::VectorXd g_t,
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector,
                                            Eigen::VectorXd *tau_total,
                                            Eigen::VectorXd *tau_ns,
                                            Eigen::MatrixXd *Null_space_iter){

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate operational space matrices 

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    Eigen::MatrixXd Jacob_t = mEndEffector->getLinearJacobian(); // Jacobian
    Jacob_t.topLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    if(augmented_projections){
        Jacob_t = Jacob_t * (*Null_space_iter).transpose();
    }
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = mEndEffector->getLinearJacobianDeriv(); // Derivative of jacobian
    Jacob_dot.topLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
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

    Eigen::MatrixXd Alpha_task = Alpha_ns;
    if(cycle==2){
        Alpha_task = Alpha_s;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Base_Jacob_dash = M.inverse() * Jacob_t.transpose();
    Eigen::MatrixXd Jacob_dash_ns = Base_Jacob_dash * Alpha_ns; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_s = Base_Jacob_dash * Alpha_s; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_dummy = Base_Jacob_dash * Alpha_s_dummy; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    Eigen::MatrixXd Jacob_dash_task = Jacob_dash_ns;
    if(cycle==2){
        Jacob_dash_task = Jacob_dash_s;
    }

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

    Eigen::VectorXd f_star = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd f_star_s = Eigen::VectorXd::Zero(3);

    if(compensate_jtspace){
        f_star =  Alpha_task * ( x_star - Jacob_dot * q_dot);

        // If the method is without the torque projections
        if(singularity_handling_method==1){
            f_star_s  =  Alpha_s  * ( x_star - Jacob_dot * q_dot);
        }
    }
    else{
        Eigen::VectorXd niu = Jacob_dash_task.transpose() * C_t  - Alpha_task * Jacob_dot * q_dot; // Operational Coriolis vector  
        Eigen::VectorXd p   = Jacob_dash_task.transpose() * g_t; // Operational Gravity vector
        f_star =  Alpha_task * x_star + niu + p; // Command forces vector for task

        // If the method is without the torque projections
        if(singularity_handling_method==1){
            Eigen::VectorXd niu_s = Jacob_dash_s.transpose() * C_t  - Alpha_s * Jacob_dot * q_dot; // Operational Coriolis vector  
            Eigen::VectorXd p_s = Jacob_dash_s.transpose() * g_t; // Operational Gravity vector
            f_star_s =  Alpha_s * x_star + niu_s + p_s; // Command forces vector for task
        }
    }
    
    // If the method is with the projections of non-singular tasks
    if(cycle==2){
        f_star = act_param * f_star + (1-act_param) * (Jacob_dash_task.transpose() * *tau_ns); // Scale Singular task by activation parameter
    }

    // If the method is without the torque projections
    if(singularity_handling_method==1){
        f_star_s = act_param * f_star_s; // Scale Singular task by activation parameter
        f_star = f_star + f_star_s; // Add non-singular and singular components
    }

    //std::cout << "F star:  \n" << f_star << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Joint torque due to task

    Eigen::VectorXd tau_star =  Jacob_t.transpose() * f_star;
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

    if(singularity_handling_method != 0){
        Eigen::MatrixXd Null_space_s =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_dummy * Jacob_t; // Null space
        *Null_space_iter = *Null_space_iter * Null_space_s.transpose();
    }
}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void OSC_Controller::AchieveCartManipulatorConstVel(Eigen::Vector3d mTarget, 
                                                double *svd_position,
                                                int cycle,
                                                Eigen::MatrixXd M, 
                                                Eigen::VectorXd C_t,
                                                Eigen::VectorXd g_t,
                                                dart::dynamics::SkeletonPtr mRobot,
                                                dart::dynamics::BodyNode* mEndEffector,
                                                Eigen::VectorXd *tau_total,
                                                Eigen::VectorXd *tau_ns,
                                                Eigen::MatrixXd *Null_space_iter){

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate operational space matrices 

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    Eigen::MatrixXd Jacob_t = mEndEffector->getLinearJacobian(); // Jacobian
    Jacob_t.topLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;
    if(augmented_projections){
        Jacob_t = Jacob_t * (*Null_space_iter).transpose();
    }
    //std::cout << "Projected Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = mEndEffector->getLinearJacobianDeriv(); // Derivative of jacobian
    Jacob_dot.topLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
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

    Eigen::MatrixXd Alpha_task = Alpha_ns;
    if(cycle==2){
        Alpha_task = Alpha_s;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Base_Jacob_dash = M.inverse() * Jacob_t.transpose();
    Eigen::MatrixXd Jacob_dash_ns = Base_Jacob_dash * Alpha_ns; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_s = Base_Jacob_dash * Alpha_s; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_dummy = Base_Jacob_dash * Alpha_s_dummy; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    Eigen::MatrixXd Jacob_dash_task = Jacob_dash_ns;
    if(cycle==2){
        Jacob_dash_task = Jacob_dash_s;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    Eigen::Vector3d x_error =  mTarget - mEndEffector->getWorldTransform().translation(); // Position error
    //std::cout<< "Cartesian error: \n" << x_error << std::endl;

    // Obtain Position Gain matrices
    Eigen::MatrixXd kp = kp_cartesian_.topLeftCorner(3, 3);

    Eigen::MatrixXd damp_coeff = kd_cartesian_.topLeftCorner(3, 3);
    Eigen::MatrixXd kd = calcDampingMatrix(Eigen::MatrixXd::Identity(3,3), kp, damp_coeff); 

    //std::cout << "Kp: \n" << kp <<std::endl;
    //std::cout << "Kd: \n" << kd <<std::endl;

    Eigen::Vector3d x_dot_desired = kp*kd.inverse()*x_error;
    //std::cout << "x_dot_desired: \n" << x_dot_desired << std::endl;

    double scale = std::min(1.0, (max_lineal_vel_ / x_dot_desired.norm()));
    //std::cout << "Scale V: \n" << scale << std::endl;

    Eigen::VectorXd x_star =  (-1.0*kd) * (mEndEffector->getLinearVelocity() - scale*x_dot_desired); // Command force vector
    if(compensate_topdown){
        x_star = x_star - Jacob_dash_ns.transpose() * *tau_total;
    }
    //std::cout << "ref accel: \n" << x_star << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational force due to task

    Eigen::VectorXd f_star = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd f_star_s = Eigen::VectorXd::Zero(3);

    if(compensate_jtspace){
        f_star =  Alpha_task * ( x_star - Jacob_dot * q_dot);

        // If the method is without the torque projections
        if(singularity_handling_method==1){
            f_star_s  =  Alpha_s  * ( x_star - Jacob_dot * q_dot);
        }
    }
    else{
        Eigen::VectorXd niu = Jacob_dash_task.transpose() * C_t  - Alpha_task * Jacob_dot * q_dot; // Operational Coriolis vector  
        Eigen::VectorXd p   = Jacob_dash_task.transpose() * g_t; // Operational Gravity vector
        f_star =  Alpha_task * x_star + niu + p; // Command forces vector for task

        // If the method is without the torque projections
        if(singularity_handling_method==1){
            Eigen::VectorXd niu_s = Jacob_dash_s.transpose() * C_t  - Alpha_s * Jacob_dot * q_dot; // Operational Coriolis vector  
            Eigen::VectorXd p_s = Jacob_dash_s.transpose() * g_t; // Operational Gravity vector
            f_star_s =  Alpha_s * x_star + niu_s + p_s; // Command forces vector for task
        }
    }
    
    // If the method is with the projections of non-singular tasks
    if(cycle==2){
        f_star = act_param * f_star + (1-act_param) * (Jacob_dash_task.transpose() * *tau_ns); // Scale Singular task by activation parameter
    }

    // If the method is without the torque projections
    if(singularity_handling_method==1){
        f_star_s = act_param * f_star_s; // Scale Singular task by activation parameter
        f_star = f_star + f_star_s; // Add non-singular and singular components
    }

    //std::cout << "F star:  \n" << f_star << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Joint torque due to task

    Eigen::VectorXd tau_star =  Jacob_t.transpose() * f_star;
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

    if(singularity_handling_method != 0){
        Eigen::MatrixXd Null_space_s =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_dummy * Jacob_t; // Null space
        *Null_space_iter = *Null_space_iter * Null_space_s.transpose();
    }
}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void OSC_Controller::AchievePosZ( Eigen::Vector3d mTargetPos, 
                                Eigen::Vector3d mTargetVel,
                                Eigen::Vector3d mTargetAccel, 
                                double *svd_position,
                                int cycle,
                                Eigen::MatrixXd M, 
                                Eigen::VectorXd C_t,
                                Eigen::VectorXd g_t,
                                dart::dynamics::SkeletonPtr mRobot,
                                dart::dynamics::BodyNode* mEndEffector,
                                Eigen::VectorXd *tau_total,
                                Eigen::VectorXd *tau_ns,
                                Eigen::MatrixXd *Null_space_iter){

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate operational space matrices 

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    Eigen::MatrixXd Jacob_t = mEndEffector->getLinearJacobian().bottomRows(1); // Jacobian
    //Jacob_t.topLeftCorner(1,3) = Eigen::MatrixXd::Zero(1,3);
    if(augmented_projections){
        Jacob_t = Jacob_t * (*Null_space_iter).transpose();
    }
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = mEndEffector->getLinearJacobianDeriv().bottomRows(1); // Derivative of jacobian
    //Jacob_dot.topLeftCorner(1,3) = Eigen::MatrixXd::Zero(1,3);
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

    Eigen::MatrixXd Alpha_task = Alpha_ns;
    if(cycle==2){
        Alpha_task = Alpha_s;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Base_Jacob_dash = M.inverse() * Jacob_t.transpose();
    Eigen::MatrixXd Jacob_dash_ns = Base_Jacob_dash * Alpha_ns; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_s = Base_Jacob_dash * Alpha_s; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_dummy = Base_Jacob_dash * Alpha_s_dummy; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    Eigen::MatrixXd Jacob_dash_task = Jacob_dash_ns;
    if(cycle==2){
        Jacob_dash_task = Jacob_dash_s;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    Eigen::VectorXd e =  mTargetPos - mEndEffector->getWorldTransform().translation(); // Position error
    //std::cout<< "Error : \n" << e << std::endl;

    Eigen::VectorXd de = mTargetVel - mEndEffector->getLinearVelocity();  // Velocity error (Target velocity is zero)

    // Obtain Position Gain matrices
    Eigen::MatrixXd kp = (kp_cartesian_.topLeftCorner(3, 3)).bottomRightCorner(1,1);

    Eigen::MatrixXd damp_coeff = (kd_cartesian_.topLeftCorner(3, 3)).bottomRightCorner(1,1);
    Eigen::MatrixXd kd = calcDampingMatrix(Eigen::MatrixXd::Identity(1,1), kp, damp_coeff); 

    Eigen::VectorXd x_star = mTargetAccel.bottomRows(1) + kd*de.bottomRows(1) + kp*e.bottomRows(1) ; // Command force vector
    //std::cout << "Command Accel: \n" << x_star << std::endl;

    if(compensate_topdown){
        x_star = x_star - Jacob_dash_ns.transpose() * *tau_total;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational force due to task

    Eigen::VectorXd f_star = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd f_star_s = Eigen::VectorXd::Zero(3);

    if(compensate_jtspace){
        f_star =  Alpha_task * ( x_star - Jacob_dot * q_dot);

        // If the method is without the torque projections
        if(singularity_handling_method==1){
            f_star_s  =  Alpha_s  * ( x_star - Jacob_dot * q_dot);
        }
    }
    else{
        Eigen::VectorXd niu = Jacob_dash_task.transpose() * C_t  - Alpha_task * Jacob_dot * q_dot; // Operational Coriolis vector  
        Eigen::VectorXd p   = Jacob_dash_task.transpose() * g_t; // Operational Gravity vector
        f_star =  Alpha_task * x_star + niu + p; // Command forces vector for task

        // If the method is without the torque projections
        if(singularity_handling_method==1){
            Eigen::VectorXd niu_s = Jacob_dash_s.transpose() * C_t  - Alpha_s * Jacob_dot * q_dot; // Operational Coriolis vector  
            Eigen::VectorXd p_s = Jacob_dash_s.transpose() * g_t; // Operational Gravity vector
            f_star_s =  Alpha_s * x_star + niu_s + p_s; // Command forces vector for task
        }
    }
    
    // If the method is with the projections of non-singular tasks
    if(cycle==2){
        f_star = act_param * f_star + (1-act_param) * (Jacob_dash_task.transpose() * *tau_ns); // Scale Singular task by activation parameter
    }

    // If the method is without the torque projections
    if(singularity_handling_method==1){
        f_star_s = act_param * f_star_s; // Scale Singular task by activation parameter
        f_star = f_star + f_star_s; // Add non-singular and singular components
    }

    //std::cout << "F star:  \n" << f_star << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Joint torque due to task

    Eigen::VectorXd tau_star =  Jacob_t.transpose() * f_star;
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

    if(singularity_handling_method != 0){
        Eigen::MatrixXd Null_space_s =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_dummy * Jacob_t; // Null space
        *Null_space_iter = *Null_space_iter * Null_space_s.transpose();
    }
}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void OSC_Controller::AchievePosZConstVel( Eigen::Vector3d mTarget, 
                                        double *svd_position,
                                        int cycle,
                                        Eigen::MatrixXd M, 
                                        Eigen::VectorXd C_t,
                                        Eigen::VectorXd g_t,
                                        dart::dynamics::SkeletonPtr mRobot,
                                        dart::dynamics::BodyNode* mEndEffector,
                                        Eigen::VectorXd *tau_total,
                                        Eigen::VectorXd *tau_ns,
                                        Eigen::MatrixXd *Null_space_iter){

    // ------------------------------------------//
    // ------------------------------------------//
    // Calculate operational space matrices 

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    Eigen::MatrixXd Jacob_t = (mEndEffector->getLinearJacobian()).bottomRightCorner(1,dofs); // Jacobian
    //Jacob_t.topLeftCorner(1,3) = Eigen::MatrixXd::Zero(1,3);
    if(augmented_projections){
        Jacob_t = Jacob_t * (*Null_space_iter).transpose();
    }
    //std::cout << "Linear Jacob: \n" << Jacob_t << std::endl;

    Eigen::MatrixXd Jacob_dot = (mEndEffector->getLinearJacobianDeriv()).bottomRightCorner(1,dofs); // Derivative of jacobian
    //Jacob_dot.topLeftCorner(1,3) = Eigen::MatrixXd::Zero(1,3);
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

    Eigen::MatrixXd Alpha_task = Alpha_ns;
    if(cycle==2){
        Alpha_task = Alpha_s;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Dynamic consistent inverse Jacobian

    Eigen::MatrixXd Base_Jacob_dash = M.inverse() * Jacob_t.transpose();
    Eigen::MatrixXd Jacob_dash_ns = Base_Jacob_dash * Alpha_ns; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_s = Base_Jacob_dash * Alpha_s; // Dynamically consistent inverse jacobian
    Eigen::MatrixXd Jacob_dash_dummy = Base_Jacob_dash * Alpha_s_dummy; // Dynamically consistent inverse jacobian
    //std::cout << "Inverse Jacobian: \n" << Jacob_dash_t << std::endl;

    Eigen::MatrixXd Jacob_dash_task = Jacob_dash_ns;
    if(cycle==2){
        Jacob_dash_task = Jacob_dash_s;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational acceleration due to task

    Eigen::VectorXd x_error =  (mTarget - mEndEffector->getWorldTransform().translation()).bottomRows(1); // Position error
    //std::cout<< "Cartesian error: \n" << x_error << std::endl;

    // Obtain Position Gain matrices
    Eigen::MatrixXd kp = (kp_cartesian_.topLeftCorner(3,3)).bottomRightCorner(1,1);

    Eigen::MatrixXd damp_coeff = (kd_cartesian_.topLeftCorner(3,3)).bottomRightCorner(1,1);
    Eigen::MatrixXd kd = calcDampingMatrix(Eigen::MatrixXd::Identity(1,1), kp, damp_coeff); 

    Eigen::VectorXd x_dot_desired = kp*kd.inverse()*x_error;

    double scale = std::min(1.0, (max_lineal_vel_ / x_dot_desired.norm()));
    //std::cout << "Scale V: \n" << scale << std::endl;

    Eigen::VectorXd x_star =  (-1.0*kd) * ( (mEndEffector->getLinearVelocity()).bottomRows(1) - scale*x_dot_desired); // Command force vector
    
    //std::cout << "Ref Accel: \n" << x_star << std::endl;
    
    if(compensate_topdown){
        x_star = x_star - Jacob_dash_ns.transpose() * *tau_total;
    }

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Operational force due to task

    Eigen::VectorXd f_star = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd f_star_s = Eigen::VectorXd::Zero(3);

    if(compensate_jtspace){
        f_star =  Alpha_task * ( x_star - Jacob_dot * q_dot);

        // If the method is without the torque projections
        if(singularity_handling_method==1){
            f_star_s  =  Alpha_s  * ( x_star - Jacob_dot * q_dot);
        }
    }
    else{
        Eigen::VectorXd niu = Jacob_dash_task.transpose() * C_t  - Alpha_task * Jacob_dot * q_dot; // Operational Coriolis vector  
        Eigen::VectorXd p   = Jacob_dash_task.transpose() * g_t; // Operational Gravity vector
        f_star =  Alpha_task * x_star + niu + p; // Command forces vector for task

        // If the method is without the torque projections
        if(singularity_handling_method==1){
            Eigen::VectorXd niu_s = Jacob_dash_s.transpose() * C_t  - Alpha_s * Jacob_dot * q_dot; // Operational Coriolis vector  
            Eigen::VectorXd p_s = Jacob_dash_s.transpose() * g_t; // Operational Gravity vector
            f_star_s =  Alpha_s * x_star + niu_s + p_s; // Command forces vector for task
        }
    }
    
    // If the method is with the projections of non-singular tasks
    if(cycle==2){
        f_star = act_param * f_star + (1-act_param) * (Jacob_dash_task.transpose() * *tau_ns); // Scale Singular task by activation parameter
    }

    // If the method is without the torque projections
    if(singularity_handling_method==1){
        f_star_s = act_param * f_star_s; // Scale Singular task by activation parameter
        f_star = f_star + f_star_s; // Add non-singular and singular components
    }

    //std::cout << "F star:  \n" << f_star << std::endl;

    // ------------------------------------------//
    // ------------------------------------------//
    // Calc Joint torque due to task

    Eigen::VectorXd tau_star =  Jacob_t.transpose() * f_star;
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

    if(singularity_handling_method != 0){
        Eigen::MatrixXd Null_space_s =  Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_dummy * Jacob_t; // Null space
        *Null_space_iter = *Null_space_iter * Null_space_s.transpose();
    }
}

} // end namespace
