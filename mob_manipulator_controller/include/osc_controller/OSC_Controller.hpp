#pragma once

#include <iostream> 
#include <algorithm> 
#include <Eigen/Dense>
#include <math.h> 
#include <vector>

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

namespace osc_controller {

/*!
 * Class containing the Effort Task Algorithms
 */
class OSC_Controller {
public:
	/*!
	 * Constructor.
	 */
	OSC_Controller();

	/*!
	 * Destructor.
	 */
	virtual ~OSC_Controller();

    /*!
	 * Function to change the gains of the controller
     * KP_C and KD_C are the cartesian gains
     * KP_J and KD_J are the joint gains
	 */
	void changeCartesianPositionGains(double Pos_X_stiffness, double Pos_Y_stiffness, double Pos_Z_stiffness,
																		double Pos_X_damping, double Pos_Y_damping, double Pos_Z_damping);

	void changeCartesianOrientationGains(double Ori_X_stiffness, double Ori_Y_stiffness, double Ori_Z_stiffness,
																			double Ori_X_damping, double Ori_Y_damping, double Ori_Z_damping);

    /*!
	 * Function to change the higher threshold for singularities
	 */
    void changeHighThSing(double new_high_thr);

    /*!
	 * Function to change the lower threshold for singularities
	 */
    void changeLowThSing(double new_low_thr);

    /***********************************************************************************/
    /***********************************************************************************/
    // Joint Tasks

    /*!
     * Effort Task
     * Hold/ Achieve a Joint Configuration
    */
    void AchieveJointConf(  Eigen::VectorXd q_desired, 
                            Eigen::MatrixXd M, 
                            Eigen::VectorXd C_t,
                            Eigen::VectorXd g_t,
                            dart::dynamics::SkeletonPtr mRobot,
                            dart::dynamics::BodyNode* mEndEffector,
                            Eigen::VectorXd *tau_total,
                            Eigen::MatrixXd *Null_space_iter);

    /***********************************************************************************/
    /***********************************************************************************/
    // Joint limit avoidance tasks

    /*!
     * Effort Task
     * Avoid Joint Limits
    */
    void AvoidJointLimitsPotentials(Eigen::MatrixXd M, 
                                    Eigen::VectorXd C_t,
                                    Eigen::VectorXd g_t,
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector,
                                    Eigen::VectorXd *tau_total,
                                    Eigen::MatrixXd *Null_space_iter);

    void AvoidJointLimitsIntermValue(Eigen::MatrixXd M, 
                                    Eigen::VectorXd C_t,
                                    Eigen::VectorXd g_t,
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector,
                                    Eigen::VectorXd *tau_total,
                                    Eigen::MatrixXd *Null_space_iter);

    void updateSJSConstraints(dart::dynamics::SkeletonPtr mRobot, double sampling_time);

    void checkSJSConstraints(Eigen::VectorXd Joint_Acceleration,
                            dart::dynamics::BodyNode* mEndEffector,
                            bool* flag_sjs);

    void updateSJSConstraintTask(Eigen::VectorXd Current_joint_accel,
                                bool* flag_sjs,
                                Eigen::MatrixXd *Jacobian_constr,
                                Eigen::VectorXd *Constr_Task_Acceleration);

    void SaturationJointSpace(Eigen::MatrixXd Jacobian,
                              Eigen::VectorXd desired_accel,
                              Eigen::MatrixXd M, 
                              Eigen::VectorXd C_t,
                              Eigen::VectorXd g_t,
                              dart::dynamics::SkeletonPtr mRobot,
                              dart::dynamics::BodyNode* mEndEffector,
                              Eigen::VectorXd *tau_total,
                              Eigen::MatrixXd *Null_space_iter);

    /***********************************************************************************/
    /***********************************************************************************/
    // Position tasks

    /*******************************************************/
    // Mobile manipulator tasks

    /*!
     * Effort Task
     * Cartesian Impedance for Position
    */
    void CartesianImpedance(  Eigen::Vector3d mTargetPos, 
                              Eigen::Vector3d mTargetVel,
                              Eigen::Vector3d mTargetAccel, 
                              double *svd_position,
                              int cycle,
                              Eigen::VectorXd tau_ext,
                              Eigen::MatrixXd M, 
                              Eigen::VectorXd C_t,
                              Eigen::VectorXd g_t,
                              dart::dynamics::SkeletonPtr mRobot,
                              dart::dynamics::BodyNode* mEndEffector,
                              Eigen::VectorXd *tau_total,
                              Eigen::VectorXd *tau_ns,
                              Eigen::MatrixXd *Null_space_iter);

    /*!
     * Effort Task
     * Hold/ Achieve a Cartesian Position
    */
    void AchieveCartesian(  Eigen::Vector3d mTargetPos, 
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
                            Eigen::MatrixXd *Null_space_iter);

    /*!
     * Effort Task
     * Go to a desired position with constant vel
    */
    void AchieveCartesianConstVel(  Eigen::Vector3d mTarget, 
                                    double *svd_position,
                                    int cycle,
                                    Eigen::MatrixXd M,
                                    Eigen::VectorXd C_t,
                                    Eigen::VectorXd g_t,
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector,
                                    Eigen::VectorXd *tau_total,
                                    Eigen::VectorXd *tau_ns,
                                    Eigen::MatrixXd *Null_space_iter);

    /*******************************************************/
    // Mobile robot tasks

    /*!
     * Effort Task
     * Tasks using only the DOF of the mobile robot
    */

    void AchieveCartesianMobilRob( Eigen::Vector3d mTargetPos, 
                                    Eigen::Vector3d mTargetVel,
                                    Eigen::Vector3d mTargetAccel, 
                                    double *svd_position,
                                    Eigen::MatrixXd M, 
                                    Eigen::VectorXd C_t,
                                    Eigen::VectorXd g_t,
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector,
                                    Eigen::VectorXd *tau_total,
                                    Eigen::MatrixXd *Null_space_iter);

    void AchieveCartesianMobilRobConstVel(  Eigen::Vector3d mTargetPos,
                                            double *svd_position,
                                            Eigen::MatrixXd M, 
                                            Eigen::VectorXd C_t,
                                            Eigen::VectorXd g_t,
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector,
                                            Eigen::VectorXd *tau_total,
                                            Eigen::MatrixXd *Null_space_iter);

    /*******************************************************/
    // Manipulator tasks

    /*!
     * Effort Task
     * Tasks using only the DOF of the manipulator
    */

    void AchieveCartesianManipulator(Eigen::Vector3d mTargetPos, 
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
                                    Eigen::MatrixXd *Null_space_iter);

    void AchieveCartManipulatorConstVel(Eigen::Vector3d mTarget, 
                                        double *svd_position,
                                        int cycle,
                                        Eigen::MatrixXd M, 
                                        Eigen::VectorXd C_t,
                                        Eigen::VectorXd g_t,
                                        dart::dynamics::SkeletonPtr mRobot,
                                        dart::dynamics::BodyNode* mEndEffector,
                                        Eigen::VectorXd *tau_total,
                                        Eigen::VectorXd *tau_ns,
                                        Eigen::MatrixXd *Null_space_iter);

    void AchievePosZ( Eigen::Vector3d mTargetPos, 
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
                      Eigen::MatrixXd *Null_space_iter);

    void AchievePosZConstVel( Eigen::Vector3d mTarget,
                              double *svd_position, 
                              int cycle,
                              Eigen::MatrixXd M, 
                              Eigen::VectorXd C_t,
                              Eigen::VectorXd g_t,
                              dart::dynamics::SkeletonPtr mRobot,
                              dart::dynamics::BodyNode* mEndEffector,
                              Eigen::VectorXd *tau_total,
                              Eigen::VectorXd *tau_ns,
                              Eigen::MatrixXd *Null_space_iter);


    /***********************************************************************************/
    /***********************************************************************************/
    // Orientation tasks

    /*******************************************************/
    // Mobile manipulator tasks

    /*!
     * Effort Task
     * Orientation Impedance
    */
    void OrientationImpedance(Eigen::Matrix3d rot_mat_desired, 
                              Eigen::Vector3d mTargetVel,
                              Eigen::Vector3d mTargetAccel,
                              double *svd_orientation,
                              int cycle,
                              Eigen::VectorXd tau_ext,
                              Eigen::MatrixXd M,
                              Eigen::VectorXd C_t,
                              Eigen::VectorXd g_t,
                              dart::dynamics::SkeletonPtr mRobot,
                              dart::dynamics::BodyNode* mEndEffector,
                              Eigen::VectorXd *tau_total,
                              Eigen::VectorXd *tau_ns,
                              Eigen::MatrixXd *Null_space_iter);

    /*!
     * Effort Task
     * Hold/ Achieve an Orientation
    */
    void AchieveOrientation(Eigen::Matrix3d rot_mat_desired, 
                            Eigen::Vector3d mTargetVel,
                            Eigen::Vector3d mTargetAccel,
                            double *svd_orientation,
                            int cycle,
                            Eigen::MatrixXd M,
                            Eigen::VectorXd C_t,
                            Eigen::VectorXd g_t,
                            dart::dynamics::SkeletonPtr mRobot,
                            dart::dynamics::BodyNode* mEndEffector,
                            Eigen::VectorXd *tau_total,
                            Eigen::VectorXd *tau_ns,
                            Eigen::MatrixXd *Null_space_iter);

    /*!
     * Effort Task
     * Go to a desired orientation with constant vel
    */
    void AchieveOrientationConstVel(Eigen::Matrix3d rot_mat_desired,
                                    double *svd_orientation,
                                    int cycle,
                                    Eigen::MatrixXd M,
                                    Eigen::VectorXd C_t,
                                    Eigen::VectorXd g_t,
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector,
                                    Eigen::VectorXd *tau_total,
                                    Eigen::VectorXd *tau_ns,
                                    Eigen::MatrixXd *Null_space_iter);

    /*******************************************************/
    // Manipulator tasks

    void AchieveOriManipulator( Eigen::Matrix3d rot_mat_desired, 
                                Eigen::Vector3d mTargetVel,
                                Eigen::Vector3d mTargetAccel,
                                double *svd_orientation,
                                int cycle,
                                Eigen::MatrixXd M,
                                Eigen::VectorXd C_t,
                                Eigen::VectorXd g_t,
                                dart::dynamics::SkeletonPtr mRobot,
                                dart::dynamics::BodyNode* mEndEffector,
                                Eigen::VectorXd *tau_total,
                                Eigen::VectorXd *tau_ns,
                                Eigen::MatrixXd *Null_space_iter);

    void AchieveOriManipulatorConstVel( Eigen::Matrix3d rot_mat_desired, 
                                        double *svd_orientation,
                                        int cycle,
                                        Eigen::MatrixXd M,
                                        Eigen::VectorXd C_t,
                                        Eigen::VectorXd g_t,
                                        dart::dynamics::SkeletonPtr mRobot,
                                        dart::dynamics::BodyNode* mEndEffector,
                                        Eigen::VectorXd *tau_total,
                                        Eigen::VectorXd *tau_ns,
                                        Eigen::MatrixXd *Null_space_iter);


    /*******************************************************/
    // Error Functions

    Eigen::Vector3d ErrorAngleAxis1(Eigen::Matrix3d rot_mat_desired, 
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector);

    Eigen::Vector3d ErrorAngleAxis2(Eigen::Matrix3d rot_mat_desired, 
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector);

    Eigen::Vector3d ErrorQuaternion1(Eigen::Matrix3d rot_mat_desired, 
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector);

    Eigen::Vector3d ErrorQuaternion2(Eigen::Matrix3d rot_mat_desired, 
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector);

    Eigen::Vector3d ErrorQuaternion3(Eigen::Matrix3d rot_mat_desired, 
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector);

    /***********************************************************************************/
    /***********************************************************************************/
    // Whole body controller functions

    Eigen::VectorXd admittance_controller(Eigen::VectorXd torque_out);

    /***********************************************************************************/
    /***********************************************************************************/
    // Miscellaneous functions

    Eigen::MatrixXd calcInertiaMatrix(Eigen::MatrixXd Alpha_inv, double* min_svd);

    void calcInertiaMatrixHandling( Eigen::MatrixXd Alpha_inv,
                                    double* min_svd,
                                    double* act_param,
                                    Eigen::MatrixXd *Alpha_ns,
                                    Eigen::MatrixXd *Alpha_s,
                                    Eigen::MatrixXd *Alpha_s_dummy);

    Eigen::MatrixXd calcDampingMatrix(Eigen::MatrixXd Alpha, 
                                      Eigen::MatrixXd Stiffness, 
                                      Eigen::MatrixXd DampingCoeff);

    /***********************************************************************************/
    /***********************************************************************************/
    // Public variables

    // Variables for hierarchy control (WBC and OSC)
    bool compensate_topdown;
    bool compensate_jtspace;
    bool augmented_projections;

    // Variables for tasks
    double max_lineal_vel_;
    double max_angular_vel_;
    
    // Variable to select the method for orientacion error calculation
    int ori_error_mode;

    // Variables for singularity handling method
    int singularity_handling_method;

    double singularity_thres_high_pos_;
    double singularity_thres_low_pos_;

    double singularity_thres_high_ori_;
    double singularity_thres_low_ori_;

    // Variables for joint limit avoidance methods
    int joint_limit_handling_method;

    Eigen::VectorXd Lower_limits;
    Eigen::VectorXd Upper_limits;

    Eigen::VectorXd joint_margin_;

    double eta_firas_;

    // Variables for intermediate value
    int   interm_alg_update_null;
    Eigen::VectorXd joint_limit_buffer;
    double gain_limit_avoidance;
    double scale_null_space;

    // Variables for SJS
    Eigen::VectorXd Max_constraint_accel;
    Eigen::VectorXd Min_constraint_accel;

    Eigen::VectorXd Max_joint_accel;
    Eigen::VectorXd Min_joint_accel;

    Eigen::VectorXd Max_joint_vel;
    Eigen::VectorXd Min_joint_vel;

    Eigen::VectorXd Max_joint_pos;
    Eigen::VectorXd Min_joint_pos;


private:

    // Gain matrices for tasks in operational and joint spaces
    Eigen::MatrixXd kp_cartesian_;
    Eigen::MatrixXd kd_cartesian_;
    Eigen::MatrixXd kp_joints_;
    Eigen::MatrixXd kd_joints_;

    // Variables for singularity handling method
    double singularity_thres_high_;
    double singularity_thres_low_;

    // Gains for WBC
    double admittance_linear_damping;
    double admittance_angular_damping;
    Eigen::MatrixXd admittance_desired_inertia;

};

} /* namespace */
