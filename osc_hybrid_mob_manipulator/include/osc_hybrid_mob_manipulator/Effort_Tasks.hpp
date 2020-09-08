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

namespace effort_tasks {

/*!
 * Class containing the Effort Task Algorithms
 */
class EffortTask {
public:
	/*!
	 * Constructor.
	 */
	EffortTask();

	/*!
	 * Destructor.
	 */
	virtual ~EffortTask();

    /*!
	 * Function to change the gains of the controller
     * KP_C and KD_C are the cartesian gains
     * KP_J and KD_J are the joint gains
	 */
	void changeGains(double kp_c, double kd_c, double kp_j, double kd_j);

    /*!
	 * Function to change the maximum linear velocity of the end effector
	 */
    void changeMaxVel(double new_max_vel);

    /*!
	 * Function to change the joint margin for avoding joint limits
	 */
    void changeJointMargin(double new_margin);

    /*!
	 * Function to change the eta in FIRAS function for avoding joint limits
	 */
    void change_etaFIRAS(double new_eta);

    /*!
	 * Function to change the higher threshold for singularities
	 */
    void changeHighThSing(double new_high_thr);

    /*!
	 * Function to change the lower threshold for singularities
	 */
    void changeLowThSing(double new_low_thr);

    /*!
     * Effort Task
     * Hold/ Achieve a Cartesian Position
    */
    void AchieveCartesian(  Eigen::VectorXd *tau_zero, 
                            Eigen::VectorXd *tau_result,
                            Eigen::Vector3d mTarget,
                            Eigen::MatrixXd M,
                            Eigen::VectorXd C_t,
                            Eigen::VectorXd g_t,
                            dart::dynamics::SkeletonPtr mRobot,
                            dart::dynamics::BodyNode* mEndEffector);

    /*!
     * Effort Task
     * Hold/ Achieve a Joint Configuration
    */
    void AchieveJointConf(  Eigen::VectorXd *tau_zero, 
                            Eigen::VectorXd *tau_result,
                            Eigen::VectorXd q_desired, 
                            Eigen::MatrixXd M,
                            Eigen::VectorXd C_t,
                            Eigen::VectorXd g_t,
                            dart::dynamics::SkeletonPtr mRobot,
                            dart::dynamics::BodyNode* mEndEffector);

    /*!
     * Effort Task
     * Hold/ Achieve an Orientation
    */
    void AchieveOrientation(Eigen::VectorXd *tau_zero, 
                            Eigen::VectorXd *tau_result,
                            Eigen::Matrix3d rot_mat_desired,  
                            Eigen::MatrixXd M,
                            Eigen::VectorXd C_t,
                            Eigen::VectorXd g_t,
                            dart::dynamics::SkeletonPtr mRobot,
                            dart::dynamics::BodyNode* mEndEffector);

    /*!
     * Effort Task
     * Make a StraigthLine
    */
    void MakeStraightLine(  Eigen::VectorXd *tau_zero, 
                            Eigen::VectorXd *tau_result,
                            Eigen::Vector3d mTarget,
                            Eigen::MatrixXd M,
                            Eigen::VectorXd C_t,
                            Eigen::VectorXd g_t,
                            dart::dynamics::SkeletonPtr mRobot,
                            dart::dynamics::BodyNode* mEndEffector);

    /*!
     * Effort Task
     * Avoid Joint Limits
    */
    void AvoidJointLimits(  Eigen::VectorXd *tau_zero, 
                            Eigen::VectorXd *tau_result,
                            Eigen::MatrixXd M,
                            Eigen::VectorXd C_t,
                            Eigen::VectorXd g_t,
                            dart::dynamics::SkeletonPtr mRobot,
                            dart::dynamics::BodyNode* mEndEffector);

    /*!
     * Effort Task
     * Hold/ Achieve a Cartesian Position avoiding singularities
    */
    void CartesianAvoidSing(Eigen::VectorXd *tau_zero, 
                            Eigen::VectorXd *tau_result,
                            Eigen::Vector3d mTarget,
                            Eigen::MatrixXd M,
                            Eigen::VectorXd C_t,
                            Eigen::VectorXd g_t,
                            dart::dynamics::SkeletonPtr mRobot,
                            dart::dynamics::BodyNode* mEndEffector);

private:

    double kp_cartesian_;
    double kd_cartesian_;
    double kp_joints_;
    double kd_joints_;

    double max_vel_;

    double joint_margin_;
    double eta_firas_;

    double singularity_thres_high_;
    double singularity_thres_low_;

};

} /* namespace */
