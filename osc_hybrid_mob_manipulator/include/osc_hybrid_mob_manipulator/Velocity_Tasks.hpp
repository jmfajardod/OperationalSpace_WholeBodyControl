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

namespace vel_tasks {

/*!
 * Class containing the Velocity Task Algorithms
 */
class VelocityTask {
public:
	/*!
	 * Constructor.
	 */
	VelocityTask();

	/*!
	 * Destructor.
	 */
	virtual ~VelocityTask();

    /*!
     * Velocity Task
     * Hold/ Achieve a Cartesian Position
    */
    void AchieveCartesian(  Eigen::VectorXd *q_zero,
                            Eigen::VectorXd *q_result,
                            Eigen::Vector3d mTarget, 
                            Eigen::MatrixXd M,
                            dart::dynamics::SkeletonPtr mRobot,
                            dart::dynamics::BodyNode* mEndEffector);

    /*!
     * Velocity Task
     * Hold/ Achieve a Joint configuration
    */
    void AchieveJointConf(  Eigen::VectorXd *q_zero,
                            Eigen::VectorXd *q_result,
                            Eigen::VectorXd q_desired,
                            dart::dynamics::SkeletonPtr mRobot,
                            dart::dynamics::BodyNode* mEndEffector);

    /*!
     * Velocity Task
     * Hold/ Achieve an orientation
    */
    void AchieveOrientation(Eigen::VectorXd *q_zero,
                            Eigen::VectorXd *q_result,
                            Eigen::Matrix3d rot_mat_desired, 
                            Eigen::MatrixXd M,
                            dart::dynamics::SkeletonPtr mRobot,
                            dart::dynamics::BodyNode* mEndEffector);

private:

    double kp_cartesian_;
    double kp_joint_;

};

} /* namespace */
