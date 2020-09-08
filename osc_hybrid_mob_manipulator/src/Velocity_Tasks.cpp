#include <osc_hybrid_mob_manipulator/Velocity_Tasks.hpp>

namespace vel_tasks {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Constructor
VelocityTask::VelocityTask(){

    kp_cartesian_ = 400.0;
    kp_joint_     = 40.0;

}
////////////////////////////////////////////////////////////////////////////////
// Destructor
VelocityTask::~VelocityTask()
{
}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a cartesian position

void VelocityTask::AchieveCartesian(Eigen::VectorXd *q_zero,
                                    Eigen::VectorXd *q_result,
                                    Eigen::Vector3d mTarget, 
                                    Eigen::MatrixXd M,
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector){
    
    // Due to the fact that in this controller only the velocities of the 
    // platform are send, the error in the Z axis is better to leave at
    // zero
    //mTarget(2) = (mEndEffector->getWorldTransform().translation())(2);

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

    // ------------------------------------------//
    // ------------------------------------------//

    Eigen::Vector3d x_dot_desired = Eigen::Vector3d::Zero(); // Desired velocity is zero

    //std::cout << "Desired pos: \n" << mTarget << std::endl;
    //std::cout << "End effector pos: \n" << mEndEffector->getWorldTransform().translation() << std::endl;

    //Eigen::Vector3d pos_error   =  mTarget - mEndEffector->getWorldTransform().translation(); // Position error
    Eigen::Vector3d pos_error   =  mEndEffector->getWorldTransform().translation() - mTarget; // Position error
    //std::cout << "Pos Error: \n" << ( x_dot_desired + kp_cartesian_ * ( pos_error )) << std::endl;

    Eigen::VectorXd cmd_vel = Jacob_dash_t * ( x_dot_desired + kp_cartesian_ * ( pos_error ) );
    //cmd_vel(0) = -cmd_vel(0);
    //cmd_vel(1) = -cmd_vel(1);
    //cmd_vel(2) = -cmd_vel(2);
    //std::cout << "Command velocities: \n" << cmd_vel << std::endl;

    Eigen::MatrixXd N_t = Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_t * Jacob_t ;

    *q_result = cmd_vel + N_t * *q_zero;
    *q_zero   = *q_result;
    //std::cout << "Command velocities: \n" << q_result << std::endl;

}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve a joint configuration

void VelocityTask::AchieveJointConf(Eigen::VectorXd *q_zero,
                                    Eigen::VectorXd *q_result,
                                    Eigen::VectorXd q_desired,
                                    dart::dynamics::SkeletonPtr mRobot,
                                    dart::dynamics::BodyNode* mEndEffector){

    // ------------------------------------------//
    // ------------------------------------------//

    std::size_t dofs = mEndEffector->getNumDependentGenCoords();

    Eigen::VectorXd q_dot_desired = Eigen::VectorXd::Zero(dofs); // Desired joint vel is zero 
    //std::cout << "q desired: \n" << q_desired << std::endl;
    //std::cout << "q dot desired: \n" << q_dot_desired << std::endl;

    Eigen::VectorXd joint_pos_error = q_desired - mRobot->getPositions(); // Position error

    Eigen::VectorXd cmd_vel = q_dot_desired + kp_joint_ * ( joint_pos_error ) ;
    //std::cout << "Cmd_vel result: \n" << cmd_vel << std::endl;

    // Due to the fact that in this case the Jacobian becomes the identity
    // the inverse is also the identity. Due to this N_t = I - Jacob_inv * Jacob
    // becomes the zero matrix.

    *q_result = cmd_vel;
    *q_zero   = *q_result;
    //std::cout << "Command velocities: \n" << q_result << std::endl;

}

////////////////////////////////////////////////////////////////////////////////
// Function to calculate the efforts required to Hold/Achieve an orientation

void VelocityTask::AchieveOrientation(  Eigen::VectorXd *q_zero,
                                        Eigen::VectorXd *q_result,
                                        Eigen::Matrix3d rot_mat_desired, 
                                        Eigen::MatrixXd M,
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

    // ------------------------------------------//
    // ------------------------------------------//

    Eigen::Matrix3d R_world_EE = mEndEffector->getWorldTransform().rotation();
    //std::cout << "Rotation matrix from W to EE: \n" << R_world_EE << std::endl;
    //std::cout << "Desired matrix from W to EE: \n"  << rot_mat_desired << std::endl;

    Eigen::Matrix3d R_EE_desired = R_world_EE.transpose() * rot_mat_desired;
    //std::cout << "Desired rotation matrix in EE: \n" << R_EE_desired << std::endl;

    Eigen::AngleAxisd u_ee (R_EE_desired);

    Eigen::Vector3d axis_desired = R_world_EE * u_ee.axis();
    double angle_desired = u_ee.angle();
    //std::cout << "Axis of vector u_world: \n"  << axis_desired  << std::endl;
    //std::cout << "Angle of vector u_world: \n" << angle_desired << std::endl;

    Eigen::Vector3d angular_vel =  Eigen::Vector3d::Zero();
    angular_vel = mEndEffector->getAngularJacobian() * mRobot->getVelocities();
    //std::cout << "Angular velocity vector: \n"  << angular_vel << std::endl;

    Eigen::VectorXd cmd_vel = Jacob_dash_t * ( angular_vel + (axis_desired * kp_cartesian_ * angle_desired) ) ; // Command celocity vector
    //std::cout << "Command velocities: \n" << cmd_vel << std::endl;

    Eigen::MatrixXd N_t = Eigen::MatrixXd::Identity(dofs, dofs) - Jacob_dash_t * Jacob_t ;

    *q_result = cmd_vel + N_t * *q_zero;
    *q_zero   = *q_result;
    //std::cout << "Command velocities: \n" << q_result << std::endl;

}

} /* namespace */