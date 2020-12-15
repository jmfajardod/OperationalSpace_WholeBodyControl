#include <mob_manipulator_controller/Effort_Tasks.hpp>

namespace effort_tasks {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

/******************************************************************/
/******************************************************************/
// Error functions

// Orientation error Osorio
Eigen::Vector3d EffortTask::ErrorAngleAxis1(Eigen::Matrix3d rot_mat_desired, 
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector){


    Eigen::Matrix3d R_world_EE = mEndEffector->getWorldTransform().rotation();
    //std::cout << "Rotation matrix from W to EE: \n" << R_world_EE << std::endl;

    Eigen::Matrix3d R_EE_desired = R_world_EE.transpose() * rot_mat_desired;
    //std::cout << "Desired rotation matrix in EE: \n" << R_EE_desired << std::endl;

    Eigen::AngleAxisd u_ee (R_EE_desired);

    Eigen::Vector3d axis_desired = R_world_EE * u_ee.axis();
    double angle_desired = u_ee.angle();
    //std::cout << "Axis of vector u_world: \n"  << axis_desired  << std::endl;
    //std::cout << "Angle of vector u_world: \n" << angle_desired << std::endl;

    Eigen::Vector3d angular_vel =  mEndEffector->getAngularJacobian() * mRobot->getVelocities();
    //std::cout << "Angular velocity vector: \n"  << angular_vel << std::endl;

    Eigen::Vector3d error_ori =  axis_desired * angle_desired ; // Orientation error

    return error_ori;                                

}

// Orientation error Caccavale
Eigen::Vector3d EffortTask::ErrorAngleAxis2(Eigen::Matrix3d rot_mat_desired, 
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector){

    Eigen::Matrix3d R_world_EE = mEndEffector->getWorldTransform().rotation();
    //std::cout << "Rotation matrix from W to EE: \n" << R_world_EE << std::endl;

    Eigen::Matrix3d R_EE_desired =  rot_mat_desired * R_world_EE.transpose() ;
    //std::cout << "Desired rotation matrix in EE: \n" << R_EE_desired << std::endl;

    Eigen::AngleAxisd u_ee (R_EE_desired);

    Eigen::Vector3d error_ori =  sin(u_ee.angle())* u_ee.axis() ; // Command force vector
    
    return error_ori; 

}

// Orientation error Yuan
Eigen::Vector3d EffortTask::ErrorQuaternion1(Eigen::Matrix3d rot_mat_desired, 
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector){

    Eigen::Matrix3d R_world_EE = mEndEffector->getWorldTransform().rotation();
    //std::cout << "Rotation matrix from W to EE: \n" << R_world_EE << std::endl;

    Eigen::Quaterniond q_des(rot_mat_desired);
    Eigen::Quaterniond q_act(R_world_EE);
    
    Eigen::Vector3d error_ori = q_des.w()*q_act.vec() - q_act.w() * q_des.vec() + q_des.vec().cross(q_act.vec());
    //std::cout << "Orientation error: \n" << e_ori << std::endl;

    return error_ori;

}

// Orientation error Caccavale
Eigen::Vector3d EffortTask::ErrorQuaternion2(Eigen::Matrix3d rot_mat_desired, 
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector){
    
    Eigen::Matrix3d R_world_EE = mEndEffector->getWorldTransform().rotation();
    //std::cout << "Rotation matrix from W to EE: \n" << R_world_EE << std::endl;

    Eigen::Matrix3d R_EE_desired = R_world_EE.transpose() * rot_mat_desired; //
    //std::cout << "Desired rotation matrix in EE: \n" << R_EE_desired << std::endl;

    Eigen::Quaterniond quat(R_EE_desired);
    Eigen::Vector3d vec_des = R_world_EE * quat.vec();

    Eigen::Vector3d error_ori =   vec_des ; // Orientation error   

    return error_ori;

}

// Orientation error Caccavale
Eigen::Vector3d EffortTask::ErrorQuaternion3(Eigen::Matrix3d rot_mat_desired, 
                                            dart::dynamics::SkeletonPtr mRobot,
                                            dart::dynamics::BodyNode* mEndEffector){

    Eigen::Matrix3d R_world_EE = mEndEffector->getWorldTransform().rotation(); 
    //std::cout << "Rotation matrix from W to EE: \n" << R_world_EE << std::endl;

    Eigen::Matrix3d R_EE_desired = rot_mat_desired * R_world_EE.transpose();
    //std::cout << "Desired rotation matrix in EE: \n" << R_EE_desired << std::endl;

    Eigen::Quaterniond quat(R_EE_desired);
    //Eigen::Vector3d vec_des = R_world_EE * quat.vec();

    Eigen::Vector3d error_ori = 2*quat.w()*quat.vec() ; 

    return error_ori;

}

} // end namespace