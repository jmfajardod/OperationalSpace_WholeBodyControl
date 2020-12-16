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

#include <mob_manipulator_controller/Mob_Manipulator_Controller.hpp>

namespace mob_manipulator_controller {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

using namespace osc_controller;

////////////////////////////////////////////////////////////////////////////////
// Function to load the robot in DART
void MobManipulatorController::loadDARTModel(){

    dart::utils::DartLoader loader;

    // Load packages where models are located in DART
    for (size_t ii = 0; ii < model_packages_paths.size(); ii++){
        loader.addPackageDirectory(model_packages_names.at(ii), model_packages_paths.at(ii));
    }

    // Load the robot
    std::string filename = urdf_model_path;
    dart_robotSkeleton = loader.parseSkeleton(filename);

    /******************************/
    // Change Wheel0 joint to WeldJoint
    dart::dynamics::Joint::Properties auxJoint0 = dart_robotSkeleton->getJoint("wheel0_joint")->getJointProperties();

    dart::dynamics::WeldJoint::Properties fixedJoint0;
    fixedJoint0.mName = auxJoint0.mName;
    fixedJoint0.mT_ParentBodyToJoint = auxJoint0.mT_ParentBodyToJoint;
    fixedJoint0.mT_ChildBodyToJoint = auxJoint0.mT_ChildBodyToJoint;

    dart_robotSkeleton->getBodyNode("mobile_manipulator/wheel0_link")->changeParentJointType<WeldJoint>(fixedJoint0);

    /******************************/
    // Change Wheel1 joint to WeldJoint
    dart::dynamics::Joint::Properties auxJoint1 = dart_robotSkeleton->getJoint("wheel1_joint")->getJointProperties();

    dart::dynamics::WeldJoint::Properties fixedJoint1;
    fixedJoint1.mName = auxJoint1.mName;
    fixedJoint1.mT_ParentBodyToJoint = auxJoint1.mT_ParentBodyToJoint;
    fixedJoint1.mT_ChildBodyToJoint = auxJoint1.mT_ChildBodyToJoint;

    dart_robotSkeleton->getBodyNode("mobile_manipulator/wheel1_link")->changeParentJointType<WeldJoint>(fixedJoint1);

    /******************************/
    // Change Wheel2 joint to WeldJoint
    dart::dynamics::Joint::Properties auxJoint2 = dart_robotSkeleton->getJoint("wheel2_joint")->getJointProperties();

    dart::dynamics::WeldJoint::Properties fixedJoint2;
    fixedJoint2.mName = auxJoint2.mName;
    fixedJoint2.mT_ParentBodyToJoint = auxJoint2.mT_ParentBodyToJoint;
    fixedJoint2.mT_ChildBodyToJoint = auxJoint2.mT_ChildBodyToJoint;

    dart_robotSkeleton->getBodyNode("mobile_manipulator/wheel2_link")->changeParentJointType<WeldJoint>(fixedJoint2);

    /******************************/
    // Change root_joint to planar joint
    dart::dynamics::Joint::Properties auxBaseJoint = dart_robotSkeleton->getRootJoint()->getJointProperties();

    dart::dynamics::PlanarJoint::Properties planarJoint;
    std::string jointName = "base_planar_joint";
    planarJoint.mName = jointName;

    planarJoint.setXYPlane();

    auto dofNames = planarJoint.mDofNames;

    planarJoint.mT_ParentBodyToJoint = auxBaseJoint.mT_ParentBodyToJoint;
    planarJoint.mT_ChildBodyToJoint = auxBaseJoint.mT_ChildBodyToJoint;

    dart_robotSkeleton->getRootBodyNode()->changeParentJointType<PlanarJoint>(planarJoint);

    Joint* base_planarJoint = dart_robotSkeleton->getJoint("base_planar_joint");
    base_planarJoint->setActuatorType(Joint::FORCE);

    dart_robotSkeleton->getDof(0)->setName("mobile_base_trans_x");
    dart_robotSkeleton->getDof(1)->setName("mobile_base_trans_y");
    dart_robotSkeleton->getDof(2)->setName("mobile_base_rot_z");

    /******************************/
    // Change Left finger joint to WeldJoint
    dart::dynamics::Joint::Properties auxJoint3 = dart_robotSkeleton->getJoint("left_finger_joint")->getJointProperties();

    dart::dynamics::WeldJoint::Properties fixedJoint3;
    fixedJoint3.mName = auxJoint3.mName;
    fixedJoint3.mT_ParentBodyToJoint = auxJoint3.mT_ParentBodyToJoint;
    fixedJoint3.mT_ChildBodyToJoint = auxJoint3.mT_ChildBodyToJoint;

    dart_robotSkeleton->getBodyNode("mobile_manipulator/left_finger_link")->changeParentJointType<WeldJoint>(fixedJoint3);

    /******************************/
    // Change Right finger joint to WeldJoint
    dart::dynamics::Joint::Properties auxJoint4 = dart_robotSkeleton->getJoint("right_finger_joint")->getJointProperties();

    dart::dynamics::WeldJoint::Properties fixedJoint4;
    fixedJoint4.mName = auxJoint4.mName;
    fixedJoint4.mT_ParentBodyToJoint = auxJoint4.mT_ParentBodyToJoint;
    fixedJoint4.mT_ChildBodyToJoint = auxJoint4.mT_ChildBodyToJoint;

    dart_robotSkeleton->getBodyNode("mobile_manipulator/right_finger_link")->changeParentJointType<WeldJoint>(fixedJoint4);

    /******************************/
    // Change Right finger joint to WeldJoint
    mEndEffector_ = dart_robotSkeleton->getBodyNode("mobile_manipulator/tcp_gripper_link");

    /******************************/
    // Add properties to the DART Model
    dart_robotSkeleton->setMobile(false);
    //dart_robotSkeleton->enableSelfCollisionCheck();

    /******************************/
    // Update number of dofs of the robot
    robot_dofs = mEndEffector_->getNumDependentGenCoords(); 

    q_k = Eigen::VectorXd::Zero(robot_dofs);
    q_dot_k = Eigen::VectorXd::Zero(robot_dofs);
    tau_zero = Eigen::VectorXd::Zero(robot_dofs);
    tau_result = Eigen::VectorXd::Zero(robot_dofs);
    tau_joints = Eigen::VectorXd::Zero(robot_dofs);
    q_dot_result = Eigen::VectorXd::Zero(robot_dofs);
    q_dot_zero = Eigen::VectorXd::Zero(robot_dofs);

    /******************************/
    // INIT q_dot and tau to zero

    dart_robotSkeleton->setMobile(false);
    dart_robotSkeleton->setPositions(q_k);
    dart_robotSkeleton->setVelocities(q_dot_k);

    // Print Message in log
    ROS_INFO("Done loading model");

}

} /* namespace */