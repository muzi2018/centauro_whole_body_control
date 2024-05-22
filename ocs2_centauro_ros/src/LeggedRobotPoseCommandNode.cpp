/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (c) 2023, Ioannis Dadiotis <ioannis.dadiotis@iit.it>. All rights reserved.
Additional modifications and contributions by Ioannis Dadiotis:
- wheel position reference in state cost (rolling issue)
******************************************************************************/

#include <string>

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <gazebo_ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>
#include  <ocs2_centauro/common/ModelSettings.h>

#include <xbot_msgs/JointState.h>

using namespace ocs2;

namespace {
scalar_t targetDisplacementVelocity;
scalar_t targetRotationVelocity;
scalar_t comHeight;
vector_t defaultJointState;
vector_t desireJointState;
}  // namespace

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / targetDisplacementVelocity;
  return std::max(rotationTime, displacementTime);
}

/**
 * Converts command line to TargetTrajectories.
 * @param [in] commadLineTarget : [deltaX, deltaY, deltaZ, deltaYaw]
 * @param [in] observation : the current observation
 */
TargetTrajectories commandLineToTargetTrajectories(const vector_t& commadLineTarget, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    // base p_x, p_y are relative to current state
    target(0) = currentPose(0) + commadLineTarget(0);
    target(1) = currentPose(1) + commadLineTarget(1);
    // base z relative to the default height
    target(2) = comHeight + commadLineTarget(2);
    // theta_z relative to current
    target(3) = currentPose(3) + commadLineTarget(3) * M_PI / 180.0;
    // theta_y, theta_x
    target(4) = currentPose(4);
    target(5) = currentPose(5);
    return target;
  }();

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};
  // desired state trajectory

  vector_t joint_state;
  joint_state.resize(37);
    for (size_t i = 0; i < joint_state.size(); i++)
  {
    joint_state[i] = defaultJointState[i];
  }
  joint_state[25] = 0.4; joint_state[26] = 0.35; joint_state[27] = 0.25; 
  joint_state[28] = -2.2; joint_state[29] = 0.0; joint_state[30] = -0.7;

  joint_state[31] = 0.36; joint_state[32] = -0.73; joint_state[33] = -0.75; 
  joint_state[34] = -1.4; joint_state[35] = -0.24; joint_state[36] = -0.14;


  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, defaultJointState;
  stateTrajectory[1] << vector_t::Zero(6), currentPose, joint_state;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/**
 * Send joints reference to TargetTrajectories
*/

TargetTrajectories jointRefToTargetTrajectories(const SystemObservation& observation){
      std::cout << "*********target1*********" << std::endl;
      std::cout << "*********observation.state.size*********" << std::endl;
      std::cout << observation.state.size() << std::endl;

  const vector_t currentPose = observation.state.segment<6>(6);

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + 20;

  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, defaultJointState;
  stateTrajectory[1] << vector_t::Zero(6), currentPose, desireJointState;

  std::cout << "desireJointState" << std::endl;
  for (size_t i = 0; i < desireJointState.size(); i++)
  {
    std::cout <<"[" << i << "] = " << desireJointState[i] << std::endl;
  }
  

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};



}


int main(int argc, char* argv[]) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string referenceFile, taskFile;
  nodeHandle.getParam("/referenceFile", referenceFile);

  std::cout << "referenceFile is " << referenceFile << std::endl;

  nodeHandle.getParam("/taskFile", taskFile);
  legged_robot::ModelSettings modelSettings = legged_robot::loadModelSettings(taskFile, "model_settings", false);
  defaultJointState.resize(modelSettings.jointNames.size());    // resize defaultJointState vector
  desireJointState.resize(modelSettings.jointNames.size());    // resize desireJointState vector
  


  loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defaultJointState);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", targetRotationVelocity);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", targetDisplacementVelocity);

  desireJointState = defaultJointState;
  desireJointState[25] = 0.5; desireJointState[26] = 0.4; desireJointState[27] = 0.4; 
  desireJointState[28] = -2.2; desireJointState[29] = 0.1; desireJointState[30] = -0.8;

  desireJointState[31] = 0.6; desireJointState[32] = -0.6; desireJointState[33] = -0.7; 
  desireJointState[34] = -1.3; desireJointState[35] = -0.3; desireJointState[36] = -0.2;
  // get wheels current position and set this as reference of the mpc state cost
  bool xbotCoreRunning;
  loadData::loadCppDataType(taskFile, "xbotcore.xbotCoreRunning", xbotCoreRunning);
  if (xbotCoreRunning) {
      boost::shared_ptr<xbot_msgs::JointState const> msgJointStatesPtr;
      msgJointStatesPtr = ros::topic::waitForMessage<xbot_msgs::JointState>("/xbotcore/joint_states", nodeHandle);
      for (int wheel_i = 1; wheel_i < 5; wheel_i++) {
          auto wheelIndex = std::distance(modelSettings.jointNames.begin(), std::find(modelSettings.jointNames.begin(), modelSettings.jointNames.end(), "j_wheel_" + std::to_string(wheel_i)));
          auto wheelIndexInXbot = std::distance(msgJointStatesPtr->name.begin(), std::find(msgJointStatesPtr->name.begin(), msgJointStatesPtr->name.end(), "j_wheel_" + std::to_string(wheel_i)));
          defaultJointState[wheelIndex] = msgJointStatesPtr->link_position[wheelIndexInXbot];
      }
  }


  // goalPose: [deltaX, deltaY, deltaZ, deltaYaw]
  const scalar_array_t relativeBaseLimit{10.0, 10.0, 1.0, 360.0};
  // TargetTrajectoriesKeyboardPublisher targetPoseCommand(nodeHandle, robotName, relativeBaseLimit, &jointRefToTargetTrajectories);
  TargetTrajectoriesKeyboardPublisher targetPoseCommand(nodeHandle, robotName, relativeBaseLimit, &commandLineToTargetTrajectories);

  const std::string commandMsg = "Enter XYZ and Yaw (deg) displacements for the TORSO, separated by spaces";
  targetPoseCommand.publishKeyboardCommand(commandMsg);

  // // goalJoints
  // TargetTrajectoriesRosPublisher targetJointCommand(nodeHandle, robotName, true, &jointRefToTargetTrajectories);
  // targetJointCommand.publishTargetTrajectories(true);

  // Successful exit
  return 0;
}
