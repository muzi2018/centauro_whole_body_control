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


scalar_t targetDisplacementVelocity;
scalar_t targetRotationVelocity;
scalar_t comHeight;
vector_t defaultJointState;
vector_t desireJointState;
bool iii = true;



/**
 * Send joints reference to TargetTrajectories
*/

TargetTrajectories jointRefToTargetTrajectories(const SystemObservation& observation){
  // std::cout << "*********target1*********" << std::endl;
  // std::cout << "*********observation.state.size*********" << std::endl;
  const vector_t currentPose = observation.state.segment<6>(6);
  // target reaching duration
  const scalar_t targetReachingTime = observation.time + 1 ;
  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};
  // desired state trajectory
  // 6 base , 6 left arm, 6 right arm, 1 grippers + 2 = 13 +2
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, defaultJointState;
  stateTrajectory[1] << vector_t::Zero(6), currentPose, desireJointState;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));
  if ( iii )
  {
    for (size_t i = 0; i < desireJointState.size(); i++)
    {
      std::cout << "desireJointState[" << i << "] = " << desireJointState[i] << std::endl;
    }
    iii = false;
  }
  

  


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

  ::ros::Subscriber observationSubscriber_;
  std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;

  //----------------------------------------------------------------
  // Get the observation topic
  //----------------------------------------------------------------
  auto observationCallback = [&](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };

  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1, observationCallback);

  desireJointState[25] = 0.4; desireJointState[26] = 0.35; desireJointState[27] = 0.25; 
  desireJointState[28] = -2.2; desireJointState[29] = 0.0; desireJointState[30] = -0.7;

  desireJointState[31] = 0.36; desireJointState[32] = -0.73; desireJointState[33] = -0.75; 
  desireJointState[34] = -1.4; desireJointState[35] = -0.24; desireJointState[36] = -0.14;
  while (ros::ok() && ros::master::check()) {
    {
    /* code */

      ::ros::spinOnce();
      
      SystemObservation observation;
      {
        std::lock_guard<std::mutex> lock(latestObservationMutex_);
        observation = latestObservation_;
      }
      // std::cout << "observation.state.size() = " << observation.state.size() << std::endl;

      if (observation.state.size()) {
      {
        jointRefToTargetTrajectories(observation);
      }
    }
  }
  }

/**
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
********************************/

  return 0;
}
