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
#include <ocs2_centauro/common/ModelSettings.h>
#include <ros/service.h>
#include <std_srvs/Empty.h>
#include <xbot_msgs/JointState.h>
#include <fstream>
#include <iostream>
#include <string>
#include <gazebo_ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include "ocs2_centauro/reference_manager/SwitchedStateReferenceManager.h"

using namespace ocs2;
using namespace ocs2::legged_robot;
using namespace std;
std::vector<std::vector<double>> doubleData;
int n_segments = 0;

scalar_t targetDisplacementVelocity;
scalar_t targetRotationVelocity;
scalar_t comHeight;
vector_t defaultJointState;
vector_t desireJointState;
vector_t desirepose;
bool iii = true;

bool arm_rl_bool = false;

std::vector<double> parseDoubles(const std::string& line) {
    std::vector<double> doubles;
    std::istringstream iss(line);
    double value;

    while (iss >> value) {
        doubles.push_back(value);
    }

    return doubles;
}


bool arm_rl_tracking(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    arm_rl_bool = !arm_rl_bool;
    return true;
};

/**
 * Send joints reference to TargetTrajectories
*/

TargetTrajectories jointRefToTargetTrajectories(const SystemObservation& observation){

  const vector_t currentPose = observation.state.segment<6>(6);

  // desired time trajectory
  scalar_array_t timeTrajectory;
  timeTrajectory.resize(doubleData.size());
  int i = 0;
  for (double& timePoint : timeTrajectory) {
      timePoint = observation.time + 0.1 * i;
      i++;
  }

  // desired state trajectory
  // 6 base , 6 left arm, 6 right arm, 1 grippers + 2 = 13 +2
  vector_array_t stateTrajectory(doubleData.size(), vector_t::Zero(observation.state.size()));
  int j = 0;
  for (size_t i = 0; i < doubleData.size(); i++)
  {
    desireJointState = defaultJointState;
    desireJointState[24] = doubleData[j][6];
      //left arm
    desireJointState[25] = doubleData[j][7]; 
    desireJointState[26] = doubleData[j][8]; 
    desireJointState[27] = doubleData[j][9]; 
    desireJointState[28] = doubleData[j][10]; 
    desireJointState[29] = doubleData[j][11]; 
    desireJointState[30] = doubleData[j][12];
    //right arm
    desireJointState[31] = doubleData[j][13]; 
    desireJointState[32] = doubleData[j][14];
    desireJointState[33] = doubleData[j][15];
    desireJointState[34] = doubleData[j][16];
    desireJointState[35] = doubleData[j][17];
    desireJointState[36] = doubleData[j][18];
    /* code */
    stateTrajectory[i] << vector_t::Zero(6), currentPose, desireJointState;
    j++;
  }
  
  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(doubleData.size(), vector_t::Zero(observation.state.size()));
  if ( iii )
  {
    for (size_t i = 0; i < desireJointState.size(); i++)
    {
      std::cout << "desireJointState[" << i << "] = " << desireJointState[i] << std::endl;
    }
    for (size_t i = 0; i < observation.state.size(); i++)
    {
      std::cout << "state[" << i << "] = " << observation.state[i] << std::endl;
    }
    iii = false;
  }

  std::cout << "timeTrajectory.size() = " << timeTrajectory.size() << std::endl;
  std::cout << "stateTrajectory.size() = " << stateTrajectory.size() << std::endl;
  std::cout << "inputTrajectory.size() = " << inputTrajectory.size() << std::endl;
  std::cout << "observation.state.size() = " << observation.state.size() << std::endl;
  std::cout << "desireJointState.size() = " << desireJointState.size() << std::endl;

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

  //----------------------------------------------------------------
  // Publish the target trajectories
  //----------------------------------------------------------------
  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, robotName));  
  ros::ServiceServer service = nodeHandle.advertiseService("arm_rl", arm_rl_tracking);




  ifstream file("/home/wang/catkin_ws_1/src/centauro_whole_body_control/ocs2_centauro_ros/data/output.txt");
  if (!file) {
      cerr << "Unable to open file!" << endl;
  }
  std::vector<std::string> lines;
  std::string line;
  if (file.is_open()) {
      while (std::getline(file, line)) {
          lines.push_back(line);
      }
      file.close();
  } else {
      std::cerr << "Unable to open file" << std::endl;
  }
    std::cout << "lines.size() = " << lines.size() << std::endl; 


  
  for (const auto& line : lines) {
      doubleData.push_back(parseDoubles(line));
  }

  ros::Rate r(10);
  while (ros::ok() && ros::master::check()) {
      {
        while (!arm_rl_bool)
        {
            ros::spinOnce();
            r.sleep();
        }
        SystemObservation observation;
        {
          std::lock_guard<std::mutex> lock(latestObservationMutex_);
          observation = latestObservation_;
        }
      // std::cout << "observation.state.size() = " << observation.state.size() << std::endl;
        if (observation.state.size()) {
        {
          std::cout << "observation.state.size() = " << observation.state.size() << std::endl;
          const auto targetTrajectories = jointRefToTargetTrajectories(observation);
          targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
          arm_rl_bool = !arm_rl_bool;
        }
      }
    }
  }

  r.sleep();

  return 0;
}
