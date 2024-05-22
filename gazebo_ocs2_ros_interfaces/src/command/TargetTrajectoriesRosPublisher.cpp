/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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
******************************************************************************/

#include "gazebo_ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h"

// MPC messages
#include <ocs2_msgs/mpc_target_trajectories.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesRosPublisher::TargetTrajectoriesRosPublisher(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix,
                                                               const std::string& targetFrame) {
  targetTrajectoriesPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_target_trajectories>(topicPrefix + "_mpc_" + targetFrame + "target", 1, false);
  ros::spinOnce();
  ROS_INFO_STREAM("The TargetTrajectories is publishing on " + topicPrefix + "_mpc_" + targetFrame + "_target topic.");
}

TargetTrajectoriesRosPublisher::TargetTrajectoriesRosPublisher(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix,
                                                                bool arm_rl_Ref_ ,jointRefToTargetTrajectories jointRefToTargetTrajectoriesFun )
                                                               :
                                                               arm_rl_Ref(arm_rl_Ref_), 
                                                               jointRefToTargetTrajectoriesFun_(std::move(jointRefToTargetTrajectoriesFun)){
  targetTrajectoriesPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_target_trajectories>(topicPrefix + "_mpc_" + "target", 1, false);
  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

  ros::spinOnce();
  // ROS_INFO_STREAM("The TargetTrajectories is publishing on " + topicPrefix + "_mpc_" + targetFrame + "_target topic.");
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesRosPublisher::~TargetTrajectoriesRosPublisher() {
  targetTrajectoriesPublisher_.shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesRosPublisher::publishTargetTrajectories(const TargetTrajectories& targetTrajectories) {
  const auto mpcTargetTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(targetTrajectories);
  targetTrajectoriesPublisher_.publish(mpcTargetTrajectoriesMsg);
}

void TargetTrajectoriesRosPublisher::publishTargetTrajectories(const TargetTrajectories& targetTrajectories, bool arm_rl_Ref) {
    // get the latest observation
    ::ros::spinOnce();
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    // get TargetTrajectories
    const auto targetTrajectories_ = jointRefToTargetTrajectoriesFun_(observation);

    // publish TargetTrajectories
    this->publishTargetTrajectories(targetTrajectories_);
}



}  // namespace ocs2
