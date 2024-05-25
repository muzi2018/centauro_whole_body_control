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



int main(int argc, char* argv[]) {
  const std::string robotName = "legged_robot";
  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;

  std::shared_ptr<SwitchedStateReferenceManager> referenceManagerPtr_;
  // ReferenceManagerInterface
//   auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, referenceManagerPtr_);
//   rosReferenceManagerPtr->subscribe(nodeHandle, targetFramesNames);

  // Subscribe TargetTrajectories
  auto targetTrajectoriesCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg) {
    auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
    referenceManagerPtr_->setTargetTrajectories(std::move(targetTrajectories));
  };
//   referenceManagerPtr_->modifyReferences
  targetTrajectoriesSubscriber_ =
      nodeHandle.subscribe<ocs2_msgs::mpc_target_trajectories>(robotName + "_mpc_target", 1, targetTrajectoriesCallback);
  ros::Rate r(10);
  while (ros::ok() && ros::master::check()) {
      {
        ros::spinOnce();
        r.sleep();
      }
  }
  return 0;
}

