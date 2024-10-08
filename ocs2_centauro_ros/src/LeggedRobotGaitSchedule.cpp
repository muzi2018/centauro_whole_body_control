#include <ros/init.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include "ocs2_centauro_ros/gait/GaitKeyboardPublisher.h"

#include "grid_map_msgs/GridMap.h" 
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include "ocs2_centauro_ros/gait/ModeSequenceTemplateRos.h"


using namespace ocs2;
using namespace legged_robot;
int gait_mode = 0;
// Callback function to process the received Bool message
void elevationMapCallback(const grid_map_msgs::GridMap& msg)
{
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);
  gait_mode = 1 ;  
  std::cout << "elevationMapCallback......." << std::endl;

  if (!map.exists("elevation")) {
    ROS_WARN("Elevation layer not found in the map.");
    return;
  }

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    const float elevation = map.at("elevation", *iterator);

    grid_map::Position position;
    map.getPosition(*iterator, position);

    double elevation_prin = elevation;
    elevation_prin = elevation_prin + 0.8 ;    
    if ( elevation_prin > 0.08 && elevation_prin < 0.1)
    {
      std::cout << "|--- x=" << position.x() << " y=" << position.y() << " z=" << elevation_prin << " --|" << std::endl; 
      // ROS_INFO("Position (%f, %f): Elevation %f", position.x(), position.y(), elevation);
    }
  }
}


int main(int argc, char* argv[]) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc_gait_schedule");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string gaitCommandFile;
  nodeHandle.getParam("/gaitCommandFile", gaitCommandFile);
  std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;





//   GaitKeyboardPublisher gaitCommand(nodeHandle, gaitCommandFile, robotName, true);
  ros::Subscriber sub = nodeHandle.subscribe("/elevation_mapping/elevation_map", 1, elevationMapCallback);
  
  ros::Publisher modeSequenceTemplatePublisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);

    std::vector<std::string> modeSequenceString = {
        "LF_LH_RH",  // 
        "STANCE",  // 
        "RF_LH_RH",  // 
        "STANCE",  // 
        "LF_RF_RH",  // 
        "STANCE",  // 
        "LF_RF_LH",  // 
        "STANCE"   // 
    };

    std::vector<scalar_t> switchingTimes = {
        0.0,  // Time for LF_LH_RH
        0.6,  // Time for STANCE
        1.2,  // Time for RF_LH_RH
        1.8,  // Time for STANCE
        2.4,  // Time for LF_RF_RH
        3.0,  // Time for STANCE
        3.6,  // Time for LF_RF_LH
        4.2,  // Time for STANCE
        4.8   // Additional time if needed
    };

  // convert the mode name to mode enum
  std::vector<size_t> modeSequence;
  modeSequence.reserve(modeSequenceString.size());
  for (const auto& modeName : modeSequenceString) {
    modeSequence.push_back(string2ModeNumber(modeName));
  }

  ModeSequenceTemplate modeSequenceTemplate(switchingTimes, modeSequence);
  if (1)
  {
    std::cout << "modeSequenceTemplate" << std::endl;
    modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
  }
  ros::spin();
  return 0;
}

