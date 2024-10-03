#include <ros/init.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include "ocs2_centauro_ros/gait/GaitKeyboardPublisher.h"

#include "grid_map_msgs/GridMap.h" 
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <grid_map_ros/grid_map_ros.hpp>


using namespace ocs2;
using namespace legged_robot;

// Callback function to process the received Bool message
void elevationMapCallback(const grid_map_msgs::GridMap& msg)
{
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);

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
  ros::spin();

  // Successful exit
  return 0;
}

