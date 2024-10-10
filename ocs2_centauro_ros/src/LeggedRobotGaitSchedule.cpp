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
double D_s1, D_s2;
double L_s = 0.175;
double delta = 0.0;

bool lock_mode = false;
int mode_index = 0;

int map_index = 0;

// Callback function to process the received Bool message
void elevationMapCallback(const grid_map_msgs::GridMap& msg)
{
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);
  gait_mode = 1 ;  
  std::cout << "......elevationMapCallback......." << std::endl;
  std::cout << "Length: " << map.getLength() << std::endl;
  if (!map.exists("elevation")) {
    ROS_WARN("Elevation layer not found in the map.");
    return;
  }
  int cols = 0;
  int rows = 0;


  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    const float elevation = map.at("elevation", *iterator);

    grid_map::Position position;
    map.getPosition(*iterator, position);

    double elevation_prin = elevation;
    elevation_prin = elevation_prin + 0.8 ;  
    

    std::cout << "map_index = " << map_index << std::endl;
    map_index++;
    std::cout << "(" << position.x() << "," << position.y() << "," << elevation <<")" <<  " ";
    // if ( map_index % 10 == 0 ){
    //   rows ++ ;
    //   std::cout << "map_index = " << map_index << std::endl;
    //   std::cout << "The " << rows << " th row" << std::endl;
    // }
    // for (size_t i = 0; i < 1; i++)
    // {
    //   std::cout << "(" << position.x() << "," << position.y() << "," << elevation <<")" <<  " ";
    // }
    // std::cout << std::endl;
    // map_index ++;

    // std::cout << "resolution: " << map.getResolution() << std::endl;  
    // std::cout << "length: " << map.getLength() << std::endl;  
    // std::cout << "size: " << map.getSize() << std::endl;


    
    // if ( elevation_prin > 0.08 && elevation_prin < 0.09 )
    // {
    //   // std::cout << "|--- x=" << position.x() << " y=" << position.y() << " z=" << elevation_prin << " --|" << std::endl; 
    //   // std::cout << "D_s1 = " << D_s1 << std::endl;
    //   if (position.x() != 0)
    //   {
    //     D_s1 = position.x();
    //   }
      
      
    //   // if (D_s1 > 1.35 || D_s1 < 1.35)
    //   //   D_s1 = 1.35;
    // }

    // if ( elevation_prin > 0.13 && elevation_prin < 0.15 )
    // {
    //   // std::cout << "|--- x=" << position.x() << " y=" << position.y() << " z=" << elevation_prin << " --|" << std::endl; 
    //   // std::cout << "D_s2 = " << D_s2 << std::endl;
    //   if (position.x() != 0)
    //   {
    //     D_s2 = position.x();
    //   }
    // }
    // double foot_xoffset = 0.347;
    // double D_stage1 = D_s1 - foot_xoffset;
    // double D_stage2 = D_s2 - foot_xoffset; 
    // std::cout << "D_stage1 = " << D_stage1 << std::endl;
    // std::cout << "D_stage2 = " << D_stage2 << std::endl;
    // std::cout << "D_s1 = " << D_s1 << std::endl;
    // std::cout << "D_s2 = " << D_s2 << std::endl;
    // std::cout << "position.x() = " << position.x() << std::endl;
    // if (position.x() > 0.13 + foot_xoffset && position.x() < 0.16 + foot_xoffset)
    //   std::cout << "elevation_prin = " << elevation_prin << std::endl;
  }
  std::cout << "print over" << std::endl;
  /** Print */
  // std::cout << "----   elevation map index = " << map_index << std::endl; // 3600;

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

  //   std::vector<size_t> modeSequence;
  //   /** stance */ 
  //   std::vector<std::string> modeSequenceString_stance = {
  //       "STANCE" 
  //   };
  //   std::vector<scalar_t> switchingTimes_stance = {
  //       0.0,  
  //       0.5
  //   };
  //   modeSequence.reserve(modeSequenceString_stance.size());
  //   for (const auto& modeName : modeSequenceString_stance) {
  //     modeSequence.push_back(string2ModeNumber(modeName));
  //   }
  //   ModeSequenceTemplate modeSequenceTemplate_stance(switchingTimes_stance, modeSequence);

  //   /** walk */ 
  //   std::vector<std::string> modeSequenceString_walk = {
  //       "LF_LH_RH",  // 
  //       "STANCE",  // 
  //       "RF_LH_RH",  // 
  //       "STANCE",  // 
  //       "LF_RF_RH",  // 
  //       "STANCE",  // 
  //       "LF_RF_LH",  // 
  //       "STANCE"   //  
  //   };
  //   std::vector<scalar_t> switchingTimes_walk = {
  //       0.0,  // Time for LF_LH_RH 1
  //       0.6,  // Time for STANCE 2
  //       1.2,  // Time for RF_LH_RH 3
  //       1.8,  // Time for STANCE 4
  //       2.4,  // Time for LF_RF_RH 5
  //       3.0,  // Time for STANCE 6
  //       3.6,  // Time for LF_RF_LH 7
  //       4.2,  // Time for STANCE 8
  //       4.8,  // Time for LF_LH_RH 9
  //   };
  //   modeSequence.reserve(modeSequenceString_walk.size());
  //   for (const auto& modeName : modeSequenceString_walk) {
  //     modeSequence.push_back(string2ModeNumber(modeName));
  //   }
  //   ModeSequenceTemplate modeSequenceTemplate_walk(switchingTimes_walk, modeSequence);

  //   /** walk more*/ 
  //   std::vector<std::string> modeSequenceString_walk_more = {
  //       "LF_LH_RH",  // 
  //       "STANCE",  // 
  //       "RF_LH_RH",  // 
  //       "STANCE",  // 

  //       "LF_LH_RH",  // 
  //       "STANCE",  // 
  //       "RF_LH_RH",  // 
  //       "STANCE",   //  

  //       "LF_RF_RH",  // 
  //       "STANCE",  // 
  //       "LF_RF_LH",  // 
  //       "STANCE",   // 
  //   };
  //   std::vector<scalar_t> switchingTimes_walk_more = {
  //       0.0,  // Time for LF_LH_RH 1
  //       0.6,  // Time for STANCE 2
  //       1.2,  // Time for RF_LH_RH 3
  //       1.8,  // Time for STANCE 4
  //       2.4,  // Time for LF_RF_RH 5
  //       3.0,  // Time for STANCE 6
  //       3.6,  // Time for LF_RF_LH 7
  //       4.2,  // Time for STANCE 8
  //       4.8,  // Time for LF_LH_RH 9
  //       5.4, 
  //       6.0,
  //       6.0,
  //       7.2
  //   };
  
  // modeSequence.reserve(modeSequenceString_walk_more.size());
  // for (const auto& modeName : modeSequenceString_walk_more) {
  //   modeSequence.push_back(string2ModeNumber(modeName));
  // }
  // ModeSequenceTemplate modeSequenceTemplate_walk_more(switchingTimes_walk_more, modeSequence);
  // /********************/



  ros::Rate loop_rate(10);
  lock_mode = true;

  //the location of the frame wheel_2 in the frame pelvis : [0.347, -0.347, -0.686]
  
  

   while (ros::ok())
  {
  //   // calculate the distance between the stage1, the stage2 and the RH foot
  //   //// the second stage
  //   double D_stage1 = D_s1 - foot_xoffset;
  //   double D_stage2 = D_s2 - foot_xoffset; 
  //   std::cout << "D_stage1 = " << D_stage1 << std::endl;
  //   std::cout << "D_stage2 = " << D_stage2 << std::endl;
  //   std::cout << "L_s = " << L_s << std::endl;
  //   // stability margin
  //   double margin_stability = 0;
  //   /// margin_stability = L_s - (D_stage2 - round(D_stage1/L_s) * L_s) < \delta ,\delta = 2/3(D_s2 - D_s1)
  //   double margin_stability1 = round(D_stage1/L_s);



  //   float a = 3;
  //   float b = 2;
  //   delta = float(b/a) * (D_s2 - D_s1);
  //   int temp = (D_s1/L_s);
  //   // std::cout << "temp = " << temp << std::endl;
  //   double stability_margin = L_s - (D_s1 - temp * L_s);
  //   // std::cout << "stability_margin = " << stability_margin << std::endl;
  //   // std::cout << "delta = " << delta << std::endl;
    


  //   // if (stability_margin < delta){
  //   //   mode_index = 1;
  //   //   if (D_s1 < 0.05 && mode_index == 1)
  //   //   {
  //   //     modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate_stance));
  //   //     loop_rate.sleep(); 
  //   //     mode_index == 2;
  //   //   }
  //   //   if (mode_index == 2)
  //   //     modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate_walk_more));
  //   // }
      

  //   // std::cout << "(D_s1/L_s) = " << (D_s1/L_s) << std::endl;
  //   // std::cout << "(D_s2/L_s) * L_s = " << (D_s2/L_s) * L_s << std::endl;

  //   // lock_mode = false;
    
  //   // loop_rate.sleep(); ros::spin();
  ros::spin();
  }
  
  return 0;
}

