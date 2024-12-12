/******************************************************************************
Copyright (c) 2023, Ioannis Dadiotis <ioannis.dadiotis@iit.it>. All rights reserved.

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

#include "ocs2_centauro/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>

#include "ocs2_centauro/gait/MotionPhaseDefinition.h"

#include <numeric>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwingTrajectoryPlanner::SwingTrajectoryPlanner(Config config, size_t numFeet) : config_(std::move(config)), numFeet_(numFeet) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);


  // // Print feetHeightTrajectoriesEvents_
  // if (time < 0.02)
  // {
  //   std::cout << "time: " << time << std::endl;
  //   std::cout << "feetHeightTrajectoriesEvents_: " << std::endl;
  //   for (size_t i = 0; i < feetHeightTrajectoriesEvents_.size(); i++) {
  //       std::cout << "Foot " << i << ": ";
  //       for (size_t j = 0; j < feetHeightTrajectoriesEvents_[i].size(); j++) {
  //           std::cout << feetHeightTrajectoriesEvents_[i][j] << " ";
  //       }
  //       std::cout << std::endl;
  //   }
  // }
  


  return feetHeightTrajectories_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const {
  // std::cout << "IIT: time = " << time << std::endl;
  // std::cout << "IIT: leg = " << leg << std::endl; 
  // feet_array_t<std::vector<scalar_t>> feetHeightTrajectoriesEvents_;

  // std::cout << "IIT: feetHeight in " << time <<std::endl;
  // for (size_t i = 0; i < feetHeightTrajectoriesEvents_.size(); i++) {
  //     std::cout << "Foot " << i << ": ";
  //     for (size_t j = 0; j < feetHeightTrajectoriesEvents_[i].size(); j++) {
  //         std::cout << feetHeightTrajectoriesEvents_[i][j] << " ";
  //     }
  //     std::cout << std::endl;
  // }


  // std::cout << "leg : " << leg << std::endl;
  // for (size_t i = 0; i < feetHeightTrajectoriesEvents_[leg].size(); i++)
  // {
  //   std::cout << feetHeightTrajectoriesEvents_[leg][i] << " ";
  // }
  // std::cout << std::endl;
  
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  // std::cout << "index : " << index << std::endl;
  return feetHeightTrajectories_[leg][index].position(time);
}

/******************************************************************************************************/
/* Constraint in X directionL feetHeightTrajecotiresEvents_ are used to get index of feet             */
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getXvelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetLongitTrajectories_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getXpositionConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetLongitTrajectories_[leg][index].position(time);
}

/******************************************************************************************************/
/* Constraint in Y directionL feetHeightTrajecotiresEvents_ are used to get index of feet             */
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getYvelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetLateralTrajectories_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getYpositionConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetLateralTrajectories_[leg][index].position(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, scalar_t initTime, scalar_t terrainHeight, feet_array_t<scalar_array_t> currentEePosition, const TargetTrajectories& targetTrajectories, const vector_t& state) {
  const scalar_array_t terrainHeightSequence(modeSchedule.modeSequence.size(), terrainHeight);
  // feet_array_t = std::array<T, 4>;
  feet_array_t<scalar_array_t> liftOffHeightSequence;
  liftOffHeightSequence.fill(terrainHeightSequence);
  feet_array_t<scalar_array_t> touchDownHeightSequence;
  touchDownHeightSequence.fill(terrainHeightSequence);
  // std::cout << "liftOffHeightSequence first two rows: " << std::endl;
  // for (size_t i = 0; i < touchDownHeightSequence[0].size(); i++){
  //   // std::cout << liftOffHeightSequence[0][i] << " " << liftOffHeightSequence[1][i] << std::endl;
  //   touchDownHeightSequence[0][i] = touchDownHeightSequence[0][i] + 0.08;
  //   touchDownHeightSequence[1][i] = liftOffHeightSequence[1][i] + 0.08;
  // }
  

  auto target_position = targetTrajectories.getDesiredState(initTime);

  // std::cout << "---- [SwingTrajectoryPlanner start x target] ----" << std::endl;
  scalar_t x_e = target_position[6] - state[6] ;
  // std::cout << x_e << std::endl;
  // std::cout << target_position[6] << std::endl;
  // std::cout << state[6] << std::endl;
  // std::cout << "---- [SwingTrajectoryPlanner end] ----" << std::endl;
  

  // Get step length for generating steps
  const auto longStepLength = this->getConfig().longStepLength;
  const auto lateralStepLength = this->getConfig().lateralStepLength;
  
  feet_array_t<scalar_array_t> targetEePosition = currentEePosition;




  // std::cout << "longStepLength = " << longStepLength << std::endl;
  // std::cout << "lateralStepLength = " << lateralStepLength << std::endl;
  // std::cout << "targetEePosition = " << std::endl;
  // for (size_t i = 0; i < targetEePosition.size(); i++)
  // {
  //   for (size_t j = 0; j < targetEePosition[i].size(); j++)
  //   {
  //     std::cout << "targetEePosition[" << i << "][" << j << "]=" << targetEePosition[i][j] << std::endl;
  //   }
  // }


  // Find mode at initTime
  size_t initMode = modeSchedule.modeAtTime(initTime);
  // std::cout << "initTime = " << initTime << std::endl;
  // std::cout << "initMode = " << initMode << std::endl;

  contact_flag_t initModeLegContactFlags = modeNumber2StanceLeg(initMode);      // Contact flags at initMode
  // if (initMode == 7){
  //   std::cout << "initModeLegContactFlags " << std::endl;
  //   std::cout << initModeLegContactFlags[0] << " " << initModeLegContactFlags[1] << " " << initModeLegContactFlags[2] << " " << initModeLegContactFlags[3];
  // }
  // std::cout << std::endl;
  int initModeContactFlagsSum = std::accumulate(initModeLegContactFlags.begin(), initModeLegContactFlags.end(), 0);     // sum contact flags
  // std::cout << "initModeContactFlagsSum = " << initModeContactFlagsSum << std::endl;
  // for (size_t i = 0; i < initModeLegContactFlags.size(); i++)
  // {
  //   std::cout << "initModeLegContactFlags[" << i << "]" <<" = " << initModeLegContactFlags[i] << std::endl;
  // }
  
  // If at initTime there is at least one leg swinging
  // std::cout << "initTime = " << initTime << std::endl;
  if (initModeContactFlagsSum < 4) {
  //  std::cout << "[Yiannis] ********************************** " << std::endl;
  //  std::cout << "[Yiannis] Non-STANCE mode is: " << modeNumber2String(initMode) << std::endl;

    // Find initial mode index within the modes vector
    auto currentModeIndex = std::distance(modeSchedule.modeSequence.begin(), std::find(modeSchedule.modeSequence.begin(),
                                                                                     modeSchedule.modeSequence.end(),
                                                                                     modeSchedule.modeAtTime(initTime)));
    // if (initMode == 7){
    //    std::cout << "[Yiannis] currentModeIndex: " << currentModeIndex << std::endl;
    // }                                                                                 
  

    // Id of the current swing leg
    int CurrentSwingLegId = std::distance(initModeLegContactFlags.begin(),
                                          std::find(initModeLegContactFlags.begin(), initModeLegContactFlags.end(), false));
    // if (initMode == 7){
    //    std::cout << "[Yiannis] CurrentSwingLegId is: " << CurrentSwingLegId << std::endl;
    // }    

    // contact flags for the current swing leg (for all modes)
    const auto CurrentSwingLegContactFlags = extractContactFlags(modeSchedule.modeSequence)[CurrentSwingLegId];

    // if (initMode == 7){
    //   std::cout << "[Yiannis] modeSchedule.modeSequence.size: " << modeSchedule.modeSequence.size() << std::endl;
    //    std::cout << "[Yiannis] CurrentSwingLeg ContactFlags: " << CurrentSwingLegContactFlags.size() << std::endl;
    //    for (int iter = 0; iter < CurrentSwingLegContactFlags.size(); iter++)
    //        std::cout << CurrentSwingLegContactFlags[iter] << " ";
    //    std::cout << " " << std::endl;
    // }
  


    // find the startTime and finalTime indices (e.g. eventTimes[startTimeIndex]) for swing lift off and touch down times
    int startTimeIndex = 0;
    int finalTimeIndex = 0;
    std::tie(startTimeIndex, finalTimeIndex) = findIndex(currentModeIndex, CurrentSwingLegContactFlags);
  //  std::cout << "[Yiannis] startTimeIndex: " << startTimeIndex << std::endl;
  //  std::cout << "[Yiannis] finalTimeIndex: " << finalTimeIndex << std::endl;

    // get lift off time and position before the ongoing swing
    scalar_t startTime = modeSchedule.eventTimes[startTimeIndex];   // ok
    // std::cout << "startTime: " << startTime << std::endl;
    scalar_t lastStanceLongPosition = feetLongitTrajectories_[CurrentSwingLegId][startTimeIndex].position(startTime);   // seems ok
    // std::cout << "feetLongitTrajectories_ " << std::endl;
    // std::cout << feetLongitTrajectories_[0].size() << std::endl;
    scalar_t lastStanceLateralPosition = feetLateralTrajectories_[CurrentSwingLegId][startTimeIndex].position(startTime);   // seems ok

//    std::cout << "[Yiannis] startSwingTime: " << startTime << std::endl;
//    std::cout << "[Yiannis] lastStanceLongPosition: " << lastStanceLongPosition << std::endl;
//    std::cout << "[Yiannis] lastStanceLateralPosition: " << lastStanceLateralPosition << std::endl;

    // modify currentEePosition to be the lastStance since this is the way splines are generated
    currentEePosition[CurrentSwingLegId].at(0) = lastStanceLongPosition;
    targetEePosition[CurrentSwingLegId].at(0) = lastStanceLongPosition;

    currentEePosition[CurrentSwingLegId].at(1) = lastStanceLateralPosition;    // lateral coordinates
    targetEePosition[CurrentSwingLegId].at(1) = lastStanceLateralPosition;
  }

  // augment target position with longStepLength
  for (int i = 0; i < targetEePosition.size(); i++) {
    targetEePosition[i].at(0) += longStepLength ;    // augment by longitudinal step
    targetEePosition[i].at(1) += lateralStepLength;    // augment by longitudinal step

  }

  // get sequences from modes for each leg based if it is a swing mode or a stance one
  std::array<feet_array_t<scalar_array_t>, 4> liftOffTouchDownSeqs = getSequenceFromMode(currentEePosition, targetEePosition, modeSchedule);
  // std::cout << "liftOffLongitudinalSequence " << liftOffTouchDownSeqs[0].size() << std::endl;
  // std::cout << "- targetEePosition -" << std::endl;
  // for (size_t i = 0; i < targetEePosition.size(); i++)
  // {
  //   std::cout << i << "th foot targetEePosition" << std::endl;
  //   std::cout << "x = " << targetEePosition[i][0] << "; y = " << targetEePosition[i][1] << "; z = " << targetEePosition[i][2]  << std::endl;   
  // }
  


  feet_array_t<scalar_array_t> liftOffLongitudinalSequence = liftOffTouchDownSeqs[0];
  // std::cout << "liftOffLongitudinalSequence = " << liftOffLongitudinalSequence[0].size() << std::endl;
  feet_array_t<scalar_array_t> touchDownLongitudinalSequence = liftOffTouchDownSeqs[1];
  feet_array_t<scalar_array_t> liftOffLateralSequence = liftOffTouchDownSeqs[2];
  feet_array_t<scalar_array_t> touchDownLateralSequence = liftOffTouchDownSeqs[3];

  update(modeSchedule,  liftOffHeightSequence, 
                        touchDownHeightSequence, 

                        liftOffLongitudinalSequence, 
                        touchDownLongitudinalSequence,
                              
                        liftOffLateralSequence, 
                        touchDownLateralSequence);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::array<feet_array_t<scalar_array_t>, 4> SwingTrajectoryPlanner::getSequenceFromMode(
        feet_array_t<scalar_array_t> initialEePosition, feet_array_t<scalar_array_t> targetEePosition,
        const ModeSchedule& modeSchedule) const
{
    // create lifOffSequence from initialEePosition
    // TODO: add for loop below
    feet_array_t<scalar_array_t> liftOffLongitudinalSequence = {        // unti now only longitudinal
        scalar_array_t(modeSchedule.modeSequence.size(), initialEePosition[0].at(0)),
        scalar_array_t(modeSchedule.modeSequence.size(), initialEePosition[1].at(0)),
        scalar_array_t(modeSchedule.modeSequence.size(), initialEePosition[2].at(0)),
        scalar_array_t(modeSchedule.modeSequence.size(), initialEePosition[3].at(0))};

    feet_array_t<scalar_array_t> liftOffLateralSequence = {        // lateral
        scalar_array_t(modeSchedule.modeSequence.size(), initialEePosition[0].at(1)),
        scalar_array_t(modeSchedule.modeSequence.size(), initialEePosition[1].at(1)),
        scalar_array_t(modeSchedule.modeSequence.size(), initialEePosition[2].at(1)),
        scalar_array_t(modeSchedule.modeSequence.size(), initialEePosition[3].at(1))};

    // Initialize touchDownSequence
    feet_array_t<scalar_array_t> touchDownLongitudinalSequence = liftOffLongitudinalSequence;
    feet_array_t<scalar_array_t> touchDownLateralSequence = liftOffLateralSequence;

    // Modify touchDownSequence
    feet_array_t<std::vector<bool>> contactFlags = extractContactFlags(modeSchedule.modeSequence);
    // std::cout << std::endl;
    // std::cout << "- contact flags -" << std::endl;
    // for (int i = 0; i < contactFlags.size(); i++) {
    //   std::cout << "Leg " << i << " contact flags: ";
    //   for (bool flag : contactFlags[i]) {
    //       std::cout << flag << " ";
    //   }
    //   std::cout << std::endl;
    // }


    for (int i = 0; i < contactFlags.size(); i++) { //foot number
        for (int j = 0; j < contactFlags[i].size(); j++) { //phase number
            if (contactFlags[i].at(j) == false)
                touchDownLongitudinalSequence[i].at(j) = targetEePosition[i].at(0);
                touchDownLateralSequence[i].at(j) = targetEePosition[i].at(1);
        }
    }

    // std::cout << std::endl;
    // int phase_num = 0;
    // for (int i = 0; i < touchDownLateralSequence.size(); i++) {
    //   std::cout << "Leg " << i << " touchDownLateralSequence: ";
    //   for (size_t j = 0; j < touchDownLateralSequence[i].size(); j++)
    //   {
    //    std::cout << j << "th phase: " << touchDownLateralSequence[i][j] << " " ;   
    //   }
    //   std::cout << std::endl;
    // }    


   // Different way of getting contact flags
//    for (int i=0; i < modeSchedule.modeSequence.size(); i++) {
//        contact_flag_t contact_flags = modeNumber2StanceLeg(modeSchedule.modeSequence[i]);

//        for (int j=0; j < 4; j++) {
////            std::cout << contact_flags[j] << std::endl;
//            if (contact_flags[j] == false) {
//                touchDownLongitudinalSequence[j].at(i) = targetEePosition[j].at(0);
//            }
//        }
//    }

    // Wrap in an array both sequences
    std::array<feet_array_t<scalar_array_t>, 4> liftOffTouchDownSeqs = {liftOffLongitudinalSequence, touchDownLongitudinalSequence,
                                                                        liftOffLateralSequence, touchDownLateralSequence};

//    // debug FL leg mode lift off and touch down planning
//    for (int i = 0; i < modeSchedule.modeSequence.size(); i++) {
//        std::cout << "Mode " << i << std::endl;
//        std::cout << "lifoff = " << liftOffTouchDownSeqs[0][0][i] << ", " << liftOffTouchDownSeqs[2][0][i] << std::endl;;
//        std::cout << "touchDown = " << liftOffTouchDownSeqs[1][0][i] << ", " << liftOffTouchDownSeqs[3][0][i] << std::endl;;
//        std::cout << " " << std::endl;
//    }

    return liftOffTouchDownSeqs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, 
                                    const feet_array_t<scalar_array_t>& liftOffHeightSequence,
                                    const feet_array_t<scalar_array_t>& touchDownHeightSequence,

                                    const feet_array_t<scalar_array_t>& liftOffLongSequence,
                                    const feet_array_t<scalar_array_t>& touchDownLongSequence,

                                    const feet_array_t<scalar_array_t>& liftOffLateralSequence,
                                    const feet_array_t<scalar_array_t>& touchDownLateralSequence) {
  
  // std::cout << "IIT swinglwg " << std::endl;
  const auto& modeSequence = modeSchedule.modeSequence;
  // std::cout << "modeSequence has size " << modeSequence.size() << std::endl;

  // for (size_t i = 0; i < modeSequence.size(); i++)
  // {
  //   std::cout << "modeSequence [" << i << "]: " << modeSequence[i] << std::endl;
  // }
  

  // std::cout << std::endl;

  const auto& eventTimes = modeSchedule.eventTimes;
  //   std::cout << "eventTimes: ";
  //   for (const auto& eventTimes_ : modeSchedule.eventTimes) {
  //       std::cout << eventTimes_ << " ";
  //   }
  //   std::cout << std::endl;

  const auto eesContactFlagStocks = extractContactFlags(modeSequence);
  // std::cout << "eesContactFlagStocks: ";
  // int i_ = 1;
  // for (const auto& eesContactFlagStocks_ : eesContactFlagStocks) {
  //     for (const auto& eesContact : eesContactFlagStocks_) {
  //         std::cout << "foot " << i_ << "= " << eesContact << " ";
  //     }
  //     i_++;
  // }
  // std::cout << std::endl;

  feet_array_t<std::vector<int>> startTimesIndices;
  feet_array_t<std::vector<int>> finalTimesIndices;
  for (size_t leg = 0; leg < numFeet_; leg++) {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
  }

  // std::cout << "startTimesIndices: ";
  // int i_ = 1;
  // for (const auto& startTimesIndices_ : startTimesIndices) {
  //     for (const auto& startTimesIndices__ : startTimesIndices_) {
  //         std::cout << "foot " << i_ << "= " << startTimesIndices__ << " ";
  //     }
  //     i_++;
  // }
  // std::cout << std::endl;


  // std::cout << "finalTimesIndices: ";
  // int i__ = 1;
  // for (const auto& finalTimesIndices_ : finalTimesIndices) {
  //     for (const auto& finalTimesIndices__ : finalTimesIndices_) {
  //         std::cout << "foot " << i__ << "= " << finalTimesIndices__ << " ";
  //     }
  //     i__++;
  // }
  // std::cout << std::endl;


//    std::cout << "[SwingTrajectoryPlanner::update] event times: ";
//    for (int i = 0; i < modeSchedule.eventTimes.size(); i++) {
//        std::cout << " " << modeSchedule.eventTimes[i];
//    }
//    std::cout << std::endl;
  // which mode is swing for feet
  for (size_t j = 0; j < numFeet_; j++) {   // loop over end effectors
    feetHeightTrajectories_[j].clear();
    feetHeightTrajectories_[j].reserve(modeSequence.size());
    feetLongitTrajectories_[j].clear();
    feetLongitTrajectories_[j].reserve(modeSequence.size());
    feetLateralTrajectories_[j].clear();
    feetLateralTrajectories_[j].reserve(modeSequence.size());

//    std::cout << "[SwingTrajectoryPlanner::update] eesContactFlagStocks: ";
//    for (int i = 0; i < eesContactFlagStocks.size(); i++) {
//        std::cout << " " << eesContactFlagStocks[j][i];
//    }
//    std::cout << std::endl;

    for (int p = 0; p < modeSequence.size(); ++p) {    // loop over mode sequence of the end effector j
      if (!eesContactFlagStocks[j][p]) {  // if end effector is a swing leg at current mode
        const int swingStartIndex = startTimesIndices[j][p];
        const int swingFinalIndex = finalTimesIndices[j][p];
        checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

        const scalar_t swingStartTime = eventTimes[swingStartIndex];
        const scalar_t swingFinalTime = eventTimes[swingFinalIndex];
//        std::cout << "[SwingTrajectoryPlanner::update] swingStartIndex = " << swingStartIndex << std::endl;
//        std::cout << "[SwingTrajectoryPlanner::update] swingFinalIndex = " << swingFinalIndex << std::endl;
      //  std::cout << std::endl;
      //  std::cout << "[SwingTrajectoryPlanner::update] swingStartTime = " << swingStartTime << std::endl;
      //  std::cout << "[SwingTrajectoryPlanner::update] swingFinalTime = " << swingFinalTime << std::endl;
      //  std::cout << std::endl;

      // **********  high
        const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);
        // std::cout << "[SwingTrajectoryPlanner::update] scaling = " << scaling << std::endl;
        const CubicSpline::Node liftOff{swingStartTime, liftOffHeightSequence[j][p], scaling * config_.liftOffVelocity};
        const CubicSpline::Node touchDown{swingFinalTime, touchDownHeightSequence[j][p], scaling * config_.touchDownVelocity};
        const scalar_t midHeight = std::min(liftOffHeightSequence[j][p], touchDownHeightSequence[j][p]) + scaling * config_.swingHeight;
        // std::cout << "[SwingTrajectoryPlanner::update] = " << liftOffHeightSequence[j][p] << std::endl;




        const CubicSpline::Node midSwingVertical{(liftOff.time + touchDown.time) / 2, midHeight, 3 * (touchDown.position - liftOff.position) / (touchDown.time - liftOff.time)};
        // std::cout << "liftOffLong.time = " << liftOff.time << std::endl;
        // std::cout << "touchDownLong.time = " << touchDown.time << std::endl;

        // **********  longitudinal
        const CubicSpline::Node liftOffLong{swingStartTime, liftOffLongSequence[j][p], scaling * config_.liftOffLongVelocity};
        const CubicSpline::Node touchDownLong{swingFinalTime, touchDownLongSequence[j][p], scaling * config_.touchDownLongVelocity};
        // std::cout << std::endl;
        // std::cout << "touchDownLongSequence " << j << "th foot in " << p << "th phase = " << std::endl;  
        // std::cout <<touchDownLongSequence[j][p] << std::endl;
        // std::cout << std::endl;

        const scalar_t midLongStep = liftOffLongSequence[j][p] + scaling * 0.5 * config_.longStepLength;
        const CubicSpline::Node midSwingLong{(liftOffLong.time + touchDownLong.time) / 2, midLongStep, 3 * (touchDownLong.position - liftOffLong.position) / (touchDownLong.time - liftOffLong.time)};

        // ************* lateral
        const CubicSpline::Node liftOffLateral{swingStartTime, liftOffLateralSequence[j][p], scaling * config_.liftOffLateralVelocity};
        const CubicSpline::Node touchDownLateral{swingFinalTime, touchDownLateralSequence[j][p], scaling * config_.touchDownLateralVelocity};
        const scalar_t midLateralStep = liftOffLateralSequence[j][p] + scaling * 0.5 * config_.lateralStepLength;
        const CubicSpline::Node midSwingLateral{(liftOffLateral.time + touchDownLateral.time) / 2, midLateralStep, 3 * (touchDownLateral.position - liftOffLateral.position) / (touchDownLateral.time - liftOffLateral.time)};

        feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);
        feetLongitTrajectories_[j].emplace_back(liftOffLong, midSwingLong, touchDownLong);     // longitudinal
        feetLateralTrajectories_[j].emplace_back(liftOffLateral, midSwingLateral, touchDownLateral);     // lateral

      } else {  // for a stance leg
        // Note: setting the time here arbitrarily to 0.0 -> 1.0 makes the assert in CubicSpline fail
        const CubicSpline::Node liftOff{0.0, liftOffHeightSequence[j][p], 0.0};
        const CubicSpline::Node touchDown{1.0, liftOffHeightSequence[j][p], 0.0};
        feetHeightTrajectories_[j].emplace_back(liftOff, liftOffHeightSequence[j][p], touchDown);

        const CubicSpline::Node liftOffLong{0.0, liftOffLongSequence[j][p], 0.0};     // longitudinal
        const CubicSpline::Node touchDownLong{1.0, liftOffLongSequence[j][p], 0.0};
        feetLongitTrajectories_[j].emplace_back(liftOffLong, liftOffLongSequence[j][p], touchDownLong);   // for stance X and Z is same (not moving)

        const CubicSpline::Node liftOffLateral{0.0, liftOffLateralSequence[j][p], 0.0};     // lateral
        const CubicSpline::Node touchDownLateral{1.0, liftOffLateralSequence[j][p], 0.0};
        feetLateralTrajectories_[j].emplace_back(liftOffLateral, liftOffLateralSequence[j][p], touchDownLateral);

      }
    }
    feetHeightTrajectoriesEvents_[j] = eventTimes;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<std::vector<int>, std::vector<int>> SwingTrajectoryPlanner::updateFootSchedule(const std::vector<bool>& contactFlagStock) {
  const size_t numPhases = contactFlagStock.size();

  std::vector<int> startTimeIndexStock(numPhases, 0);
  std::vector<int> finalTimeIndexStock(numPhases, 0);

  // find the startTime and finalTime indices for swing feet
  for (size_t i = 0; i < numPhases; i++) {
    if (!contactFlagStock[i]) {
      std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
    }
  }
  return {startTimeIndexStock, finalTimeIndexStock};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::vector<bool>> SwingTrajectoryPlanner::extractContactFlags(const std::vector<size_t>& phaseIDsStock) const {
  const size_t numPhases = phaseIDsStock.size();
  // std::cout << "numPhases: " << numPhases << std::endl;
  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < numFeet_; j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<int, int> SwingTrajectoryPlanner::findIndex(size_t index, const std::vector<bool>& contactFlagStock) {
  const size_t numPhases = contactFlagStock.size();

  // skip if it is a stance leg
  if (contactFlagStock[index]) {
    return {0, 0};
  }

  // find the starting time
  int startTimesIndex = -1;
  for (int ip = index - 1; ip >= 0; ip--) {
    if (contactFlagStock[ip]) {
      startTimesIndex = ip;
      break;
    }
  }

  // find the final time
  int finalTimesIndex = numPhases - 1;
  for (size_t ip = index + 1; ip < numPhases; ip++) {
    if (contactFlagStock[ip]) {
      finalTimesIndex = ip - 1;
      break;
    }
  }

  return {startTimesIndex, finalTimesIndex};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                                      const std::vector<size_t>& phaseIDsStock) {
  const size_t numSubsystems = phaseIDsStock.size();
  if (startIndex < 0) {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++) {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of take-off for the first swing of the EE with ID " + std::to_string(leg) + " is not defined.");
  }
  if (finalIndex >= numSubsystems - 1) {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++) {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of touch-down for the last swing of the EE with ID " + std::to_string(leg) + " is not defined.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale) {
  // std::cout << "IIT Swing trajectory scaling" << std::endl;
  // std::cout << "Start time: " << startTime << std::endl;
  // std::cout << "Final time: " << finalTime << std::endl;
  // std::cout << "Swing time scale: " << swingTimeScale << std::endl;
  return std::min(1.0, (finalTime - startTime) / swingTimeScale);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);

  if (verbose) {
    std::cerr << "\n #### Swing Trajectory Config:";
    std::cerr << "\n #### =============================================================================\n";
  }

  SwingTrajectoryPlanner::Config config;
  const std::string prefix = fieldName + ".";

  loadData::loadPtreeValue(pt, config.addPlanarConstraints, prefix + "addPlanarConstraints", verbose);

  loadData::loadPtreeValue(pt, config.liftOffVelocity, prefix + "liftOffVelocity", verbose);
  loadData::loadPtreeValue(pt, config.touchDownVelocity, prefix + "touchDownVelocity", verbose);
  loadData::loadPtreeValue(pt, config.swingHeight, prefix + "swingHeight", verbose);
  loadData::loadPtreeValue(pt, config.swingTimeScale, prefix + "swingTimeScale", verbose);

  loadData::loadPtreeValue(pt, config.liftOffLongVelocity, prefix + "liftOffLongVelocity", verbose);
  loadData::loadPtreeValue(pt, config.touchDownLongVelocity, prefix + "touchDownLongVelocity", verbose);
  loadData::loadPtreeValue(pt, config.longStepLength, prefix + "longStepLength", verbose);

  loadData::loadPtreeValue(pt, config.liftOffLateralVelocity, prefix + "liftOffLateralVelocity", verbose);
  loadData::loadPtreeValue(pt, config.touchDownLateralVelocity, prefix + "touchDownLateralVelocity", verbose);
  loadData::loadPtreeValue(pt, config.lateralStepLength, prefix + "lateralStepLength", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return config;
}

SwingTrajectoryPlanner::Config SwingTrajectoryPlanner::getConfig()
{
    return config_;
}

}  // namespace legged_robot
}  // namespace ocs2
