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

#include "ocs2_centauro/reference_manager/SwitchedModelReferenceManager.h"
#include <numeric>
namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                             int targetFrameNumber)
    : ReferenceManager(std::vector<TargetTrajectories>(targetFrameNumber, TargetTrajectories()), TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                             std::shared_ptr<ArmSwingTrajectoryPlanner> armSwingTrajectoryPtr,
                                                             std::shared_ptr<ForceTorqueSensing> forceTorqueSensingPtr,
                                                             int targetFrameNumber)
    : ReferenceManager(std::vector<TargetTrajectories>(targetFrameNumber, TargetTrajectories()), TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
      armSwingTrajectoryPtr_(std::move(armSwingTrajectoryPtr)),
      forceTorqueSensingPtr_(std::move(forceTorqueSensingPtr)){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
locoma_contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2ActiveContacts(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;

  // TODO: if I increase the final time it can be useful for planning motion and contact switch later
  // std::cout << "************    foot gait   ***********" << std::endl;
  
  if ( 0 ){
    /* code */
    std::cout << "************    foot gait   ***********" << std::endl;
    std::cout << "planning initTime: " << initTime << std::endl;
    std::cout << "planning finalTime: " << finalTime << std::endl << std::endl;
    std::cout << "-: enter getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime) " << std::endl;
  }

    // std::cout << "timeHorizon: " << timeHorizon << std::endl;
  

  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  // std::cout << "initTime: " << initTime - timeHorizon << std::endl;
  // std::cout << "finalTime: " << finalTime + timeHorizon << std::endl;
  // std::cout << "timeHorizon: " << timeHorizon << std::endl;

  // const auto& modeSequence = modeSchedule.modeSequence;
  // std::cout << "modeSequence has size " << modeSequence.size() << std::endl;

  
  // for (size_t i = 0; i < modeSequence.size(); i++)
  // {
  //   std::cout << "modeSequence [" << i << "]: " << modeSequence[i] << std::endl;
  // }
  // std::cout << std::endl;


  // const auto& eventTimes = modeSchedule.eventTimes;
  // std::cout << "eventTimes has size " << eventTimes.size() << std::endl;
  // for (size_t i = 0; i < eventTimes.size(); i++)
  // {
  //   std::cout << "eventTimes [" << i << "]: " << eventTimes[i] << std::endl;
  // }
  // std::cout << std::endl;


  const scalar_t terrainHeight = 0.0;

  // Define eeCurrentPosition
  feet_array_t<scalar_array_t> eeCurrentPosition;
  arms_array_t<scalar_array_t> armEeCurrentPosition, armEeCurrentOrientation;

  // Receive eeKinematicsPosition from Kinematics pointer objects
  for (int i = 0; i< eeKinematicsPtrArray_.size(); i++){
      auto eeKinematicsPosition = eeKinematicsPtrArray_.at(i)->getPosition(initState);      // get EE position at initial state
      // std::cout << eeKinematicsPosition.size() << std::endl;
      for (int j = 0; j < eeKinematicsPosition.size(); j++) {
          // set values to eeCurrentPosition, from eigen::matrix to std::vector
          // std::cout << eeKinematicsPosition.at(j).rows() << std::endl;
          eeCurrentPosition[i].assign(eeKinematicsPosition.at(j).data(),
                                    eeKinematicsPosition.at(j).data() + eeKinematicsPosition.at(j).rows() * eeKinematicsPosition.at(j).cols());
          
          // Debug
//          std::cout << "[Yiannis] Contact position: " << eeKinematicsPosition.at(j).transpose() << std::endl;
      }
  }


  swingTrajectoryPtr_->update(modeSchedule, initTime, terrainHeight, eeCurrentPosition);  // pass the current ee position
  if (armSwingTrajectoryPtr_ != nullptr) {      // if arm trajectories have to be planned
      // receive current arm ee position
      for (int i = 0; i< armEeKinematicsPtrArray_.size(); i++){
          auto eeKinematicsPosition = armEeKinematicsPtrArray_.at(i)->getPosition(initState);      // get EE position at initial state
          auto eeKinematicsOrientation = armEeKinematicsPtrArray_.at(i)->getOrientation(initState);

          for (int j = 0; j < eeKinematicsPosition.size(); j++) {
              // set values to armEeCurrentPosition, from eigen::matrix to std::vector
              armEeCurrentPosition[i].assign(eeKinematicsPosition.at(j).data(),
                                        eeKinematicsPosition.at(j).data() + eeKinematicsPosition.at(j).rows() * eeKinematicsPosition.at(j).cols());
          }
          for (int j = 0; j < eeKinematicsOrientation.size(); j++) {
              armEeCurrentOrientation[i].assign(eeKinematicsOrientation[j].data(),
                                        eeKinematicsOrientation[j].data() + eeKinematicsOrientation[j].size());
          }
      }
      armSwingTrajectoryPtr_->update(modeSchedule, initTime, armEeCurrentPosition, armEeCurrentOrientation,
                                     {std::move(getFrameTargetTrajectories(0)), std::move(getFrameTargetTrajectories(1))});
  }
}

}  // namespace legged_robot
}  // namespace ocs2
