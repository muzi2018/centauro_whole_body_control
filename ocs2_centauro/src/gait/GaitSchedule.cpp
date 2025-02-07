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
******************************************************************************/

#include "ocs2_centauro/gait/GaitSchedule.h"
#include <numeric>

int i = 0;

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitSchedule::GaitSchedule(ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate, scalar_t phaseTransitionStanceTime)
    : modeSchedule_(std::move(initModeSchedule)),
      modeSequenceTemplate_(std::move(initModeSequenceTemplate)),
      phaseTransitionStanceTime_(phaseTransitionStanceTime) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::insertModeSequenceTemplate(const ModeSequenceTemplate& modeSequenceTemplate, scalar_t startTime, scalar_t finalTime) {
  // std::cout << " insertModeSequenceTemplate " << std::endl;
  modeSequenceTemplate_ = modeSequenceTemplate;
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;

  // find the index on which the new gait should be added
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), startTime) - eventTimes.begin();

  // delete the old logic from the index
  if (index < eventTimes.size()) {
    eventTimes.erase(eventTimes.begin() + index, eventTimes.end());
    modeSequence.erase(modeSequence.begin() + index + 1, modeSequence.end());
  }

  // add an intermediate stance phase unless at the start of the new gait we are already at stance
  scalar_t phaseTransitionStanceTime = phaseTransitionStanceTime_;
  auto lastModeLegContactFlags = modeNumber2StanceLeg(modeSequence.back());
  if (!modeSequence.empty() && std::accumulate(lastModeLegContactFlags.begin(), lastModeLegContactFlags.end(), 0) == 4) {
    phaseTransitionStanceTime = 0.0;
  }

  if (phaseTransitionStanceTime > 0.0) {
    eventTimes.push_back(startTime);
    modeSequence.push_back(ModeNumber::STANCE);
  }

  // tile the mode sequence template from startTime+phaseTransitionStanceTime to finalTime.
  tileModeSequenceTemplate(startTime + phaseTransitionStanceTime, finalTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * templateModeSequence: modeSequence = {11 15}, switchingTimes = {0 0.8 1.6} 
 * 1. currentTime=initTime = 0.0    finalTime = 1.0   lowerBoundTime = -1.0       upperBoundTime = 2.0      => eventTimes = 2,                            modeSequence = 15 15 
 * 2. currentTime=initTime = 0.002  finalTime = 1.002 lowerBoundTime = -0.998     upperBoundTime = /2.002/  => eventTimes = 2 2.8 3.6,                    modeSequence = 15 11 15 15
 * 3. currentTime=initTime = 1.608  finalTime = 2.608 lowerBoundTime = 0.608      upperBoundTime = /3.608/  => eventTimes = 2 2.8 3.6,4.4,5.2,            modeSequence = 15 11 15 11 15 15
 * 4. currentTime=initTime = 3.206  finalTime = 4.206 lowerBoundTime = 2.206      upperBoundTime = /5.206/  => eventTimes = 2 2.8 3.6,4.4,5.2,6.0,6.8     modeSequence = 15 11 15 11 15 11 15 15
 * 5. currentTime=initTime = 3.812  finalTime = 4.812 lowerBoundTime = /2.812/    upperBoundTime = 5.812    => eventTimes = 2.8 3.6,4.4,5.2,6.0,6.8       modeSequence = 15 15 11 15 11 15 15
 * 6. currentTime=initTime = 4.608  finalTime = 5.608 lowerBoundTime = /3.608/    upperBoundTime = 6.608    => eventTimes = 3.6 4.4 5.2 6 6.8             modeSequence = 15 11 15 11 15 15
 *                                                                                                                                                        modeSequence = 15 11 15 11 15 11 15 15
 *                                                                                                                                                        modeSequence = 15 15 11 15 11 15 15
 *                                                                                                                                                        modeSequence = 15 11 15 11 15 15
 *                                                                                                                                                        modeSequence = 15 11 15 11 15 11 15 15
 *                                                                                                                                                        modeSequence = 15 15 11 15 11 15 15
*/

ModeSchedule GaitSchedule::getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime) { // lowerBoundTime = 2.0 upperBoundTime = 1.0
  auto& eventTimes = modeSchedule_.eventTimes; // evenTimes: 2.0
  auto& modeSequence = modeSchedule_.modeSequence; // modeSequence: STANCE STANCE
  
  std::cout << "lowerBoundTime = " << lowerBoundTime << std::endl;
  std::cout << "upperBoundTime = " << upperBoundTime << std::endl;
  std::cout << "eventTimes =" ;
  for (size_t i = 0; i < eventTimes.size(); i++)
  {
    std::cout << eventTimes[i] << " " ;
  }
  std::cout << std::endl;
  std::cout << "modeSequence =" ;
  for (size_t i = 0; i < modeSequence.size(); i++)
  {
    std::cout << modeSequence[i] << " " ;
  }
  std::cout << std::endl;




  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), lowerBoundTime) - eventTimes.begin(); // index = 0

  if (index > 0) { 
    eventTimes.erase(eventTimes.begin(), eventTimes.begin() + index - 1);   
    modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);  
    modeSequence.front() = ModeNumber::STANCE; 
  }

  const auto tilingStartTime = eventTimes.empty() ? upperBoundTime : eventTimes.back(); // tilingStartTime = 2.0
  eventTimes.erase(eventTimes.end() - 1, eventTimes.end()); // eventTimes: Nan
  modeSequence.erase(modeSequence.end() - 1, modeSequence.end()); // modeSequence: STANCE

  tileModeSequenceTemplate(tilingStartTime, upperBoundTime); 
  // std::cout << "lowerBoundTime = " << lowerBoundTime << std::endl;

  
  return modeSchedule_;// eventTimes: 2.0 2.8 3.6, 10.0 modeSequence: STANCE LF_LH_RH STANCE STANCE

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime) { // startTime = 2.0 finalTime = 3.0
  auto& eventTimes = modeSchedule_.eventTimes; // eventTimes = Nan 
  auto& modeSequence = modeSchedule_.modeSequence;// modeSequence: STANCE
  std::vector<size_t> templateModeSequence;
  templateModeSequence.push_back(ModeNumber::LF_LH_RH);
  templateModeSequence.push_back(ModeNumber::STANCE); // templateModeSequence: LF_LH_RH, STANCE

  if (!eventTimes.empty() && startTime <= eventTimes.back()) {
    throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");
  }

  // add a initial time
  eventTimes.push_back(startTime); // eventTimes:2.0
 
  while (eventTimes.back() < finalTime) {
    for (size_t i = 0; i < templateModeSequence.size(); i++) { //size : 2 
      modeSequence.push_back(templateModeSequence[i]); // modeSequence: STANCE LF_LH_RH STANCE
      scalar_t deltaTime = 0.8; 
      eventTimes.push_back(eventTimes.back() + deltaTime); // eventTimes: 2.0 2.8 3.6
    }  
  }    

  modeSequence.push_back(ModeNumber::STANCE);// modeSequence: STANCE LF_LH_RH STANCE STANCE
}

}  // namespace legged_robot
}  // namespace ocs2
