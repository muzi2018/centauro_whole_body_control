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
 * 1. currenttime = 0.0 lowerBoundTime = -1.0 upperBoundTime = 1.0 => eventTimes = 2.0, modeSequence: STANCE STANCE
 * 2. currenttime = 2.0 lowerBoundTime = 1.0 upperBoundTime = 3.0 => eventTimes = 2.0 6.0, modeSequence: STANCE STANCE STANCE 
 * 3. currenttime = 6.0 lowerBoundTime = 5.0 upperBoundTime = 7.0 => eventTimes = 6.0, 10.0, modeSequence: STANCE STANCE STANCE
 */

ModeSchedule GaitSchedule::getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime) { // currenttime = 2.0 lowerBoundTime = 1.0 upperBoundTime = 3.0
  auto& eventTimes = modeSchedule_.eventTimes; // evenTimes: 2.0
  auto& modeSequence = modeSchedule_.modeSequence; // modeSequence: STANCE STANCE



  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), lowerBoundTime) - eventTimes.begin();
  if (index > 0) { 
    eventTimes.erase(eventTimes.begin(), eventTimes.begin() + index - 1);  
    modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);  
    modeSequence.front() = ModeNumber::STANCE; 
  }

  const auto tilingStartTime = eventTimes.empty() ? upperBoundTime : eventTimes.back(); // tilingStartTime = 2.0
  eventTimes.erase(eventTimes.end() - 1, eventTimes.end()); // eventTimes: Nan
  modeSequence.erase(modeSequence.end() - 1, modeSequence.end()); // modeSequence: STANCE

  tileModeSequenceTemplate(tilingStartTime, upperBoundTime); // tilingStartTime = 2.0, upperBoundTime = 3.0
  return modeSchedule_;// eventTimes = 2.0 6.0, modeSequence: STANCE STANCE STANCE

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime) { // startTime =2.0 finalTime = 3.0
  auto& eventTimes = modeSchedule_.eventTimes; // eventTimes = Nan 
  auto& modeSequence = modeSchedule_.modeSequence;// modeSequence: STANCE
  const auto& templateTimes = modeSequenceTemplate_.switchingTimes;// templateTimes: 0.0 4.0
  const auto& templateModeSequence = modeSequenceTemplate_.modeSequence;// templateModeSequence: STANCE
  const size_t numTemplateSubsystems = modeSequenceTemplate_.modeSequence.size();// numTemplateSubsystems: 2

  if (numTemplateSubsystems == 0) {
    return;
  }

  if (!eventTimes.empty() && startTime <= eventTimes.back()) {
    throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");
  }

  // add a initial time
  eventTimes.push_back(startTime); // eventTimes:2.0
  
  while (eventTimes.back() < finalTime) {
    for (size_t i = 0; i < templateModeSequence.size(); i++) { // 1
      modeSequence.push_back(templateModeSequence[i]); // modeSequence: STANCE STANCE
      scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i]; // deltaTime: 4.0
      eventTimes.push_back(eventTimes.back() + deltaTime); // evnetTimes: 2.0 6.0
    }  // end of i loop
  }    // end of while loop

  // default final phase
  modeSequence.push_back(ModeNumber::STANCE);// modeSequence: STANCE STANCE STANCE
}

}  // namespace legged_robot
}  // namespace ocs2
