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
ModeSchedule GaitSchedule::getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime) {
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;

  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), lowerBoundTime) - eventTimes.begin();
    // std::cout << "index: " << index << std::endl;
    // std::cout << "modeSequence size = " << modeSequence.size() << std::endl;
    // std::cout << "lowerBoundTime: " << lowerBoundTime << " upperBoundTime: " << upperBoundTime << std::endl;


  if ( 0 )
  {
    std::cout << "index: " << index << std::endl;
    std::cout << "lowerBoundTime: " << lowerBoundTime << " upperBoundTime: " << upperBoundTime << std::endl;
    std::cout << "modeSequence size = " << modeSequence.size() << std::endl;

  

    for (size_t i = 0; i < modeSequence.size(); i++)
    {
      std::cout << modeSequence[i] << std::endl;
    }
    std::cout << "evenTimes size = "  << eventTimes.size() << std::endl;
    for (size_t i = 0; i < eventTimes.size(); i++)
    {
      std::cout << eventTimes[i] << std::endl;
    }

  if (modeSequence.size() == 3 )
  {

    i ++ ;
    std::cout << "--- getModeSchedule --- " << std::endl;
    // std::cout << "lowerBoundTime: " << lowerBoundTime << " upperBoundTime: " << upperBoundTime << std::endl;
    // std::cout << "default modeSequence size = " << modeSequence.size() << std::endl;
    // for (size_t i = 0; i < modeSequence.size(); i++)
    // {
    //   std::cout << modeSequence[i] << std::endl;
    // }

    // std::cout << "default evenTimes size = "  << eventTimes.size() << std::endl;
    // for (size_t i = 0; i < eventTimes.size(); i++)
    // {
    //   std::cout << eventTimes[i] << std::endl;
    // }
    // std::cout << "the lowerBounTime is on " << index << " th " << "eventTimes" << std::endl; 
    
    // std::cout << "tilingStartTime: " << tilingStartTime << std::endl;

    // std::cout << "after erase" << std::endl;
    // std::cout << "modeSequence size = " << modeSequence.size() << std::endl;
    // for (size_t i = 0; i < modeSequence.size(); i++)
    // {
    //   std::cout << modeSequence[i] << std::endl;
    // }

    // std::cout << "evenTimes size = "  <<  eventTimes.size() << std::endl;
    // for (size_t i = 0; i < eventTimes.size(); i++)
    // {
    //   std::cout << eventTimes[i] << std::endl;
    // }

  }

  }

  if (index > 0) {
    // delete the old logic from index and set the default start phase to stance
    eventTimes.erase(eventTimes.begin(), eventTimes.begin() + index - 1);  // keep the one before the last to make it stance
    modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);
    if (0)
    {
      std::cout << "lowerBoundTime = " << lowerBoundTime << std::endl;
      std::cout << "upperBoundTime = " << upperBoundTime << std::endl;
      std::cout << " ----------------*************--------------- " << std::endl;
      std::cout << "modeSequence size = " << modeSequence.size() << std::endl;
      for (size_t i = 0; i < modeSequence.size(); i++)
      {
        std::cout << modeSequence[i] << std::endl;
      }
      std::cout << "evenTimes size = "  <<  eventTimes.size() << std::endl;
      for (size_t i = 0; i < eventTimes.size(); i++)
      {
        std::cout << eventTimes[i] << std::endl;
      }
    }
    
    // set the default initial phase
    modeSequence.front() = ModeNumber::STANCE;
  }


  // Start tiling at time
  const auto tilingStartTime = eventTimes.empty() ? upperBoundTime : eventTimes.back();
  

  // delete the last default stance phase
  eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
  modeSequence.erase(modeSequence.end() - 1, modeSequence.end());

if ( 0 )
{

  std::cout << "tilingStartTime = " << tilingStartTime << std::endl;
  std::cout << "delete the last default stance phase" << std::endl;
  std::cout << "modeSequence size = " << modeSequence.size() << std::endl;
  for (size_t i = 0; i < modeSequence.size(); i++)
  {
    std::cout << modeSequence[i] << std::endl;
  }
  std::cout << "evenTimes size = "  <<  eventTimes.size() << std::endl;
  for (size_t i = 0; i < eventTimes.size(); i++)
  {
    std::cout << eventTimes[i] << std::endl;
  }


  
  
  i++;
  std::cout << "--: enter tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime)" << std::endl;
}


  // tile the template logic
  tileModeSequenceTemplate(tilingStartTime, upperBoundTime);
  // std::cout << "***************************************" << std::endl;

  if (0)
  {
    std::cout << "lowerBoundTime = " << lowerBoundTime << std::endl;
    std::cout << "upperBoundTime = " << upperBoundTime << std::endl;
    std::cout << " ----------------*************--------------- " << std::endl;
    std::cout << "modeSequence size = " << modeSequence.size() << std::endl;
    for (size_t i = 0; i < modeSequence.size(); i++)
    {
      std::cout << modeSequence[i] << std::endl;
    }
    std::cout << "evenTimes size = "  <<  eventTimes.size() << std::endl;
    for (size_t i = 0; i < eventTimes.size(); i++)
    {
      std::cout << eventTimes[i] << std::endl;
    }
  }

  // std::cout << std::endl;
  return modeSchedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime) { // 6 6.004


  
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;
  const auto& templateTimes = modeSequenceTemplate_.switchingTimes;
  const auto& templateModeSequence = modeSequenceTemplate_.modeSequence;
  const size_t numTemplateSubsystems = modeSequenceTemplate_.modeSequence.size();
  if ( 0 )
  {
    std::cout << std::endl;
    std::cout << "tilingStartTime = " << startTime << std::endl;
    std::cout << "tilingFinalTime = " << finalTime << std::endl;
    std::cout << "eventTimes size = " << eventTimes.size() << std::endl;
    std::cout << "modeSequence size = " << modeSequence.size() << std::endl;
    std::cout << "templateTimes size = " << templateTimes.size() << std::endl;
    std::cout << "templateModeSequence size = " << templateModeSequence.size() << std::endl;
    std::cout << "numTemplateSubsystems size = " << numTemplateSubsystems << std::endl;
  }
  



  // If no template subsystem is defined, the last subsystem should continue for ever
  if (numTemplateSubsystems == 0) {
    return;
  }

  if (!eventTimes.empty() && startTime <= eventTimes.back()) {
    throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");
  }

  // add a initial time
  eventTimes.push_back(startTime);

  // std::cout << "add a initial time" << std::endl;
  // std::cout << "eventTimes size = " << eventTimes.size() << std::endl;
  // for (const auto& elem : eventTimes) {
  //   std::cout << elem << " ";
  // }
  // std::cout << std::endl;




  // TODO: why finalTime is not the final time and is the duration of the horizon of the mpc?
  // concatenate from index
  while (eventTimes.back() < finalTime) {
    // std::cout << "when the final time in eventTimes < finalTime" << std::endl;
    // std::cout << "finalTime = " << finalTime << std::endl;
    // std::cout << "eventTimes.back() = " << eventTimes.back() << std::endl;
    for (size_t i = 0; i < templateModeSequence.size(); i++) {
      modeSequence.push_back(templateModeSequence[i]);
      scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i];
      eventTimes.push_back(eventTimes.back() + deltaTime);
    }  // end of i loop
  }    // end of while loop

  // default final phase
  modeSequence.push_back(ModeNumber::STANCE);

  // std::cout << "finalmodeSchedule " << std::endl;
  // std::cout << "eventTimes size = " << eventTimes.size() << std::endl;
  // for (const auto& elem : eventTimes) {
  //   std::cout << elem << " ";
  // }

  // std::cout << std::endl << std::endl;

  // std::cout << "modeSequence size = " << modeSequence.size() << std::endl;
  // for (const auto& elem : modeSequence) {
  //   std::cout << elem << " ";
  // }
  // std::cout << std::endl;

  if ( 0)
  {
    std::cout << "--- tileModeSequenceTemplate --- " << std::endl;

    i ++;
    std::cout << "startTime: " << startTime << " finalTime: " << finalTime << std::endl;
    std::cout << "eventTimes: " << eventTimes.size() << std::endl;
    for (const auto& elem : eventTimes) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    std::cout << "modeSequence: ";
    for (const auto& elem : modeSequence) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    std::cout << "templateTimes: ";
    for (const auto& elem : templateTimes) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    std::cout << "templateModeSequence: ";
    for (const auto& elem : templateModeSequence) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    std::cout << "numTemplateSubsystems: " << numTemplateSubsystems << std::endl;

    std::cout << std::endl;

    std::cout << "after add init time eventTimes: " << eventTimes.size() << std::endl;
    for (const auto& elem : eventTimes) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    std::cout << "final modesequence: " << std::endl;


    std::cout << "eventTimes: " << modeSchedule_.eventTimes.size() << std::endl;
    for (const auto& elem : modeSchedule_.eventTimes) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    std::cout << "modeSequence: " << modeSchedule_.modeSequence.size() << std::endl;
    for (const auto& elem : modeSchedule_.modeSequence) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

  }

}

}  // namespace legged_robot
}  // namespace ocs2
