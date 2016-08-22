/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2007, MBARI.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the TREX Project nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/

#include "Id.hh"
#include "LabelStr.hh"
#include "Debug.hh"

/**
 * @author Conor McGann
 * @file Common type declarations used for TREX.
 */

using namespace EUROPA;

namespace TREX {

  /*!< The type for a tick - a unit of time */
  typedef unsigned int TICK;

  class Agent;
  typedef Id<Agent> AgentId;

  class AgentListener;
  typedef Id<AgentListener> AgentListenerId;

  class TeleoReactor;
  typedef Id<TeleoReactor> TeleoReactorId;

  class Server;
  typedef Id<Server> ServerId;

  class Observer;
  typedef Id<Observer> ObserverId;

  class Observation;

  class DbCore;
  typedef Id<DbCore> DbCoreId;

  /* USEFUL UTILITIES */

  LabelStr compose(const LabelStr& prefix, const LabelStr& suffix);

}

#define CPU_STAT_LOG "cpuStat.log"

/**
  @brief Create a logging message, which will
  only be created or used when the given condition is true at run time.
  @param marker A string that "marks" the message to enable it by.
  @param data The data to be printed when the message is enabled.
*/
#define TREX_INFO(marker, data) TREX_INFO_COND(true, marker, data)

/**
  @brief Create a conditional logging message, which will
  only be created or used when the given condition is true at run time.
  @param cond An additional condition to be checked before printing the message,
         which can be any C/C++ expression that could be used in an if statement.
  @param marker A string that "marks" the message to enable it by.
  @param data The data to be printed when the message is enabled.
*/
#define TREX_INFO_COND(cond, marker, data) { \
  static DebugMessage *dmPtr = DebugMessage::addMsg(__FILE__, __LINE__, marker); \
  if (dmPtr->isEnabled() && (cond)) { \
    try { \
      getStream().exceptions(std::ios_base::badbit); \
      getStream() << "[" << marker << "]" << data << std::endl; \
    } \
    catch(std::ios_base::failure& exc) { \
      checkError(ALWAYS_FAIL, exc.what()); \
      throw; \
    } \
  } \
}
