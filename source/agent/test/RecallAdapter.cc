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

#include "Adapter.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Observer.hh"

/**
 * @author Conor McGann
 * @brief Demonstrate plug-in of handling goal requests and generating observations
 * Used to test the recall function where it will generate notifications for a single numeric timeline based on the tick value. This will
 * conflict with the planned values for the other reactors.
 */
namespace TREX {

  class RecallAdapter: public Adapter {
  public:
    RecallAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : Adapter(agentName, configData){}

    void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
      m_observer = observer;
      ObservationByValue obs("c", "NumberTimeline.holds");
      obs.push_back("value", new IntervalIntDomain(0,0));
      m_observer->notify(obs);
    }

    /**
     * Force observation of value based on tick value. This will force plan failures directly and indirectly.
     */
    bool synchronize(){
      ObservationByValue obs("c", "NumberTimeline.holds");
      obs.push_back("value", new IntervalIntDomain(getCurrentTick(),getCurrentTick()));
      m_observer->notify(obs);
      return true;
    }

    void notify(const Observation& observation){
      checkError("This should never happen", observation.getObjectName().toString());
    }

    bool handleRequest(const TokenId& goal){ return true;}

  private:
    ObserverId m_observer; /*!< The agent observer interface*/
  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<RecallAdapter> l_RecallAdapter_Factory("RecallAdapter");
}
