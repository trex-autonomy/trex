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
#include "ActionAdapter.hh"

/**
 * @author Conor McGann
 * @brief Demonstrate plug-in of handling goal requests and generating observations
 * In practice we may wish to execute the goal on a separate thread to increase throughput of the Agent since execution will be slow, including network hops. We
 * will not want to block the agent on that. That can be addressed in a more substantive base class.
 */
namespace TREX {

  class GamePlayAdapter: public Adapter {
  public:
    GamePlayAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : Adapter(agentName, LabelStr("Adapter"), 1, 0, "Adapter.cfg"){}

    void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
      m_observer = observer;

      // Generate Observations for initial states of externals
      m_observer->notify(ObservationByValue("player1", "Player.Wait"));
      m_observer->notify(ObservationByValue("player1.hand", "PlayerState.Hides"));
      m_observer->notify(ObservationByValue("player2", "Player.Wait"));
      m_observer->notify(ObservationByValue("player2.hand", "PlayerState.Hides"));
    }

    void handleNextTick(){}

    void notify(const Observation& observation){
      checkError("This should never happen", observation.getObjectName().toString());
    }

    bool handleRequest(const TokenId& goal){
      // Receipt of this goal means immediate execution, so we can propagate the bounds immediately
      checkError(goal->start()->lastDomain().isMember(getCurrentTick()), 
		 getCurrentTick() << " is not a member of the current domain " <<
		 goal->start()->lastDomain().toString());

      goal->start()->restrictBaseDomain(IntervalIntDomain(getCurrentTick()));

      ObjectId object = goal->getObject()->lastDomain().getSingletonValue();

      debugMsg("GamePlayAdapter:handleRequest", 
	       goal->toString() << " for " << object->getName().toString() << " starting in " << goal->start()->lastDomain().toString());

      // All goals received will produce an observation that they happened
      const LabelStr& timelineName = object->getName();
      const LabelStr& predicateName = goal->getPredicateName();
      ObservationByValue observation(timelineName, predicateName);
      m_observer->notify(observation);

      return true;
    }

  private:
    ObserverId m_observer; /*!< The agent observer interface*/
  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<GamePlayAdapter> l_GamePlayAdapter_Factory("GamePlayAdapter");

}
