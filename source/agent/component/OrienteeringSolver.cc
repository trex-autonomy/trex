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

#include "OrienteeringSolver.hh"
#include "Token.hh"

namespace TREX {
  DynamicGoalFilter::DynamicGoalFilter(const TiXmlElement& configData): FlawFilter(configData, true) {}

  bool DynamicGoalFilter::test(const EntityId& entity){
    checkError(TokenId::convertable(entity), "Invalid configuration for " << entity->toString());

    TokenId token(entity);
    return !OrienteeringSolver::isGlobalNextGoal(token);
  }



  std::vector<OrienteeringSolverId> OrienteeringSolver::s_goalSolvers;

  bool OrienteeringSolver::isGlobalNextGoal(TokenId token) {
    for (std::vector<OrienteeringSolverId>::iterator it = s_goalSolvers.begin(); 
	 it != s_goalSolvers.end(); it++) {
      if ((*it)->isNextGoal(token)) {
	return true;
      }
    }
    return false;
  }

  bool OrienteeringSolver::isNextGoal(TokenId token) {
    debugMsg("OrienteeringSolver", (m_goalManager->isNextToken(token) ? "Is" : "Not")
	     << std::string(" next goal: ") << token->toString());
    return m_goalManager->isNextToken(token);
  }


  OrienteeringSolver::OrienteeringSolver(const TiXmlElement& cfgXml) 
    : FlawManagerSolver(cfgXml), m_goalManager(GoalManagerId::noId()), m_stepCount(0) {
    s_goalSolvers.push_back(this->getId());
  }
  
  OrienteeringSolver::~OrienteeringSolver() {
    for (std::vector<OrienteeringSolverId>::iterator it = s_goalSolvers.begin(); 
	 it != s_goalSolvers.end(); it++) {
      if ((OrienteeringSolver*)(*it) == this) {
	s_goalSolvers.erase(it);
	break;
      }
    }
  }
  
  void OrienteeringSolver::init(PlanDatabaseId db, TiXmlElement* cfgXml) {
    ///initDbListener(db, cfgXml);
    initCeListener(db, cfgXml);
    initFlawManagers(db, cfgXml);
    assertTrue(getFlawManagerCount() == 1, 
	       "You must have one and only one GoalManager per solver, and nothing else.");
    assertTrue((GoalManager*)getFlawManager(0),
	       "You must have one and only one GoalManager per solver, and nothing else.");
    m_goalManager = ((GoalManager*)getFlawManager(0))->getId();
  }
  
  bool OrienteeringSolver::isExhausted() {
    return false;
  }
  
  void OrienteeringSolver::step() { 
    if (!noMoreFlaws()) {
      m_stepCount++;
      m_goalManager->step();
    }
  }
  
  unsigned int OrienteeringSolver::getDepth() {
    return m_stepCount;
  }
  
  unsigned int OrienteeringSolver::getStepCount() {
    return m_stepCount;
  }
  
  
  bool OrienteeringSolver::noMoreFlaws() {
    return m_goalManager->noMoreFlaws();
  }
  
  
  void OrienteeringSolver::clear() {
    m_stepCount = 0;
    m_goalManager->reset();
  }
  
  
  void OrienteeringSolver::reset() {
    m_stepCount = 0;
    m_goalManager->reset();
  }
  
  
}
