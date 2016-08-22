#include "TestMonitor.hh"
#include "Utilities.hh"

/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2009. Willow Garage.
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

namespace TREX {

  TestConditionHandler::TestConditionHandler(const TiXmlElement& config)
    : FlawHandler(config){
    // Refresh the weight to use the over-ridden function
    refreshWeight();

    debugMsg("TestConditionHandler:TestConditionHandler", "Allocation of Factory");
  }

  std::string TestConditionHandler::toString() const{
    std::stringstream ss;
    ss << "[TestConditionHandler]" << FlawHandler::toString();
    return ss.str();
  }

  DecisionPointId TestConditionHandler::create(const DbClientId& client, const EntityId& flaw, const LabelStr& explanation) const {
    DecisionPoint* dp = new EUROPA::SOLVERS::OpenConditionDecisionPoint(client, flaw, *FlawHandler::m_configData, explanation);
    dp->setContext(m_context);
    return dp->getId();
  }

  bool TestConditionHandler::customStaticMatch(const EntityId& entity) const {
    bool result = TestMonitor::isCondition(entity->getKey());
    debugMsg("TestConditionHandler:customStaticMatch", entity->toString() << " is " << (result ? "" : "not ") << "a match.");
    return result;
  }

  TestMonitorConstraintBase::TestMonitorConstraintBase(const LabelStr& name,
						       const LabelStr& propagatorName,
						       const ConstraintEngineId& constraintEngine,
						       const std::vector<ConstrainedVariableId>& variables,
						       bool shouldBeCompleted)
    : Constraint(name, propagatorName, constraintEngine, variables){
    checkError(variables.size() == 1, variables.size());
    checkError(TokenId::convertable(variables[0]->parent()), variables[0]->toString());
    TokenId token = (TokenId) variables[0]->parent();
    TestMonitor::registerCondition(token->getKey(), token->toString(), shouldBeCompleted);
  }

  TestMonitorConstraintBase::~TestMonitorConstraintBase(){
  }

  CompletionMonitorConstraint::CompletionMonitorConstraint(const LabelStr& name,
							   const LabelStr& propagatorName,
							   const ConstraintEngineId& constraintEngine,
							   const std::vector<ConstrainedVariableId>& variables)
    : TestMonitorConstraintBase(name, propagatorName, constraintEngine, variables, true){}

  RejectionMonitorConstraint::RejectionMonitorConstraint(const LabelStr& name,
							 const LabelStr& propagatorName,
							 const ConstraintEngineId& constraintEngine,
							 const std::vector<ConstrainedVariableId>& variables)
    : TestMonitorConstraintBase(name, propagatorName, constraintEngine, variables, false){}

  TestMonitor::Entry::Entry(int key_, const std::string& label_, bool expectedValue_)
    :key(key_), label(label_), expectedValue(expectedValue_), actualValue(!expectedValue_), resolved(false){}

  bool TestMonitor::success(){
    // Iterate over all entries. Each one must be resolved and the expected and actual values must match
    for(std::list<TestMonitor::Entry>::const_iterator it = entries().begin(); it != entries().end(); ++it){
      const Entry& entry = *it;
      if(!entry.resolved || entry.expectedValue != entry.actualValue)
	return false;
    }

    return true;
  }

  void TestMonitor::reset(){
    entries().clear();
  }

  std::string TestMonitor::toString(){
    std::stringstream ss;
    for(std::list<TestMonitor::Entry>::const_iterator it = entries().begin(); it != entries().end(); ++it){
      const Entry& entry = *it;
      ss << (entry.resolved && entry.expectedValue == entry.actualValue ? "SUCCESS" : "FAILURE") << " for ";
      ss << entry.label << "(";
      ss << (entry.resolved ? "resolved" : "unresolved") << ", ";
      ss << (entry.expectedValue ? "expected to succeed" : " expected to fail") << ", ";
      ss << (entry.actualValue ? " suceeded" : "failed") << ")\n";
    }

    return ss.str();
  }

  void TestMonitor::registerCondition(int key, const std::string& label, bool expectedValue){
    RETURN_IF_NO_EVAL;
    // If there are no entries, then register a listener. The listener will be cleared up when the agent is deleted
    if(entries().empty())
      new TestMonitor::AgentListener();

    // Now add the entry
    entries().push_back(TestMonitor::Entry(key, label, expectedValue));
  }

  void TestMonitor::updateValue(int key, bool completed){
    for(std::list< Entry >::iterator it = entries().begin(); it != entries().end(); ++it){
      Entry& entry = *it;
      if(entry.key == key){
	entry.actualValue = completed;
	entry.resolved = true;
      }
    }
  }

  bool TestMonitor::isCondition(int key){
    for(std::list< Entry >::iterator it = entries().begin(); it != entries().end(); ++it){
      Entry& entry = *it;
      if(entry.key == key)
	return true;
    }

    return false;
  }

  void TestMonitor::AgentListener::notifyRejected(const TokenId& token){
    TestMonitor::updateValue(token->getKey(), false);
  }

  void TestMonitor::AgentListener::notifyCompleted(const TokenId& token){
    TestMonitor::updateValue(token->getKey(), true);
  }
}
