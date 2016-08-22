/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2009, Willow Garage.
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

#include "Functions.hh"
#include "Utilities.hh"
#include "Token.hh"
#include "Schema.hh"
#include "PlanDatabase.hh"
#include "TokenVariable.hh"
#include "Object.hh"
#include "LabelStr.hh"
#include "DbCore.hh"
#include "Timeline.hh"

using namespace EUROPA;

namespace TREX {
  /*!< Useful constants */
  const LabelStr ExecutionConstraint::SUCCESS("SUCCESS");
  const LabelStr ExecutionConstraint::PREEMPTED("PREEMPTED");
  const LabelStr ExecutionConstraint::ABORTED("ABORTED");
  const LabelStr ExecutionConstraint::ACTION_ACTIVE("AgentAction.Active");
  const LabelStr ExecutionConstraint::ACTION_INACTIVE("AgentAction.Inactive");
  const LabelStr ExecutionConstraint::PARAM_MAX_DURATION("max_duration");
  const LabelStr ExecutionConstraint::PARAM_STATUS("status");
  const LabelStr ExecutionConstraint::EQUALS("equals");
  const LabelStr ExecutionConstraint::CONTAINS("contains");
  const LabelStr ExecutionConstraint::ENDS("ends");

  ExecutionConstraint::ExecutionConstraint(const LabelStr& name,
				       const LabelStr& propagatorName,
				       const ConstraintEngineId& constraintEngine,
				       const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables) {}

  ExecutionFunction::ExecutionFunction(const LabelStr& name,
				       const LabelStr& propagatorName,
				       const ConstraintEngineId& constraintEngine,
				       const std::vector<ConstrainedVariableId>& variables)
    : ExecutionConstraint(name, propagatorName, constraintEngine, makeScope(variables)),
      m_result(static_cast<BoolDomain&>(getCurrentDomain(getScope()[0]))),
      m_clock(static_cast<const IntervalIntDomain&>(getScope()[1]->lastDomain())),
      m_start(static_cast<const IntervalIntDomain&>(getScope()[2]->lastDomain())),
      m_end(static_cast<const IntervalIntDomain&>(getScope()[3]->lastDomain())),
      m_max_duration(static_cast<IntervalIntDomain&>(getCurrentDomain(getScope()[4]))),
      m_token(getParentToken(getScope()[2])){
  }

  std::vector<ConstrainedVariableId> ExecutionFunction::makeScope(const std::vector<ConstrainedVariableId>& variables){
    checkError(variables.size() >= 2, "There must be at least 2 variables in the scope of this constraint. Instead there are only " << variables.size());

    // If the variable set is greater than 2 then we have already constructed the scope. This can be the case when constraints
    // are being migrated for example
    if(variables.size() > 2)
      return variables;

    ConstrainedVariableId result_var = variables[0];
    ConstrainedVariableId token_var = variables[1];

    TokenId parent_token = getParentToken(token_var);

    checkError(parent_token.isId(),
	       "We assume the second variable is a variable of a AgentAction::Active token, but it is not. " << token_var->toLongString());

    checkError(parent_token->getPlanDatabase()->getSchema()->isA(parent_token->getPredicateName(), ACTION_ACTIVE),
	       "We assume the second variable is a variable of a AgentAction::Active token, but it is not. " << parent_token->toLongString());

    checkError(parent_token->getVariable(PARAM_MAX_DURATION, false).isId(), "Must have a max_duration parameter." << parent_token->toLongString());

    // Now build the new scope.
    std::vector<ConstrainedVariableId> new_scope;
    new_scope.push_back(result_var);
    new_scope.push_back(DbCore::getAgentClockVariable(parent_token->getPlanDatabase()));
    new_scope.push_back(parent_token->start());
    new_scope.push_back(parent_token->end());
    new_scope.push_back(parent_token->getVariable(PARAM_MAX_DURATION, false));
    new_scope.push_back(parent_token->getState());
    return new_scope;
  }

  bool ExecutionFunction::isStarted() const{
    return m_token->isActive() && m_start.isSingleton() && m_start.getUpperBound() <= m_clock.getLowerBound();
  }

  bool ExecutionFunction::isEnded(){
    return m_token->isCommitted() && m_end.isSingleton() && m_end.getUpperBound() <= m_clock.getLowerBound() && hasStatus();
  }

  bool ExecutionFunction::isStatus(const LabelStr& status) {
    return status == m_status;
  }

  /**
   * This function is very important to make sure that we do not propagate values until we actually have a valid status variable.
   * To do this, we verify that the timeline in question has a successor token with a status value bound. Also, this token must no
   * longer be current (i.e. not a current observation)
   */
  bool ExecutionFunction::hasStatus(){
    if(m_status == EMPTY_LABEL()){

      // Obtain the successor token.
      TimelineId timeline = m_token->getObject()->lastDomain().getSingletonValue();
      DbCoreId db_core = DbCore::getInstance(m_token);
      TokenId successor_token = db_core->getValue(timeline, m_end.getUpperBound());

      if(successor_token.isId()){

	checkError(successor_token->getPlanDatabase()->getSchema()->isA(successor_token->getPredicateName(), ACTION_INACTIVE), 
		   "Token is out of place. " << successor_token->toLongString());

	const AbstractDomain& successor_status = getCurrentDomain(successor_token->getVariable(PARAM_STATUS, false));

	// Should no longer be current
	if(successor_status.isSingleton() && !db_core->isCurrentObservation(m_token))
	  m_status = successor_status.getSingletonValue();
      }
    }

    return m_status != EMPTY_LABEL();
  }


  IsStarted::IsStarted(const LabelStr& name,
			 const LabelStr& propagatorName,
			 const ConstraintEngineId& constraintEngine,
			 const std::vector<ConstrainedVariableId>& variables)
    : ExecutionFunction(name, propagatorName, constraintEngine, variables){} 

  void IsStarted::handleExecute(){
    if(isStarted())
      m_result.set(1);
  }

  IsEnded::IsEnded(const LabelStr& name,
			 const LabelStr& propagatorName,
			 const ConstraintEngineId& constraintEngine,
			 const std::vector<ConstrainedVariableId>& variables)
    : ExecutionFunction(name, propagatorName, constraintEngine, variables){} 

  void IsEnded::handleExecute(){
    if(isEnded())
      m_result.set(1);
  }

  IsTimedOut::IsTimedOut(const LabelStr& name,
			 const LabelStr& propagatorName,
			 const ConstraintEngineId& constraintEngine,
			 const std::vector<ConstrainedVariableId>& variables)
    : ExecutionFunction(name, propagatorName, constraintEngine, variables), m_fired(false){} 

  /**
   * @brief We only do evaluation once the token has started. We immediately commit to the result based on the bounds.
   */
  void IsTimedOut::handleExecute(){
    if(!isStarted())
      return;

    if(!m_fired){
      m_fired = true;
      if( (m_end.getLowerBound() - m_start.getUpperBound()) > m_max_duration.getUpperBound())
	 m_result.set(1);
      else
	m_result.set(0);
    }
  }

  void IsTimedOut::setSource(const ConstraintId& source_constraint){
    IsTimedOut* c = (IsTimedOut*) source_constraint;
    checkError(c != NULL, "Invalid cast from source constraint " << source_constraint->toString());
    m_fired = c->m_fired;
  }

  IsStatus::IsStatus(const LabelStr& name,
			 const LabelStr& propagatorName,
			 const ConstraintEngineId& constraintEngine,
			 const std::vector<ConstrainedVariableId>& variables)
    : ExecutionFunction(name, propagatorName, constraintEngine, variables){} 

  void IsStatus::handleExecute(){
    if(isEnded()){
      if(checkStatus()){
	m_result.set(1);
      }
      else{
	m_result.set(0);
      }

      if(m_result.isEmpty())
	return;

      // Restrict the base domain of the result, since we hae gone to great lengths to make sure that
      // the source variables are committed, and not subject to change, in which case this is just a function. If
      // a conflict arises, the recovery logic of the core will have to apply
      getScope()[0]->restrictBaseDomain(m_result);
    }
  }

  IsSucceded::IsSucceded(const LabelStr& name,
			 const LabelStr& propagatorName,
			 const ConstraintEngineId& constraintEngine,
			 const std::vector<ConstrainedVariableId>& variables)
    : IsStatus(name, propagatorName, constraintEngine, variables){} 

  bool IsSucceded::checkStatus() {return isStatus(SUCCESS); }

  IsAborted::IsAborted(const LabelStr& name,
			 const LabelStr& propagatorName,
			 const ConstraintEngineId& constraintEngine,
			 const std::vector<ConstrainedVariableId>& variables)
    : IsStatus(name, propagatorName, constraintEngine, variables){} 

  bool IsAborted::checkStatus() {return isStatus(ABORTED); }

  IsPreempted::IsPreempted(const LabelStr& name,
			 const LabelStr& propagatorName,
			 const ConstraintEngineId& constraintEngine,
			 const std::vector<ConstrainedVariableId>& variables)
    : IsStatus(name, propagatorName, constraintEngine, variables){} 

  bool IsPreempted::checkStatus() {return isStatus(PREEMPTED); }

  MasterSlaveRelation::MasterSlaveRelation(const LabelStr& name,
				       const LabelStr& propagatorName,
				       const ConstraintEngineId& constraintEngine,
				       const std::vector<ConstrainedVariableId>& variables)
    : ExecutionConstraint(name, propagatorName, constraintEngine, makeScope(variables)),
      m_token(getParentToken(variables[0])),
      m_relation(m_token->getRelation()){
    checkError(m_token->getPlanDatabase()->getSchema()->isA(m_token->getPredicateName(), ACTION_ACTIVE),
	       "This constraint must be an active behavior token. " << m_token->toString());

    debugMsg("trex:extensions:MasterSlaveRelation", m_token->toString());
  }

  /**
   * The variable belongs to a token. Pull that token and build relationships from there
   */
  std::vector<ConstrainedVariableId> MasterSlaveRelation::makeScope(const std::vector<ConstrainedVariableId>& variables){
    checkError(variables.size() == 1 || variables.size() >= 7, 
	       "There will be either 1 variable or 8 variables. For the latter, we must be copying a constraint." << variables.size());

    // If the variable set is greater than 1 then we have already constructed the scope.
    if(variables.size() > 1)
      return variables;

    TokenId token = getParentToken(variables[0]);
    TokenId master = token->master();
    // Now build the new scope if we have a master
    if(master.isId()){
      std::vector<ConstrainedVariableId> new_scope;
      new_scope.push_back(token->start());
      new_scope.push_back(token->end());
      new_scope.push_back(token->duration());
      new_scope.push_back(token->getVariable(PARAM_MAX_DURATION, false));
      
      new_scope.push_back(master->start());
      new_scope.push_back(master->end());
      new_scope.push_back(master->duration());
      
      // Only obtain max duration if parent is also a behavior
      if(master->getPlanDatabase()->getSchema()->isA(master->getPredicateName(), ACTION_ACTIVE))
	new_scope.push_back(master->getVariable(PARAM_MAX_DURATION, false));

      return new_scope;
    }

    return variables;    
  }
    
  /**
   * The implementation assumes that the master token projects a latest end time desired, based on the master
   * end time and the master max duration bound.
   */
  void MasterSlaveRelation::handleExecute(){
    if(getScope().size() >= 7){

      condDebugMsg(m_token->master().isId(), "trex:extensions:MasterSlaveRelation:handleExecute", 
	       "Evaluating relation <" << m_relation.toString() << "> between master " << 
		   m_token->getMaster()->toString() << " and slave " << m_token->toString());

      if(m_relation == EQUALS || m_relation == CONTAINS || m_relation == ENDS){
	const IntervalIntDomain& start = static_cast<const IntervalIntDomain&>(getCurrentDomain(getScope()[0]));
	const IntervalIntDomain& end =  static_cast<const IntervalIntDomain&>(getCurrentDomain(getScope()[1]));
	IntervalIntDomain& max_duration =  static_cast<IntervalIntDomain&>(getCurrentDomain(getScope()[3]));

	const IntervalIntDomain& master_start =  static_cast<const IntervalIntDomain&>(getCurrentDomain(getScope()[4])); 
	const IntervalIntDomain& master_end =  static_cast<const IntervalIntDomain&>(getCurrentDomain(getScope()[5]));
	const IntervalIntDomain& master_duration =  static_cast<const IntervalIntDomain&>(getCurrentDomain(getScope()[6])); 

	int master_max_duration = master_duration.getUpperBound();
	// If we have a parent that is a behavior, we have the parents max duration parameter
	if(getScope().size() == 8)
	  master_max_duration = (int) getCurrentDomain(getScope()[7]).getUpperBound();

	int latest_end_time = std::min(master_start.getUpperBound() + master_max_duration, end.getUpperBound());
	latest_end_time = std::min(latest_end_time, (int) master_end.getUpperBound());
	int max_duration_ub = latest_end_time - start.getLowerBound();
	max_duration.intersect(MINUS_INFINITY, max_duration_ub);
      }
    }
  }
}
