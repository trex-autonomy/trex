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

#include "DbSolver.hh"
#include "ComponentFactory.hh"
#include "Utils.hh"
#include "Token.hh"
#include "Context.hh"
#include "FlawFilter.hh"

namespace TREX {
  
  DbSolver::DbSolver(const PlanDatabaseId& db, TiXmlElement* solverCfg): m_db(db) {
    if (!solverCfg->Attribute("composite")) {
      AbstractSolverId solver = (new EuropaSolverAdapter(*solverCfg))->getId();
      solver->init(db, solverCfg);
      m_solvers.push_back(solver);
    } else {
      for (TiXmlElement* child = solverCfg->FirstChildElement(); child;
	   child = child->NextSiblingElement()) {
	ComponentFactoryMgr* cfm = (ComponentFactoryMgr*)db->getEngine()->getComponent("ComponentFactoryMgr");
	AbstractSolverId solver =  cfm->createInstance(*child);
	solver->init(db, child);
	m_solvers.push_back(solver);
      }
    }
  }

  DbSolver::~DbSolver() {
    cleanup(m_solvers);
  }



  bool DbSolver::isExhausted() {
    for (std::vector<AbstractSolverId>::iterator it = m_solvers.begin(); 
	 it != m_solvers.end(); it++) {
      AbstractSolverId solver = *it;
      if (solver->isExhausted()) {
	return true;
      }
    }
    return false;
  }

  unsigned int DbSolver::getDepth() {
    unsigned int ans = 0;
    for (std::vector<AbstractSolverId>::iterator it = m_solvers.begin(); 
	 it != m_solvers.end(); it++) {
      AbstractSolverId solver = *it;
      ans += solver->getDepth();
    }
    return ans;
  }
  unsigned int DbSolver::getStepCount() {
    unsigned int ans = 0;
    for (std::vector<AbstractSolverId>::iterator it = m_solvers.begin(); 
	 it != m_solvers.end(); it++) {
      AbstractSolverId solver = *it;
      ans += solver->getStepCount();
    }
    return ans;
  }
  
  void DbSolver::step() {
    for (std::vector<AbstractSolverId>::iterator it = m_solvers.begin(); 
	 it != m_solvers.end(); it++) {
      AbstractSolverId solver = *it;
      if (!solver->noMoreFlaws()) {
	solver->step();
	return;
      }
    }
  }

  bool DbSolver::noMoreFlaws() {
    for (std::vector<AbstractSolverId>::iterator it = m_solvers.begin(); 
	 it != m_solvers.end(); it++) {
      AbstractSolverId solver = *it;
      if (!solver->noMoreFlaws()) {
	return false;
      }
    }
    return true;
  }

  void DbSolver::clear() {
    for (std::vector<AbstractSolverId>::iterator it = m_solvers.begin(); 
	 it != m_solvers.end(); it++) {
      AbstractSolverId solver = *it;
      solver->clear();
    }
  }

  void DbSolver::reset() {
    for (std::vector<AbstractSolverId>::iterator it = m_solvers.begin(); 
	 it != m_solvers.end(); it++) {
      AbstractSolverId solver = *it;
      solver->reset();
    }
  }

  bool DbSolver::inDeliberation(const EntityId& entity) const{
    for (std::vector<AbstractSolverId>::const_iterator it = m_solvers.begin(); it != m_solvers.end(); it++) {
      AbstractSolverId solver = *it;
      if(solver->inDeliberation(entity))
	return true;
    }
    return false;
  }

  AbstractSolver::AbstractSolver(const TiXmlElement& cfgXml) {
    // Extract the name of the Solver
    m_name = extractData(cfgXml, "name");

    // Allocate the context
    m_context = ((new Context(m_name.toString() + "Context"))->getId());
  }

  AbstractSolver::~AbstractSolver() {
    delete (Context*) m_context;
  }

  EuropaSolverAdapter::EuropaSolverAdapter(const TiXmlElement& cfgXml) : AbstractSolver(cfgXml) { }
  EuropaSolverAdapter::~EuropaSolverAdapter() { delete (Solver*)m_solver; }
  void EuropaSolverAdapter::init(PlanDatabaseId db, TiXmlElement* cfgXml) { 
    m_solver = (new Solver(db, *cfgXml))->getId();
  }
  bool EuropaSolverAdapter::isExhausted() { 
    return m_solver->isExhausted(); 
  }
  void EuropaSolverAdapter::step() {
    return m_solver->step();
  }
  unsigned int EuropaSolverAdapter::getDepth() {
    return m_solver->getDepth(); 
  }
  unsigned int EuropaSolverAdapter::getStepCount() {
    return m_solver->getStepCount(); 
  }
  bool EuropaSolverAdapter::noMoreFlaws() { 
    IteratorId id = m_solver->createIterator();
    bool ret = m_solver->noMoreFlaws() && id->done();
    delete (FlawIterator*)id;
    return ret;
  }
  void EuropaSolverAdapter::clear() {
    return m_solver->clear();
  }
  void EuropaSolverAdapter::reset() { 
    return m_solver->reset();
  }
    
  bool EuropaSolverAdapter::inDeliberation(const EntityId& entity) const {
    const DecisionStack& decision_stack = m_solver->getDecisionStack();
    for(DecisionStack::const_iterator it = decision_stack.begin(); it != decision_stack.end(); ++it){
      DecisionPointId decision_point = *it;
      if(entity->getKey() == (int) decision_point->getFlawedEntityKey())
	return true;
    }

    return false;
  }

  FlawManagerSolver::FlawManagerSolver(const TiXmlElement& cfgXml) : AbstractSolver(cfgXml), m_dbListener(NULL), m_ceListener(NULL) { }

  FlawManagerSolver::~FlawManagerSolver() {
    cleanup(m_flawManagers);
    if (m_dbListener) {
      delete m_dbListener;
    }
    if (m_ceListener) {
      delete m_ceListener;
    }
  }
  void FlawManagerSolver::initDbListener(PlanDatabaseId db, TiXmlElement* cfgXml) {
    m_dbListener = new DbListener(db, *this);
  }
  void FlawManagerSolver::initCeListener(PlanDatabaseId db, TiXmlElement* cfgXml) {
    m_ceListener = new CeListener(db->getConstraintEngine(), *this);
  }
  void FlawManagerSolver::initFlawManagers(PlanDatabaseId db, TiXmlElement* cfgXml) {
    AbstractSolver::init(db, cfgXml);
    ComponentFactoryMgr* cfm = (ComponentFactoryMgr*)m_db->getEngine()->getComponent("ComponentFactoryMgr");
    for (TiXmlElement* child = cfgXml->FirstChildElement(); child;
	 child = child->NextSiblingElement()) {
      FlawManagerId manager = cfm->createInstance(*child)->getId();
      manager->initialize(*child, m_db, m_context, FlawManagerId::noId());
      m_flawManagers.push_back(manager);
    } 
  }
  void FlawManagerSolver::notifyAdded(const TokenId& token) {
    debugMsg("FlawManagerSolver", "FMS Add token: " << token->toString());
    for(std::vector<FlawManagerId>::const_iterator it = m_flawManagers.begin(); it != m_flawManagers.end(); ++it) {
      (*it)->notifyAdded(token);
    }
  }
  void FlawManagerSolver::notifyRemoved(const TokenId& token) {
    debugMsg("FlawManagerSolver", "FMS Remove token: " << token->toString());
    for(std::vector<FlawManagerId>::const_iterator it = m_flawManagers.begin(); it != m_flawManagers.end(); ++it) {
      (*it)->notifyRemoved(token);
    }
  }
  FlawManagerSolver::DbListener::DbListener(const PlanDatabaseId& db, FlawManagerSolver& solver)
    : PlanDatabaseListener(db), m_solver(solver) {}
  
  void FlawManagerSolver::DbListener::notifyRemoved(const TokenId& token) {
    debugMsg("FlawManagerSolver", "Remove token: " << token->toString());
    m_solver.notifyRemoved(token);
  }
  void FlawManagerSolver::DbListener::notifyAdded(const TokenId& token) {
    debugMsg("FlawManagerSolver", "Add token: " << token->toString());
    m_solver.notifyAdded(token);
  }
  FlawManagerId FlawManagerSolver::getFlawManager(unsigned int i) {
    return m_flawManagers[i];
  }
  unsigned int FlawManagerSolver::getFlawManagerCount() {
    return m_flawManagers.size();
  }
  FlawManagerSolver::CeListener::CeListener(const ConstraintEngineId& ce, FlawManagerSolver& solver)
    : ConstraintEngineListener(ce), m_solver(solver) {}
  void FlawManagerSolver::CeListener::notifyRemoved(const ConstraintId& variable){
    m_solver.notifyRemoved(variable);
  }
  void FlawManagerSolver::CeListener::notifyRemoved(const ConstrainedVariableId& variable){
    m_solver.notifyRemoved(variable);
  }
  void FlawManagerSolver::CeListener::notifyChanged(const ConstrainedVariableId& variable, 
					 const DomainListener::ChangeType& changeType){
    m_solver.notifyChanged(variable, changeType);
  }
  void FlawManagerSolver::CeListener::notifyAdded(const ConstraintId& constraint){
    m_solver.notifyAdded(constraint);
  }
  void FlawManagerSolver::notifyRemoved(const ConstraintId& variable) {
    debugMsg("FlawManagerSolver", "FMS Remove constriant: " << variable->toString());
    for(std::vector<FlawManagerId>::const_iterator it = m_flawManagers.begin(); it != m_flawManagers.end(); ++it) {
      (*it)->notifyRemoved(variable);
    }
  }
  void FlawManagerSolver::notifyRemoved(const ConstrainedVariableId& variable) {
    debugMsg("FlawManagerSolver", "FMS Remove variable: " << variable->toString());
    for(std::vector<FlawManagerId>::const_iterator it = m_flawManagers.begin(); it != m_flawManagers.end(); ++it) {
      (*it)->notifyRemoved(variable);
    }
  }
  void FlawManagerSolver::notifyChanged(const ConstrainedVariableId& variable, 
					 const DomainListener::ChangeType& changeType) {
    switch(changeType){
    case DomainListener::UPPER_BOUND_DECREASED:
    case DomainListener::LOWER_BOUND_INCREASED:
    case DomainListener::VALUE_REMOVED:
    case DomainListener::EMPTIED:
      return;
    default:
      debugMsg("FlawManagerSolver", "FMS Change variable ("
	       << DomainListener::toString(changeType) << "): " << variable->toString());
      for(std::vector<FlawManagerId>::const_iterator it = m_flawManagers.begin(); it != m_flawManagers.end(); ++it) {
	(*it)->notifyChanged(variable, changeType);
      }
    }
  }
  void FlawManagerSolver::notifyAdded(const ConstraintId& variable) {
    debugMsg("FlawManagerSolver", "FMS Added constraint: " << variable->toString());
    for(std::vector<FlawManagerId>::const_iterator it = m_flawManagers.begin(); it != m_flawManagers.end(); ++it) {
      (*it)->notifyAdded(variable);
    }
  }
}






