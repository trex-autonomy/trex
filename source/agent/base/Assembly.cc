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
#include "Assembly.hh"
#include "Utilities.hh"
#include "Constraints.hh"
#include "GoalManager.hh"
#include "OrienteeringSolver.hh"
#include "GreedyOpenConditionManager.hh"
#include "DbCore.hh"
#include "DbSolver.hh"
#include "TestMonitor.hh"
#include "Interpreter.hh"
#include "Functions.hh"
#include "CFunction.hh"

#include "ModuleConstraintEngine.hh"
#include "Propagators.hh"
#include "ModulePlanDatabase.hh"
#include "ModuleRulesEngine.hh"
#include "ModuleTemporalNetwork.hh"
#include "ModuleSolvers.hh"
#include "ModuleNddl.hh"
#include "Nddl.hh"

// Support for required major plan database components
#include "PlanDatabase.hh"
#include "ConstraintEngine.hh"
#include "RulesEngine.hh"
#include "Filters.hh"

// Transactions
#include "DbClientTransactionPlayer.hh"
#include "DbClientTransactionLog.hh"
#include "NddlDefs.hh"

// Misc
#include "Utils.hh"

#include <fstream>
#include <sstream>

namespace TREX {
  void initialize() { } //Used to force the library to load.

  Assembly::Schema* Assembly::Schema::s_instance = NULL;

  Assembly::Assembly()
  {
    assertTrue(ALWAYS_FAIL, "Should never get here.");
  }

  Assembly::Assembly(const LabelStr& agentName, const LabelStr& reactorName)
    : m_agentName(agentName), m_reactorName(reactorName)
  {
    addModule((new ModuleConstraintEngine())->getId()); 
    addModule((new ModuleConstraintLibrary())->getId());
    addModule((new ModulePlanDatabase())->getId());
    addModule((new ModuleRulesEngine())->getId());
    addModule((new ModuleTemporalNetwork())->getId());
    addModule((new ModuleSolvers())->getId());
    addModule((new ModuleNddl())->getId());

    // Call base class initialization
    doStart();

    // Initialize member variables
    m_schema = ((EUROPA::Schema*)getComponent("Schema"))->getId();
    m_constraintEngine = ((ConstraintEngine*)getComponent("ConstraintEngine"))->getId();
    m_planDatabase = ((PlanDatabase*) getComponent("PlanDatabase"))->getId();
    m_rulesEngine = ((RulesEngine*) getComponent("RulesEngine"))->getId();
    m_ppw = NULL;

    // Add another propagator to handle propagation of commitment constraints. Will be scheduled last to ensure that
    // the nextwork is fully propagated before we make any commitments.
    new DefaultPropagator(LabelStr("OnCommit"), m_constraintEngine->getId());

    // Disable auto propagation
    m_constraintEngine->setAutoPropagation(false);

    // Register components
    Assembly::Schema* s = Assembly::Schema::instance();
    s->registerComponents(*this);

    // Finally, set the domain comparator explicitly
    DomainComparator::setComparator((EUROPA::Schema*) m_schema);
  }

  Assembly::~Assembly() 
  {  
    if(m_ppw == NULL)
      delete m_ppw;

    doShutdown(); 
  }

  bool Assembly::playTransactions(const char* txSource)
  {
    check_error(txSource != NULL, "NULL transaction source provided.");
    static bool isFile(true);

    std::ifstream f1(findFile("NDDL.cfg").c_str());
    std::ifstream f2(findFile("temp_nddl_gen.cfg").c_str());
    TiXmlElement* iroot = NULL;
    if (f1.good()) {
      iroot = EUROPA::initXml(findFile("NDDL.cfg").c_str());
    } else if (f2.good()) {
      iroot = EUROPA::initXml(findFile("temp_nddl_gen.cfg").c_str());
    } else {
      checkError(false, "Could not find 'NDDL.cfg' or 'temp_nddl_gen.cfg'");
    }
    if (iroot) {
      for (TiXmlElement * ichild = iroot->FirstChildElement();
	   ichild != NULL;
	   ichild = ichild->NextSiblingElement()) {
	if (std::string(ichild->Value()) == "include") {
	  std::string path = std::string(ichild->Attribute("path"));
	  for (unsigned int i = 0; i < path.size(); i++) {
	    if (path[i] == ';') {
	      path[i] = ':';
	    }
	  }
	  getLanguageInterpreter("nddl")->getEngine()->getConfig()->setProperty("nddl.includePath", path);
	}
      }
    }
    try {
      std::string ret = executeScript("nddl", txSource, isFile);
      assertTrue(ret == "", "Parser failed in " + std::string(txSource) + " with return: " + ret);
    } catch(std::string ex) {
      assertTrue(false, "Parser failed: " + ex);
    } catch(...) {
      assertTrue(false, "Parser failed with unknown exception reading " + std::string(txSource));
    }

    return m_constraintEngine->constraintConsistent();
  }

  const std::string& Assembly::exportToPlanWorks(TICK tick, unsigned int attempt){
    static const std::string sl_reply("DONE");

    // The assembly might be getting dumped without being propagated
    //if(attempt == 0) {
    //  checkError(m_constraintEngine->constraintConsistent(), "Should be propagated.");
    //}

    // Write out the data
    getPPW()->write(tick,attempt);

    return sl_reply;
  }

  DbWriter* Assembly::getPPW(){
    if(m_ppw == NULL){
      m_ppw = new DbWriter(m_agentName.toString(), m_reactorName.toString(), m_planDatabase, m_constraintEngine, m_rulesEngine);
    }

    return m_ppw;
  }

  Assembly::SchemaPlugIn::SchemaPlugIn(){
    Assembly::Schema::instance()->registerPlugIn(this);
  }

  Assembly::SchemaPlugIn::~SchemaPlugIn(){
    //Assembly::Schema::instance()->unregisterPlugIn(this);
  }


  Assembly::Schema::Schema(){
    if(s_instance != NULL)
      delete s_instance;

    s_instance = this;
  }

  Assembly::Schema::~Schema(){
    s_instance = NULL;
  }

  Assembly::Schema* Assembly::Schema::instance(){
    if(s_instance == NULL)
      new Assembly::Schema();

    return s_instance;
  }

  void Assembly::Schema::registerPlugIn(SchemaPlugIn* plugIn){
    m_plugIns.push_back(plugIn);
  }

  void Assembly::Schema::unregisterPlugIn(SchemaPlugIn* plugIn){
    std::vector<SchemaPlugIn*> old(m_plugIns);
    m_plugIns.clear();
    for(unsigned int i = 0; i < old.size(); i++){
      if(old[i] != plugIn)
	m_plugIns.push_back(old[i]);
    }
  }

#define DECLARE_FUNCTION_TYPE(cname, fname, constraint, type, args)	\
  class cname##Function : public CFunction				\
  {									\
  public:								\
    cname##Function() : CFunction(#fname) {}				\
    virtual ~cname##Function() {}					\
    									\
    virtual const char* getConstraint() { return constraint; }		\
    virtual const DataTypeId getReturnType() { return type::instance(); } \
    virtual unsigned int getArgumentCount() { return args; }		\
    virtual void checkArgTypes(const std::vector<DataTypeId>& argTypes) {} \
  };

  DECLARE_FUNCTION_TYPE(IsStarted, isStarted, "isStarted", BoolDT, 1);
  DECLARE_FUNCTION_TYPE(IsEnded, isEnded, "isEnded", BoolDT, 1);
  DECLARE_FUNCTION_TYPE(IsTimedOut, isTimedOut, "isTimedOut", BoolDT, 1);
  DECLARE_FUNCTION_TYPE(IsSucceded, isSucceded, "isSucceded", BoolDT, 1);
  DECLARE_FUNCTION_TYPE(IsAborted, isAborted, "isAborted", BoolDT, 1);
  DECLARE_FUNCTION_TYPE(IsPreempted, isPreempted, "isPreempted", BoolDT, 1);

  void Assembly::Schema::registerComponents(const Assembly& assembly){

    ConstraintEngineId constraintEngine = ((ConstraintEngine*) assembly.getComponent("ConstraintEngine"))->getId();
    checkError(constraintEngine.isValid(), "No ConstraintEngine registered");

    // Register functions
    constraintEngine->getCESchema()->registerCFunction((new IsStartedFunction())->getId());
    constraintEngine->getCESchema()->registerCFunction((new IsEndedFunction())->getId());
    constraintEngine->getCESchema()->registerCFunction((new IsTimedOutFunction())->getId());
    constraintEngine->getCESchema()->registerCFunction((new IsSuccededFunction())->getId());
    constraintEngine->getCESchema()->registerCFunction((new IsAbortedFunction())->getId());
    constraintEngine->getCESchema()->registerCFunction((new IsPreemptedFunction())->getId());

    // Create User-defined stuff.
    for (unsigned int i = 0; i < m_plugIns.size(); i++) {
      m_plugIns[i]->registerComponents(assembly);
    }
  }

  class AssemblyPlugIn: public Assembly::SchemaPlugIn{
    void registerComponents(const Assembly& assembly){
      // Register constraints
      TREX_REGISTER_CONSTRAINT(assembly, SetDefaultOnCommit, defaultOnCommit, OnCommit);
      TREX_REGISTER_CONSTRAINT(assembly, AbsMaxOnCommit, absMaxOnCommit, OnCommit);
      TREX_REGISTER_CONSTRAINT(assembly, SetDefault, default, Default);
      TREX_REGISTER_CONSTRAINT(assembly, SetDefault, bind, Default);
      TREX_REGISTER_CONSTRAINT(assembly, LessThanConstraint, lt, Default);
      TREX_REGISTER_CONSTRAINT(assembly, TestLessThan, testLT, Default);
      TREX_REGISTER_CONSTRAINT(assembly, Neighborhood, neighborhood, Default);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::CompletionMonitorConstraint, assertCompleted, Default);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::RejectionMonitorConstraint, assertRejected, Default);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::IsStarted, isStarted, Default);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::IsEnded, isEnded, Default);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::IsTimedOut, isTimedOut, Default);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::IsSucceded, isSucceded, Default);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::IsAborted, isAborted, Default);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::IsPreempted, isPreempted, Default);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::MasterSlaveRelation, trex_behavior, Default);

      // Solver component registration
      TREX_REGISTER_FLAW_FILTER(assembly, TREX::GoalsOnlyFilter, GoalsOnly);
      TREX_REGISTER_FLAW_FILTER(assembly, TREX::NoGoalsFilter, NoGoals);
      TREX_REGISTER_FLAW_FILTER(assembly, TREX::DynamicGoalFilter, DynamicGoalFilter);
      TREX_REGISTER_FLAW_MANAGER(assembly, TREX::GoalManager, GoalManager);
      TREX_REGISTER_FLAW_MANAGER(assembly, TREX::GreedyOpenConditionManager, GreedyOpenConditionManager);
      TREX_REGISTER_COMPONENT_FACTORY(assembly, TREX::EuclideanCostEstimator, EuclideanCostEstimator);
      TREX_REGISTER_COMPONENT_FACTORY(assembly, TREX::OrienteeringSolver, OrienteeringSolver);
      TREX_REGISTER_COMPONENT_FACTORY(assembly, TREX::EuropaSolverAdapter, EuropaSolverAdapter);
      TREX_REGISTER_FLAW_FILTER(assembly, DeliberationFilter, DeliberationFilter);
      TREX_REGISTER_FLAW_FILTER(assembly, SOLVERS::SingletonFilter, NotSingletonGuard);
      TREX_REGISTER_COMPONENT_FACTORY(assembly, TREX::TestConditionHandler, TestConditionHandler);
    }
  };

  TREX_REGISTER_SCHEMA_PLUGIN(AssemblyPlugIn);
}
