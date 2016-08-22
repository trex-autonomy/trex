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

/**
 * @author Conor McGann
 * @file DbCore implementation.
 */


#include "DbCore.hh"
#include "DbSolver.hh"
#include "Agent.hh"
#include "TeleoReactor.hh"
#include "Schema.hh"
#include "PlanDatabase.hh"
#include "PlanDatabaseWriter.hh"
#include "Timeline.hh"
#include "Debug.hh"
#include "Server.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Observer.hh"
#include "DbClient.hh"
#include "UnboundVariableDecisionPoint.hh"
#include "FlawHandler.hh"
#include "RuleInstance.hh"
#include "RuleVariableListener.hh"
#include "XMLUtils.hh"
#include "TemporalAdvisor.hh"
#include "Utilities.hh"
#include "Filters.hh"
#include "TestMonitor.hh"

// For fileio
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace TREX {

  /* IMPLEMENTATION FOR DELIBERATION FILTER */

  DeliberationFilter::DeliberationFilter(const TiXmlElement& configData): FlawFilter(configData, true) {}

  /**
   * This method implements the horizon policy for deliberation.
   */
  bool DeliberationFilter::test(const EntityId& entity){
    static unsigned int sl_counter(0);

    TokenId token;

    if(ConstrainedVariableId::convertable(entity)){
      ConstrainedVariableId var = entity;

      // Exclude if already a singleton
      if(var->lastDomain().isSingleton())
	return true;

      EntityId parent = var->parent();

      // Exclude variable if global or contained by an object
      if(parent.isNoId() || ObjectId::convertable(parent))
	return true;

      if(RuleInstanceId::convertable(parent))
	token = RuleInstanceId(parent)->getToken();
      else
	token = parent;

      // If ever the parent token is inactive then the variable is not deciable
      if(!token->isActive())
	return true;
    }
    else
      token = entity;

    if(m_core.isNoId()){
      m_core = DbCore::getInstance(token);
    }

    TREX_INFO("trex:debug:planning", "[" << sl_counter++ << "] Evaluating " << tokenToString(token) << " with " << 
	     token->start()->lastDomain().toString() << " AND " << 
	     token->end()->lastDomain().toString());

    // Cheap check if token is out of scope due to being ignored
    if(m_core.isId() && !m_core->inScope(token)){
      TREX_INFO("trex:debug:planning", "Excluding " << tokenToString(token)); 
      return true;
    }

    // Now focus on the token
    const IntervalIntDomain& startTime = token->start()->lastDomain();
    const IntervalIntDomain& endTime = token->end()->lastDomain();

    // If definitely out of temporal scope, get rid of it
    //if(startTime.getLowerBound() > horizon().getUpperBound() ||
    // endTime.getUpperBound() <= m_core->getCurrentTick())
    // return true;

    // Special Treatment if it is in fact a condition (for now called a TestMonitor). Cheap check first
    if(TestMonitor::isCondition(token->getKey())){
      TREX_INFO("trex:debug:planning:flawfiltering", "Evaluating test condition " << tokenToString(token));
      bool inScope = startTime.getUpperBound() <= m_core->getCurrentTick() && 
	startTime.isSingleton() && endTime.getUpperBound() > m_core->getCurrentTick();

      TREX_INFO("trex:debug:planning:flawfiltering", (!inScope ? "Exclude " : "Allow ") <<
		entity->toString() << " with token scope " << token->start()->lastDomain().toString() <<
		" AND " << token->end()->lastDomain().toString());

      return !inScope;
    }

    // If the token is necessarily in the past, exclude from deliberation. It will be handled in synchronization
    // or not at all if sufficiently in the past. Also, if it necessarily starts in the distant future, ignore it
    if(endTime.getUpperBound() <= (m_core.isId() ? (m_core->getCurrentTick()) : horizon().getLowerBound()) || 
       startTime.getLowerBound() >= horizon().getUpperBound()){
      TREX_INFO("trex:debug:planning:flawfiltering", "Out of horizon so excluding " <<
		entity->toString() << " with token scope " << token->start()->lastDomain().toString() <<
		" AND " << token->end()->lastDomain().toString());
      return true;
    }

    // First criteria is if the token is necessarily in the horizon. This is the first
    // check since we must plan to completion for the given horizon. Since we have already excluded tokens that are necessarily
    // in the execution frontier, we need only concern ourselves with tokens that necessrily start before the end of the horizon.
    bool inScope = startTime.getUpperBound() < horizon().getUpperBound();

    // Now we consider if the token is inevitable, and could be considered in this planning cycle. 
    // It is inevitable if it necessarily starts within the mission window.
    inScope = inScope || startTime.getUpperBound() < Agent::instance()->getFinalTick();

    // Finally, if it has a master that it could precede, since the master is already in the plan, the slave can be considered
    if(!inScope && token->master().isId())
      inScope =  DbCore::isAction(token) || !token->getPlanDatabase()->getTemporalAdvisor()->canPrecede(token->master(), token);

    TREX_INFO("trex:debug:planning", (!inScope ? "Exclude " : "Allow ") <<
		 entity->toString() << " with token scope " << token->start()->lastDomain().toString() <<
		 " AND " << token->end()->lastDomain().toString());

    return !inScope;
  }

  TICK DeliberationFilter::currentTick(){
    return (Agent::instance().isId() ? Agent::instance()->getCurrentTick() : 0);
  }

  void DeliberationFilter::update(TICK start, TICK end){
    checkError(start >= Agent::instance()->getCurrentTick(), start << " < " << Agent::instance()->getCurrentTick());
    checkError(end <= Agent::instance()->getFinalTick(), end << " > " << Agent::instance()->getCurrentTick());
    horizon() = IntervalIntDomain((int) start, (int) end);
  }

  const IntervalIntDomain& DeliberationFilter::getHorizon(){
    if(Agent::instance().isId())
      return horizon();
    else
      return HorizonFilter::getHorizon();
  }

  IntervalIntDomain& DeliberationFilter::horizon(){
    static IntervalIntDomain sl_horizon(0, PLUS_INFINITY);
    return sl_horizon;
  }

  std::ostream& DeliberationFilter::getStream(){
    if(m_core.isId())
      return m_core->getStream();
    else
      return DebugMessage::getStream();
  }

  // Allocate key collection
  std::map<int, EntityId> DbCore::s_foreignKeyRelation;

  /**
   * @brief Little utility function for composing strings to make names
   */
  LabelStr compose(const LabelStr& prefix, const LabelStr& suffix){
    std::stringstream sstr;
    sstr << prefix.toString() << "." << suffix.toString();
    return sstr.str();
  }

  TimelineContainer::TimelineContainer(const TimelineId& timeline)
    : m_timeline(timeline), m_lastObserved(0) {}

  const TimelineId& TimelineContainer::getTimeline() const {
    return m_timeline;
  }

  void TimelineContainer::updateLastObserved(TICK tick) {
    // Over-write prior value
    m_lastObserved = tick;
  }

  TICK TimelineContainer::lastObserved() const {return m_lastObserved;}

  const ServerId& TimelineContainer::getServer() const { return m_server; }

  void TimelineContainer::setServer(const ServerId& server){ m_server = server;}

  bool TimelineContainer::isDispatched(const TokenId& token){
    for(TokenSet::const_iterator it = m_dispatchedTokens.begin(); it != m_dispatchedTokens.end(); ++it){
      TokenId t = *it;
      checkError(t.isValid(), t << " is a bad id in " << m_timeline->getName().toString());
    }

    return m_dispatchedTokens.find(token) != m_dispatchedTokens.end();
  }

  const TokenSet& TimelineContainer::getDispatchedTokens() const {
    return m_dispatchedTokens;
  }

  void TimelineContainer::markDispatched(const TokenId& token){    
    m_dispatchedTokens.insert(token);
  }

  void TimelineContainer::clearDispatched(const TokenId& token){
    m_dispatchedTokens.erase(token);
  }

  void TimelineContainer::handleRemoval(const TokenId& token){
    m_dispatchedTokens.erase(token);
  }

  DbCore::DbListener::DbListener(DbCore& dbCore)
    : PlanDatabaseListener(dbCore.m_db), m_dbCore(dbCore){}

  void DbCore::DbListener::notifyAdded(const TokenId& token){ m_dbCore.handleAddition(token); }

  void DbCore::DbListener::notifyMerged(const TokenId& token){ m_dbCore.handleMerge(token); }

  void DbCore::DbListener::notifySplit(const TokenId& token){ m_dbCore.handleSplit(token); }

  void DbCore::DbListener::notifyActivated(const TokenId& token){ m_dbCore.handleActivated(token); }

  void DbCore::DbListener::notifyDeactivated(const TokenId& token){ m_dbCore.handleDeactivated(token); }

  void DbCore::DbListener::notifyRemoved(const TokenId& token){ m_dbCore.handleRemoval(token); }

  void DbCore::DbListener::notifyCommitted(const TokenId& token){ m_dbCore.handleCommitted(token); } 

  void DbCore::DbListener::notifyRejected(const TokenId& token){ m_dbCore.handleRejected(token); }

  void DbCore::DbListener::notifyTerminated(const TokenId& token){ m_dbCore.handleTerminated(token); } 

  ConstrainedVariableId DbCore::getAgentClockVariable(const PlanDatabaseId db){
    static const LabelStr VAR_AGENT_CLOCK("AGENT_CLOCK");
    ConstrainedVariableId result = db->getGlobalVariable(VAR_AGENT_CLOCK);
    checkError(result.isValid(), "Expect a clock variable named " << VAR_AGENT_CLOCK.toString() << " bot did not find it. Fix TREX.nddl.");
    return result;
  }

  DbCore::DbCore(const LabelStr& agentName, const TiXmlElement& configData)
    : TeleoReactor(agentName, configData),
      m_assembly(agentName, getName()),
      m_db(m_assembly.getPlanDatabase()),
      m_dbListener(*this),
      m_synchronizer(getId()),
      m_currentTickCycle(0),
      m_lastCompleteTick(MINUS_INFINITY),
      m_state(DbCore::INACTIVE),
      m_solverCfg(findFile(extractData(configData, "solverConfig").toString())),
      m_statePath(LogManager::instance().reactor_dir_path(agentName.toString(),getName().toString(),"reactor_states").c_str()),
      m_conflictPath(LogManager::instance().reactor_dir_path(agentName.toString(),getName().toString(),"conflicts").c_str()),
      m_planLog(LogManager::instance().reactor_file_path(agentName.toString(),getName().toString(),"plan.log").c_str()),
      m_lastRecalled(0)
  {

    DebugMessage::setStream(getStream());

    instancesByDb().insert(std::pair<PlanDatabaseId, DbCoreId>(m_db, getId()));

    const LabelStr  configFile(findFile(compose(getAgentName(), compose(getName(), "nddl")).toString()));

    LogManager::use(configFile.toString());
    m_assembly.playTransactions(configFile.c_str());

    // Close the database if not already closed. No new objects will be created
    if(!m_db->isClosed())
      m_db->close();

    // Initialize the 'missionEnd' constant with the final tick
    ConstrainedVariableId missionEndVar = m_db->getGlobalVariable("MISSION_END");
    missionEndVar->restrictBaseDomain(IntervalIntDomain(Agent::instance()->getFinalTick(), Agent::instance()->getFinalTick()));

    // Initialize the 'TICK_DURATION' constant based on the clock
    ConstrainedVariableId tickDurationVar = m_db->getGlobalVariable("TICK_DURATION");
    double tick_duration = Agent::instance()->getClock().getSecondsPerTick();
    TREX_INFO("trex:info", "Using a tick duration of " << tick_duration << " seconds.");
    tickDurationVar->restrictBaseDomain(IntervalDomain(tick_duration, tick_duration));

    // Load the solver configuration file
    TiXmlElement* solverCfg = LogManager::initXml( m_solverCfg.c_str() );
    m_solver = new DbSolver(m_db, solverCfg);
    delete solverCfg;
    checkError(m_solver.isValid(), m_solver);

    // Finally, get all inactive tokens loaded in the initial state and store them in the initial goal set. If there are any goals
    // that are not rejectable, flag an error and quit
    const TokenSet& tokens = m_db->getTokens();
    std::vector<TokenId> facts;
    for (TokenSet::const_iterator it = tokens.begin(); it != tokens.end(); ++it){
      TokenId token = *it;
      checkError(token.isValid(), token);

      checkError(!token->isActive(), "Initially there should be no active tokens. " << tokenToString(token));

      // If a fact, make sure it is on an internal timeline
      if(token->isFact())
	facts.push_back(token);
      else if(!isAction(token)){ // Must be a rejectable goal, and an orphan
	checkError(token->master().isNoId(), tokenToString(token) << " is not an orphan. How can this be. Its master is " << token->master()->toString());

	// Should be inactive and rejectable
	checkError(token->isInactive() && token->getState()->baseDomain().isMember(Token::REJECTED), 
		   "Must be rejectable if a goal. It aint. " << tokenToString(token));
      }
    }

    // Apply facts to the database
    applyFacts(facts);

    // Now populate configuration tables
    configure();

    propagate();
  }

   DbCore::~DbCore(){
     DebugMessage::setStream(getStream());

     TREX_INFO("DbCore:~DbCore", "Cleaning up " << getName().toString());

     checkError(m_solver.isValid(), m_solver);
     m_solver.release();

     // Purge goals, ensuring messages are sent for all goals abut their final status if appropriate

     for(TokenSet::iterator it = m_goals.begin(); it != m_goals.end(); ++it){
       TokenId goal = *it;
       TREX_INFO("DbCore:~DbCore", "Evaluating buffered goal " << goal->toString());

       if(goal->isRejected()){
	 TREX_INFO("DbCore:~DbCore", goal->toString() << " was rejected.");
	 Agent::instance()->notifyRejected(goal);
       }
       else if(goal->isCommitted() || (goal->isMerged() && goal->getActiveToken()->isCommitted())){
	 TREX_INFO("DbCore:~DbCore", goal->toString() << " was completed.");
	 Agent::instance()->notifyCompleted(goal);
       }
       else {
	 TREX_INFO("DbCore:~DbCore", goal->toString() << " was not resolved.");
       }
     }

     instancesByDb().clear();

     s_foreignKeyRelation.clear();
  }

  void DbCore::notify(const Observation& observation){
    TREX_INFO("trex:info:trace", nameString() << observation.toString());

    // Get the client to work with
    DbClientId client = m_db->getClient();

    // Allocate a token - it should be inactive but not rejectable - cannot deny the truth
    TokenId token = client->createToken(observation.getPredicate().c_str(), NULL, NOT_REJECTABLE);

    // Bind the object variable
    ObjectId timeline = client->getObject(observation.getObjectName().c_str());
    const ConstrainedVariableId& objectVar = token->getObject();
    objectVar->specify(timeline);
    objectVar->restrictBaseDomain(objectVar->lastDomain());

    // A notification of a value should have the semantics of stating a fact is true at a given time. This means the latest start is
    // the current tick, and the earliest end is the current tick. Notably, it does not constrain the earliest time it could
    // become true.
    token->start()->restrictBaseDomain(IntervalIntDomain(getCurrentTick(), getCurrentTick()));
    token->end()->restrictBaseDomain(IntervalIntDomain(getCurrentTick()+1, PLUS_INFINITY));

    // Restrict the base domains for each parameter specifed.
    // TO DO: Make sure this is type safe
    for(unsigned int i = 0; i < observation.countParameters(); i++){
      const std::pair<LabelStr, const AbstractDomain*>& nameValuePair = observation[i];
      const LabelStr& varName = nameValuePair.first;
      const AbstractDomain& varDom = *(nameValuePair.second);
      const ConstrainedVariableId& param = token->getVariable(varName);
      checkError(param.isValid(), tokenToString(token) << " has no variable named " << varName.toString() << ". " << token->toLongString());
      TREX_INFO("trex:info:trace", nameString() << "Restricting " << param->toString() << " to " << varDom.toString());
      restrict(param, varDom);
    }

    // Store the token in the observation list
    std::map< int, TimelineContainer  >::iterator it = m_externalTimelineTable.find(timeline->getKey());
    checkError(it != m_externalTimelineTable.end(), "Failed to find and entry for " << observation.getObjectName().toString());
    it->second.updateLastObserved(getCurrentTick());

    // Buffer the observation for identification purposes later
    bufferObservation(token);
  }

  /**
   * @brief Post the goal and all the constraints you can. The goal is from another database. The key here is to
   * replicate the requested token and its related entities (variables and consraints). We maintain the mapping for all
   * replicated entities in the foreignKeyTable.
   */
  bool DbCore::handleRequest(const TokenId& goal){
    TREX_INFO("DbCore:handleRequest", nameString() << "Request received for " << goal->getPredicateName().toString() << "(" << goal->getKey() << ")");

    // Get the client to work with
    DbClientId client = m_db->getClient();

    // Allocate a token - it should be inactive and rejectable. This respects the semantics that the owner reactor
    // is the decider of what goes or does not go. May get requests that conflict with each other, or with observations.
    TokenId localGoal = client->createToken(goal->getPredicateName().c_str(), NULL, REJECTABLE);

    // A request must have its object variable bound. 
    checkError(goal->getObject()->lastDomain().isSingleton(), goal->getObject()->lastDomain().toString());

    // Object variables require special treatment since their Id's map to objects of the foreign database. Thus we
    // have to convert to the label and from there extract the corresponding object domain
    restrict(localGoal->getObject(), goal->getObject()->lastDomain());

    // The request is not valid if it is required to start in the past. The latest start time must be >= the current time.
    checkError(goal->start()->lastDomain().getUpperBound() >= getCurrentTick(),
	       goal->start()->lastDomain().toString() << " is dispatched too late for TICK " << getCurrentTick());

    // The request is not valid if it cannot start until after the next planning cycle!
    checkError(goal->start()->lastDomain().getLowerBound() <= (getCurrentTick() + getLookAhead()),
	       goal->start()->lastDomain().toString() << " is dispatched too early for TICK " << getCurrentTick() << 
	       ", lookAhead " << getLookAhead());

    localGoal->start()->restrictBaseDomain(goal->start()->lastDomain());
    localGoal->end()->restrictBaseDomain(goal->end()->lastDomain());
    localGoal->duration()->restrictBaseDomain(goal->duration()->lastDomain());
    setDispatchTime(localGoal);
 
    addEntity(goal, localGoal);
    addEntity(goal->getObject(), localGoal->getObject());
    addEntity(goal->duration(), localGoal->duration());
    addEntity(goal->start(), localGoal->start());
    addEntity(goal->end(), localGoal->end());

    // Iterate over parameters
    const std::vector<ConstrainedVariableId>& foreignParams = goal->parameters();
    const std::vector<ConstrainedVariableId>& localParams = localGoal->parameters();
    unsigned int paramCount = localParams.size();

    for(unsigned int i = 0; i < paramCount; i++) {
      const ConstrainedVariableId& foreignParam = foreignParams[i];
      const ConstrainedVariableId& localParam = localParams[i];
      restrict(localParam, foreignParam->lastDomain());
      addEntity(foreignParam, localParam);
    }

    // Finally, we migrate constraints. This leverages the foreign key mapping constructed above.
    applyConstraints(goal);

    // If the goal request is inconsistent, reject and delete it out of hand. This case is derived from test acse dispatch.2. The problem is that a
    // goal comes with the baggage of constraints that immediately apply and may prevent synchronization. No insertions can be tolerated which
    // intorduce inherent inconsistency
    if(!propagate()){
      TREX_INFO("trex:warning:handleRequest", "Goal " << localGoal->toLongString() << " causes inconsistency immediately and will be removed.");
      terminate(localGoal);
      // Clean terminated tokens
      Entity::discardAll(m_terminatedTokens);
      purgeOrphanedKeys();
      Entity::garbageCollect();
      return false;
    }

    // Finally, switch to indicate the planner should be active
    m_state = DbCore::ACTIVE;

    TREX_INFO("DbCore:handleRequest", nameString() << "Local Goal " << localGoal->toString() << " for request " << goal->toString());
    return true;
  }

  /**
   * Handling a recall requires goals corresponding to the foreign key to be removed and the foreign key mapping to be
   * removed also. Goals are buffered for removal in synchronization for the next tick.
   * @see recover
   */
  void DbCore::handleRecall(const TokenId& tok){
    TREX_INFO("DbCore:handleRecall", nameString() << "Recall received: " << tok->toString());

    markInvalid("Recall received. Will reset internal timelines.");

    // If we do not have the token locally, it has already been removed
    if(!hasEntity(tok))
      return;

    // Get the corresponding goal token and buffer it for removal on the next go.
    TokenId localGoal = getLocalEntity(tok);
    checkError(localGoal.isValid(), localGoal);
    m_recallBuffer.push_back(localGoal->getKey());

    // Remove foreign keys for goal and its variables
    removeEntity(tok);
    const std::vector<ConstrainedVariableId>& foreignParams = tok->parameters();
    for(unsigned int i = 0; i < foreignParams.size(); i++) {
      const ConstrainedVariableId& foreignParam = foreignParams[i];
      removeEntity(foreignParam);
    }

    // Remove foreign keys for constraints
    const std::vector<ConstrainedVariableId>& variables = tok->getVariables();
    for (unsigned int i = 0; i < variables.size(); i++){
      std::set<ConstraintId> constraints;
      const ConstrainedVariableId& var = variables[i];
      var->constraints(constraints);
      for(std::set<ConstraintId>::const_iterator c_it = constraints.begin(); c_it != constraints.end(); ++c_it){
	const ConstraintId constraint = *c_it;
	// Just remove it. No harm if removed more than once, or if never present at all.
	removeEntity(constraint);
      }
    }
  }

  /**
   * @brief Applies all the constraints it can on the variables of the foreign token
   * Finally, we must process the constraints. The basic plan is:
   * Get all the constraints. For each:
   * 1. If already copied ignore it
   * 2. If its variables are not present in the database already ignore it
   * 3. Otherwise make a local copy using the constraint cloning
   */
  void DbCore::applyConstraints(const TokenId& goal){
    const std::vector<ConstrainedVariableId>& variables = goal->getVariables();
    const unsigned int max_i = variables.size();
    for (unsigned int i = 0; i < max_i; i++){
      ConstrainedVariableId var = variables[i];
      std::set<ConstraintId> constraints;
      var->constraints(constraints);
      for(std::set<ConstraintId>::const_iterator c_it = constraints.begin(); c_it != constraints.end(); ++c_it){
	const ConstraintId constraint = *c_it;
	TREX_INFO("DbCore:applyConstraints", nameString() << "Evaluating " << constraint->toString() << " for " << goal->toString());

	// If built in to this token ignore it
	if(goal->isStandardConstraint(constraint) || 
	   (RuleVariableListenerId::convertable(constraint) && RuleVariableListenerId(constraint)->getRuleInstance()->getToken() == goal))
	  continue;

	// if already copied, ignore it
	if(hasEntity(constraint))
	   continue;

	// Set up a local scope collection to build up the scope as we go
	std::vector<ConstrainedVariableId> localScope;

	// if it refers to variables that are not present we ignore it. As another goal gets added we may revisit
	const std::vector<ConstrainedVariableId>& scope = constraint->getScope();
	unsigned int j_max = scope.size();
	for(unsigned int j = 0; j < j_max; j++){
	  const ConstrainedVariableId& foreignVar = scope[j];

	  // If the constraint has an entity not currently in the database, skip the constraint. It can be migrated
	  // at some later time perhaps when further tokens are posted.
	  if(!hasEntity(foreignVar)){
	    TREX_INFO("DbCore:applyConstraints", nameString() << "Excluding since " << foreignVar->toString() << " is not in the database.");
	    break;
	  }

	  ConstrainedVariableId localVar = getLocalEntity(foreignVar);

	  checkError(localVar->parent().isId() && TokenId::convertable(localVar->parent()),
		     localVar->toString() << " should be part of a token but it is not.");

	  // If the local variable is not part of this database then we ignore it. We should only link to variables for tokens dispatched to
	  // this reactor.
	  if(localVar->getConstraintEngine() != m_db->getConstraintEngine()){
	    TREX_INFO("DbCore:applyConstraints", nameString() << "Excluding since " << foreignVar->toString() << " is not in the database.");
	    break;
	  }

	  // If the local variable is part of a token that is rejected, then the goal should be rejected.
	  TokenId localToken = (TokenId) localVar->parent();
	  if(localToken->isRejected()){
	    TokenId newGoal = (TokenId) getLocalEntity(goal);
	    newGoal->reject();
	    return;
	  }

	  localScope.push_back(localVar);
	}

	// Now if the scope size match we can migrate the constraint
	if(localScope.size() == scope.size()){
	  TREX_INFO("DbCore:applyConstraints", nameString() << "Copying from " << goal->toString() << " constraint " << constraint->toString());

	  ConstraintId localConstraint = m_assembly.getConstraintEngine()->createConstraint(constraint->getName(), localScope);
	  localConstraint->setSource(constraint);
	  addEntity(constraint, localConstraint);
	}
      }
    }
  }

  void DbCore::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
    checkError(m_observer.isNoId(), "Must not be initialized yet.");
    m_observer = observer;

    checkError(initialTick >= getCurrentTick(), "Assume for now that this is the case. It impacts the test for dispatching initial conditions.");

    // Iterate over the external timelines. For each entry, update the server id from the map
    for(std::map<int, TimelineContainer>::iterator it = m_externalTimelineTable.begin();it != m_externalTimelineTable.end();++it){
      TimelineContainer& tc = it->second;
      TimelineId timeline = tc.getTimeline();
      std::map<double, ServerId>::const_iterator c_it = serversByTimeline.find(timeline->getName());
      condDebugMsg(c_it == serversByTimeline.end(), "trex:error", "Could not find a server for " << timeline->toString() << ". No reactor owns this timeline.");
      checkError(c_it != serversByTimeline.end(), "Could not find a server for " << timeline->toString() << ". No reactor owns this timeline.");
      tc.setServer(c_it->second);
    }

    m_sync_stepCount = 0;
    m_search_depth = 0;
    m_search_stepCount = 0;
    

    TickLogger *log = LogManager::instance().getTickLog(CPU_STAT_LOG);
    log->addField(getName().toString()+".sync.nSteps", m_sync_stepCount);
    log->addField(getName().toString()+".search.maxDepth", m_search_depth);
    log->addField(getName().toString()+".search.nSteps", m_search_stepCount);
  }

  /**
   * @brief Used only by commonInit
   */
  void DbCore::configure(){

    checkError( m_externalTimelineTable.empty() && m_internalTimelineTable.empty(), "Not empty, must have populated more than once.");

    std::list<ObjectId> objects;
    m_db->getObjectsByType(Agent::TIMELINE(), objects);

    for(std::list<ObjectId>::const_iterator it = objects.begin(); it != objects.end(); ++it){
      ObjectId object = *it;
      const LabelStr& object_name = object->getName(); 

      LabelStr composedName = compose(object_name, Agent::TIMELINE_MODE());
      // MUCH FASTER, BUT MORE BRITTLE: ConstrainedVariableId mode = object->getVariables()[0];
      ConstrainedVariableId mode =  object->getVariable(composedName);
      checkError(mode.isValid(), "No variable for '" << composedName.toString() << "' on " << object->toString());
      checkError(mode->lastDomain().isSingleton(), mode->toString());
      if(mode->getSpecifiedValue() == Agent::EXTERNAL_TIMELINE()){
	m_externalLabels.push_back(object_name);
	// Add an entry for external timelines. Link server later
	m_externalTimelineTable.insert(std::pair<int, TimelineContainer >(object->getKey(), TimelineContainer(object)));
	m_timelines.push_back(object);
      }
      else if(mode->getSpecifiedValue() == Agent::INTERNAL_TIMELINE()){
	m_internalLabels.push_back(object_name);
	m_internalTimelineTable.push_back( std::pair<TimelineId, TICK>(object, (TICK) PLUS_INFINITY));
	m_timelines.push_back(object);
      }
    }

    TREX_INFO("trex:info", m_internalTimelineTable.size() << " internal timelines. " << m_externalTimelineTable.size() << " external timelines.");
  }

  /**
   * @brief Iterates over all AgentTimeline instances and sorts according to the mode into the result categories.
   */
  void DbCore::queryTimelineModes(std::list<LabelStr>& externals,std::list<LabelStr>& internals) {
    externals.assign(m_externalLabels.begin(), m_externalLabels.end());
    internals.assign(m_internalLabels.begin(), m_internalLabels.end());
  }

  void DbCore::fillTimelineDescription(const TimelineId tl, PlanDescription::TimelineDescription &tlDesc) const {
    PlanDescription::TokenDescription tokDesc;

    // Set the name key
    tlDesc.name = tl->getName();

    // Iterate over tokens
    std::list<TokenId> const &tokens = tl->getTokenSequence();
    std::list<TokenId>::const_iterator tokit = tokens.begin();
    std::list<TokenId>::const_iterator const endtok = tokens.end();

    for( ;endtok!=tokit; ++tokit ) {
      TokenId const &tok = *tokit;
      // Only report active tokens
      if(!tok->isActive()) { continue; }

      TempVarId tokStart = tok->start(),
		tokEnd = tok->end();

      tokDesc.start[0] = tokStart->getLowerBound();
      tokDesc.start[1] = tokStart->getUpperBound();

      tokDesc.end[0] = tokEnd->getLowerBound();
      tokDesc.end[1] = tokEnd->getUpperBound();

      tokDesc.name = tok->getPredicateName();

      tokDesc.key = tok->getKey();

      // Add token to timeline description
      tlDesc.tokens.push_back(tokDesc);
    }
  }

  void DbCore::getPlanDescription(PlanDescription &planDesc) const{

    // Clear the plan description
    planDesc.clear();

    // Set tick
    planDesc.m_tick = getCurrentTick();

    // Set reactor name
    planDesc.m_reactorName = getName();

    // Internals 
    std::vector< std::pair<TimelineId, TICK> >::const_iterator intit = m_internalTimelineTable.begin();
    std::vector< std::pair<TimelineId, TICK> >::const_iterator const endint = m_internalTimelineTable.end();
    
    // Populate timeline description with tokens
    for( ; endint!=intit; ++intit ) {
      PlanDescription::TimelineDescription tlDesc;
      fillTimelineDescription(intit->first, tlDesc);
      planDesc.m_internalTimelines.push_back(tlDesc);
    }

    // Externals
    std::map<int, TimelineContainer>::const_iterator 
      cextit = m_externalTimelineTable.begin();
    std::map<int, TimelineContainer>::const_iterator const
      endcext = m_externalTimelineTable.end();

    for( ; endcext!=cextit; ++cextit ) {
      PlanDescription::TimelineDescription tlDesc;
      fillTimelineDescription(cextit->second.getTimeline(), tlDesc);
      planDesc.m_externalTimelines.push_back(tlDesc);
    }
  }

  void DbCore::writeTimeline(const TimelineId tl, const char mode, std::ofstream &db_out) const {
    // Write out the timeline description
    db_out
      << tl->getKey()			    // Key
      << "\t" << tl->getName().toString()   // Name
      << "\t" << mode			    // Mode designator [I,E,A]
      << std::endl;

    // Iterate over tokens
    std::list<TokenId> const &tokens = tl->getTokenSequence();
    std::list<TokenId>::const_iterator tokit = tokens.begin();
    std::list<TokenId>::const_iterator const endtok = tokens.end();

    for( ;endtok!=tokit; ++tokit ) {
      TokenId const &tok = *tokit;
      // Only report active tokens
      if(!tok->isActive()) { continue; }

      TempVarId tokStart = tok->start(),
		tokEnd = tok->end();

      db_out 
	<< "\t" << tok->getKey()		      // Token key
	<< "\t" << tok->getPredicateName().toString() // Token name
	<< "\t" << tokStart->getLowerBound()	      // Start lower
	<< "\t" << tokStart->getUpperBound()	      // Start upper
	<< "\t" << tokEnd->getLowerBound()	      // End lower
	<< "\t" << tokEnd->getUpperBound()	      // End upper
	<< std::endl;
    }
  }

  std::string DbCore::dumpState(bool verbose) {
    // Create a new file
    std::ostringstream oss;
    oss << m_statePath << "/" << getCurrentTick() << "." << Agent::instance()->getCurrentAttempt() << ".reactorstate";

    // Open file for writing
    std::ofstream db_out(oss.str().c_str());

    // Internals 
    std::vector< std::pair<TimelineId, TICK> >::const_iterator intit = m_internalTimelineTable.begin();
    std::vector< std::pair<TimelineId, TICK> >::const_iterator const endint = m_internalTimelineTable.end();
    
    // Write out timelne name and contents
    for( ; endint!=intit; ++intit ) {
      const TimelineId tl = intit->first; 
      writeTimeline(tl,'I', db_out);
    }

    // Externals
    std::map<int, TimelineContainer>::const_iterator 
      cextit = m_externalTimelineTable.begin();
    std::map<int, TimelineContainer>::const_iterator const
      endcext = m_externalTimelineTable.end();

    for( ; endcext!=cextit; ++cextit ) {
      const TimelineId tl = cextit->second.getTimeline();
      writeTimeline(tl,'E', db_out);
    }

    // Close the file handle
    db_out.close();
    
    // Output assembly as well (very large amount of data, will slow execution)
    condDebugMsg(!verbose, "trex:monitor:verbose", nameString() << m_assembly.exportToPlanWorks(getCurrentTick(), Agent::instance()->getCurrentAttempt()));
    condDebugMsg(verbose, "trex:monitor:conflicts:verbose", nameString() << m_assembly.exportToPlanWorks(getCurrentTick(), Agent::instance()->getCurrentAttempt()));
    
    return std::string("Success.");
  }

  std::string DbCore::writeConflict(std::string brief_description, std::string analysis) {

    // Write out associated DbState and Assembly for all reactors
    Agent::instance()->dumpState(true);

    // Create conflict file path
    std::ostringstream oss;
    oss << m_conflictPath << "/" << getCurrentTick() << "." << Agent::instance()->getCurrentAttempt() << ".conflict";
    
    // Open file for writing
    std::ofstream conflict_out(oss.str().c_str());

    // Write out conflict data
    conflict_out<<brief_description<<std::endl;
    conflict_out<<analysis;
    
    // Close file
    conflict_out.close();
    
    // Increment the  number of attempts
    Agent::instance()->incrementAttempts();

    return std::string("Success.");
  }

  void DbCore::addDbListener(EUROPA::PlanDatabaseListener& listener) {
    listener.setPlanDatabase(m_db);
  }

  void DbCore::handleTickStart(){
    m_sync_stepCount = 0;
    m_search_depth = 0;
    m_search_stepCount = 0;
    
    if(m_state == DbCore::INVALID){
      TREX_INFO("DbCore:handleTickStart", nameString() << "Database is invalid. Repair required.");
      return;
    }

    // Update the clock variable
    getAgentClockVariable(m_db)->restrictBaseDomain(IntervalIntDomain(getCurrentTick(), PLUS_INFINITY));
    
    // Since things depend on the clock there could easily be a constraint violated by a tick increment. We generate an excpetion
    // if the database is inconsistent and the deliberative reactor will handle repair.
    if(!propagate())
      return;

    checkError(verifyEntities(), "Bad entity detected.");

    // If the state is inactive, then we increment the current tick cycle.
    // We do this because in order to evaluate flaws we need to set the horizon
    if(m_state == DbCore::INACTIVE)
      m_currentTickCycle = getCurrentTick();

    // Start actiosn as needed
    updateActions();
    if(!propagate())
      return;
    
    // Send goals planned on server timelines
    dispatchCommands();

    TREX_INFO("trex:info", nameString() << "Database State Below" <<  std::endl << PlanDatabaseWriter::toString(m_db));
  }

  /**
   * @brief Tests if the solver can be activated.
   */
  bool DbCore::hasWork(){
    TREX_INFO("DbCore:hasWork", nameString() << "Checking for deliberation activity.");
    if(m_state == DbCore::INVALID)
      return false;

    // If it is active then we assume it has work to do. Quick check
    if(m_state == DbCore::ACTIVE){
      TREX_INFO("DbCore:hasWork", nameString() << "Already active, so yes.");
      return true;
    }

    // A Quick check will see if the planner can be executed yet
    if(m_lastCompleteTick == (int) getCurrentTick()){
      TREX_INFO("DbCore:hasWork", nameString() << "COMPLETE");
      return false;
    }

    if(Agent::instance()->getFinalTick() < (m_currentTickCycle + 1)){
      TREX_INFO("DbCore:hasWork", nameString() << "Extends beyond final tick.");
      return false;
    }

    return true;
  }

  void DbCore::resume(){
    TREX_INFO("DbCore:resume",  nameString() );

    // May have been pre-empted by a recall for example.
    if(m_state == DbCore::INVALID){
      TREXLog() << nameString() << "Database invalid for resuming deliberation." << std::endl;
      return;
    }

    checkError(isValidDb(), "Invalid database before deliberation step.");

    if(m_solver->isExhausted()) {
      m_planLog<<'['<<getCurrentTick()<<"] No plan found"<<std::endl;
      TREXLog() << nameString() << "No plan found." << std::endl;
      markInvalid("Solver exhausted. No Plan found. Problem is over constrained. Enable Solver:step in Debug.cfg to investigate", true);
      return;
    }

    if(!propagate())
      return;

    TREX_INFO("DbCore:resume:verbose",  nameString() << std::endl << PlanDatabaseWriter::toString(m_db) );

    // We will assume we have work to do. Trust but verify
    checkError(hasWork(), "Has no work to do. Error in locking or other logic.");

    // If the current state is INACTIVE, we need to trigger the active state
    if(m_state == DbCore::INACTIVE) {
      m_state = DbCore::ACTIVE;
      m_currentTickCycle = getCurrentTick();
      TREX_INFO("DbCore:resume",nameString() << "Starting cycle for tick " << m_currentTickCycle );
    }

    checkError(m_state == DbCore::ACTIVE, "Should always be in this state by now if clean up done correctly.");


    // Retrieve and store a copy of the old horizon to reset afterwards. It is shared state for the process.
    setHorizon();
    const IntervalIntDomain& horizon = DeliberationFilter::getHorizon();

    TREX_INFO("DbCore:resume",  nameString() << "Using horizon " << horizon.toString());

    checkError(true || !horizon.isMember(getCurrentTick()), 
	       "Cannot be planning at the execution frontier. Collides with synchronization.");

    // Execute a single step in the search
    m_solver->step();

    m_search_depth = std::max(m_search_depth, m_solver->getDepth());
    m_search_depth += m_solver->getStepCount();
    TREX_INFO("trex:info:planning", "Step: depth == " << m_solver->getDepth() << " count == " << m_solver->getStepCount());

    // Now handle the aftermath. Dispatch is handled on a clock tick, so just update tick cycles
    // and clear the solver.
    processPendingTokens();
    if(m_solver->noMoreFlaws()){

      condDebugMsg(m_solver->getStepCount() > 0, "trex:info:planning", logPlan("New Plan"));

      if(deactivateSolver()){
	// Restore to inactive state
	m_state = DbCore::INACTIVE;
	TREX_INFO("trex:info:planning", nameString() << "COMPLETED ");
      }
    }

    return;
  }

  /**
   * @brief Low-level integrity check.
   */
  bool DbCore::isValidDb() const{
    if(m_state == DbCore::INVALID)
      return false;

    std::set<EntityId> entities;
    Entity::getEntities(entities);
    for(std::set<EntityId>::const_iterator it = entities.begin(); it != entities.end(); ++it){
      EntityId entity = *it;
      if(!entity.isValid())
	return false;
    }

    return true;
  }

  bool DbCore::isGoal(const TokenId& tok) const{
    bool result = tok->master().isNoId();
    return result && m_goals.find(tok) != m_goals.end();
  }

  bool DbCore::isObservation(const TokenId& tok) const {return m_observations.find(tok) != m_observations.end();}
  
  bool DbCore::isAction(const TokenId& tok) {
    return false;
    return tok->getPlanDatabase()->getSchema()->isA(tok->getBaseObjectType(), Agent::ACTION());
  }

  /**
   * In terminating a token, we can declare its final outcome if it were a goal
   */
  void DbCore::terminate(const TokenId& token){
    static unsigned int sl_counter(0);
    sl_counter++;

    TREX_INFO("trex:info", nameString() << "Terminating " << tokenToString(token));

    // If it is a goal, generate messages indicating its eventual fate
    if(isGoal(token)){
      if(token->isInactive() || token->isRejected())
	Agent::instance()->notifyRejected(token);
      else
	Agent::instance()->notifyCompleted(token);
    }

    // Terminate token. Will discard in batch later
    token->terminate();

    condDebugMsg(m_db->getConstraintEngine()->isRelaxed(), "trex:error", nameString() << "Discovered relaxation in archiving when terminating " << token->toLongString());
  }

  /**
   * @brief Notifies observers of new information.
   */
  void DbCore::notifyObservers(){
    checkError(m_state != DbCore::INVALID, "Should not be publishing when state is invalid");

    TREX_INFO("trex:debug:synchronization:notifyObservers", nameString() <<  "START");

    for(std::vector< std::pair<TimelineId, TICK> >::iterator it = m_internalTimelineTable.begin(); it != m_internalTimelineTable.end(); ++it){
      TimelineId timeline = it->first;
      TICK lastPublished = it->second;

      // Can skip if already published for the current tick
      checkError((lastPublished == (TICK) PLUS_INFINITY) || (lastPublished <= getCurrentTick()), lastPublished << " > " << getCurrentTick());
      if(lastPublished == getCurrentTick())
	continue;

      // We want the token at the current tick. If there is NECESSARILY a change in value then we consider it suitable for
      // dispatch. This means the start time == the currentTick. This is mainly a consideration where we are handling the first tick
      const std::list<TokenId>& tokenSequence = timeline->getTokenSequence();
      for(std::list<TokenId>::const_iterator t_it = tokenSequence.begin(); t_it != tokenSequence.end(); ++t_it){
	TokenId token = *t_it;
	checkError(token.isValid(), token);
	TREX_INFO("trex:debug:synchronization:notifyObservers", 
		  nameString() << "Evaluating " << tokenToString(token) << " for update.");

	// If we are processing the initial tick, set the start time to be exactly 0
	if(getCurrentTick() == 0 && token->start()->lastDomain().getUpperBound() == 0){
	  token->start()->restrictBaseDomain(IntervalIntDomain(0, 0));
	  if(!propagate()){
	    TREX_INFO("trex:error", nameString() << "Inconsistent on propagating initial time bounds." << std::endl  << m_synchronizer.propagationFailure());

	    throw std::runtime_error("Fatal Error propagating initial time bounds in " + nameString() + "\n\n" + m_synchronizer.propagationFailure());
	  }
	}

	const IntervalIntDomain& startTime = token->start()->lastDomain();
	const IntervalIntDomain& endTime = token->end()->lastDomain();

	// If the earliest start time is after the current tick then we can finish
	if(startTime.getLowerBound() > getCurrentTick())
	  break;

	// If the token has not been committed, do so since it is in the past
	if(!token->isCommitted())
	  commitAndRestrict(token);

	if(!propagate()){
	  TREX_INFO("trex:error", nameString() << "Inconsistent on propagating committed values during synchronization." << std::endl  << m_synchronizer.propagationFailure());
	  // CONFLICT
	  throw  std::runtime_error("Fatal Error propagating committed values during synchronization in " + nameString() + "\n\n" + m_synchronizer.propagationFailure());
	}

	// If the latest end time <= the current tick, skip ahead
	if(endTime.getUpperBound() <= getCurrentTick()){
	  token->restrictBaseDomains();
	  continue;
	}

	// If the start time is not a singleton it does not have to be published, so we do not.
	if(startTime.getUpperBound() > getCurrentTick() && !startTime.isSingleton())
	  continue;

	// If we have a hit, generate a notification.
	if(startTime.isMember(getCurrentTick()) && m_notificationKeys.find(token->getKey()) == m_notificationKeys.end()){
	  IntervalIntDomain startBounds((int) token->start()->lastDomain().getUpperBound(), getCurrentTick());
	  token->start()->restrictBaseDomain(startBounds);

	  if(!propagate()){
	    TREX_INFO("trex:error", nameString() << "Inconsistent on propagating committed values during synchronization." << std::endl  << m_synchronizer.propagationFailure());
	    // CONFLICT
	    throw std::runtime_error("Fatal Error propagating committed values during synchronization in " + nameString() + "\n\n" + m_synchronizer.propagationFailure());
	  }

	  // Now dispatch observation
	  ObservationByReference obs(token);
	  m_observer->notify(obs);

	  // Log last published for this tick
	  it->second = getCurrentTick();

	  m_notificationKeys.insert(token->getKey());
	}
      }
    }

    TREX_INFO("trex:debug:synchronization:notifyObservers", nameString() <<  "END");
  }

  /**
   * @brief Dispatch Goals To Respective Servers.
   *
   * The window of time for dispatching is based on server lookahead. 
   * The intuition is that we dispatch commands to the server if they could be started in the dispatch window
   * and if they could execute for at least a 1 tick duration in that window.
   */
  void DbCore::dispatchCommands(){
    TREX_INFO("trex:debug:dispatching:dispatchCommands", nameString() << "START");

    std::vector<TokenId> activeUncontrollableEvents;
    bool initialized(false);
    for(std::map<int, TimelineContainer>::iterator it = m_externalTimelineTable.begin(); it != m_externalTimelineTable.end(); ++it){
      TimelineContainer& tc = it->second;
      TimelineId timeline = tc.getTimeline();
      ServerId server = tc.getServer();

      // The lower bound for dispatching is based on the server latency.
      TICK dispatchLB = getCurrentTick();

      // Upper bound includes lookahead.
      TICK dispatchUB = std::min(dispatchLB + server->getLookAhead(),  Agent::instance()->getFinalTick());

      TREX_INFO("trex:debug:dispatching", 
	       nameString() << "Evaluating tokens on " << timeline->getName().toString() <<
	       " for dispatch in [" << dispatchLB << ", " << dispatchUB << "]");

      if(dispatchUB < dispatchLB)
	continue;

      IntervalIntDomain dispatchWindow(dispatchLB, dispatchUB);
      

      // Get the timeline to dispatch as a list and then process it
      const std::list<TokenId>& tokenSequence = timeline->getTokenSequence();
      for(std::list<TokenId>::const_iterator t_it = tokenSequence.begin(); t_it != tokenSequence.end(); ++t_it){
	TokenId token = *t_it;

	// If we have already dispatched the token we can skip it. We would be better served storing this execution frontier
	// to more efficiently dispatch. This could be done in a derivative of the Timeline designed for execution systems
	if(token->isCommitted() || tc.isDispatched(token) || inDeliberation(token))
	  continue;

	if (!initialized) {
	  initialized = true;
	  getActiveUncontrollableEvents(activeUncontrollableEvents);
	}

	checkError(token.isValid(), token);

	TREX_INFO("trex:debug:dispatching", 
		 nameString() << "Evaluating " << tokenToString(token) << " for dispatch window [" << dispatchLB << ", " << dispatchUB << "]");
	
	const IntervalIntDomain& startTime = token->start()->lastDomain();

	// If the start time of the token is necessarily beyond the dispatch window, we are done with this timeline
	if(startTime.getLowerBound() >= dispatchUB)
	  break; 

	if (hasPendingPredecessors(token, activeUncontrollableEvents, true)) 
	  break;

	TICK latestStart = (TICK) startTime.getUpperBound();

	// If the start time of the token is necessarily before the server could execute it, we will not dispatch it. Synchronization
	// will determine if it is in fact going to be true or not.
	if(latestStart < getCurrentTick())
	  continue;

	// If the token has an  overlap with the dispatch window of at least on tick duration, send it. If the request
	// is accepted we mark it as dispatched, otherwise we break from this timeline, since we do not expect a server to receice a request
	// for a latter goal if it does not accept a predecessor. Note that the semantics of accepting a request are not the same as rejecting
	// the request outright. It is rather the question of whether you can serve the request now. Absent a positive reponse, we wil retry
	// on the next iteration
	if(startTime.intersects(dispatchWindow)){
	  TREX_INFO("trex:dispatching", nameString() << "Dispatching " << token->toLongString());
	  token->getObject()->restrictBaseDomain(token->getObject()->lastDomain());

	  if(!propagate()){
	    TREX_INFO("trex:warning:dispatchCommands", nameString() << "Dispatching " << token->toLongString() << " failed due to an inconsistent network.");
	    // CONFLICT
	    return;
	  }

	  if(server->request(token)){
	    tc.markDispatched(token);
	    setDispatchTime(token);
	  }
	  else
	    break;
	}
      }
    }

    TREX_INFO("trex:debug:dispatching:dispatchCommands", nameString() << "END");
  }


  /**
   * @brief Dispatch Recalls To Respective Servers. All tokens in the future that have been dispatched should be recalled.
   */
  void DbCore::dispatchRecalls(){

    if(m_lastRecalled == getCurrentTick())
      return;

    m_lastRecalled = getCurrentTick();

    TREX_INFO("trex:debug:dispatching:dispatchRecalls", nameString() << "START");

    for(std::map<int, TimelineContainer>::iterator it = m_externalTimelineTable.begin(); it != m_externalTimelineTable.end(); ++it){
      TimelineContainer& tc = it->second;
      TimelineId timeline = tc.getTimeline();
      ServerId server = tc.getServer();

      // If the end time is not passed, consider it.
      const std::list<TokenId>& tokenSequence = timeline->getTokenSequence();
      for(std::list<TokenId>::const_iterator t_it = tokenSequence.begin(); t_it != tokenSequence.end(); ++t_it){
	TokenId token = *t_it;
	checkError(token.isValid(), token);

	TREX_INFO("trex:debug:dispatching", nameString() << 
		 "Evaluating " << tokenToString(token) << " for recall. Dispatched[" << tc.isDispatched(token) << "] "
		 "Ends:" << token->end()->baseDomain().toString());

	// If the token has been dispatched and it is not finished yet ,recall it.
	if(tc.isDispatched(token) && token->end()->baseDomain().getUpperBound() > getCurrentTick() && !observedNow(token)){

	  TREX_INFO("trex:dispatching", nameString() << "Recalling " << tokenToString(token));
	  server->recall(token);
	  tc.clearDispatched(token);
	  resetDispatchTime(token);
	}
      }
    }

    TREX_INFO("trex:debug:dispatching:dispatchRecalls", nameString() << "END");
  }

  /**
   * @brief Applies to exogenous timelines only. Can force an inconsistency by detecting a timeout for an expected value.
   * Locate the token at the execution frontier and extend the earliest end time by a tick.
   */
  bool DbCore::extendCurrentValue(const TimelineId& timeline){
    if(m_state == DbCore::INVALID){
      TREX_INFO("trex:warning:synchronization", nameString() << "Invalid on call to extend current value");
      return false;
    }

    // Retrieve value from prior tick. Prevent underflow
    TICK tick = getCurrentTick();

    TokenId token = getValue(timeline, (tick == 0 ? 0 : tick - 1));

    // If there is no current value, or the one we have cannot be extended, we have missed an
    // expected operation
    if(token.isNoId() || token->end()->lastDomain().getUpperBound() ==  tick || (tick == 0 && !isObservation(token))){

      TREX_INFO_COND(token.isNoId(), "trex:warning:synchronization", nameString() <<
	  "Missed expected observation. No current value.");

      TREX_INFO_COND(token.isId(), "trex:warning:synchronization", nameString() << 
	       "Missed expected observation to terminate " << tokenToString(token) << " ending at " << token->end()->toString());

      TREXLog() << nameString() <<  "Missed expected observation on " <<
	timeline->toString() << std::endl;

      // Force invalid state to trigger a repair
      std::string missing_observation_str;
      TREX_INFO("trex:monitor:conflicts:nominal", nameString() << (missing_observation_str = missingObservation(timeline)));

      std::string token_extension_failure_str;
      TREX_INFO_COND(token.isId(), "trex:monitor:conflicts:nominal", nameString() << (token_extension_failure_str = m_synchronizer.tokenExtensionFailure(token)));

      markInvalid(std::string("Expected an observation on ") + timeline->toString() + ". Are observations being generated?. The plan may simply be broken.",
	  true, missing_observation_str + "\n" + token_extension_failure_str);

      return false;
    }

    TREX_INFO("DbCore:extendCurrentValue", nameString() << 
	     timeline->getName().toString() << " extends value " << tokenToString(token) << " for next tick " << getCurrentTick() + 1);

    return extendCurrentValue(token);
  }

  /**
   * @brief Handles the extension of current value for a single token.
   */
  bool DbCore::extendCurrentValue(const TokenId& token){
    TREX_INFO("DbCore:extendCurrentValue:token", nameString() << "Extending " << tokenToString(token) << " for end >= " << getCurrentTick());

    checkError(token->start()->lastDomain().getUpperBound() <= getCurrentTick(),
	       token->start()->lastDomain() << " and TICK " << getCurrentTick());

    checkError(token->end()->lastDomain().getUpperBound() > getCurrentTick(), 
	       token->end()->lastDomain() << " and TICK " << getCurrentTick());

    // If the token has not been committed, do so
    if(!token->isCommitted())
      commitAndRestrict(token);

    token->end()->restrictBaseDomain(IntervalIntDomain(getCurrentTick() + 1, (int) token->end()->baseDomain().getUpperBound()));

    return propagate();
  }

  bool DbCore::propagate(){
    if(m_state == DbCore::INVALID)
      return false;

    if(!m_db->getConstraintEngine()->propagate()){
      TREXLog() << nameString() << "Inconsistent plan." << std::endl;
      TREX_INFO("DbCore:propagate", nameString() << "Inconsistent plan.");
      markInvalid("The constraint network is inconsistent. To investigate, enable ConstraintEngine in Debug.cfg. Look for EMPTIED domain in log output to find the culprit.",true);
      TREX_INFO("trex:monitor:conflicts:nominal", nameString() << m_synchronizer.propagationFailure());
    }
    else {
      processPendingTokens();
    }

    return m_state != DbCore::INVALID;
  }

  /**
   * @brief Restrict base domains of a committed token. This method is called after synchronization
   * and just prior to advancing to the next tick. A token can be removed if its end time <= current tick since
   * that means that once the tick advances, it will be outside the scope of synchronization.
   * @return true if to be removed.
   * @see archive
   */
  bool DbCore::restrict(const TokenId& token){
    checkError(token->isCommitted(), "Must be committed.");

    bool result = false;

    // If the token is in the past, and will no longer be relevant in synchronization, then we can do
    // final restrictions and mark for removal
    if(token->end()->lastDomain().getUpperBound() <= getCurrentTick()){
      token->restrictBaseDomains();
      result = true;
    }
    else // Just nudge the end time
      token->end()->restrictBaseDomain(IntervalIntDomain(getCurrentTick(), PLUS_INFINITY));

    propagate();
    return result;
  }

  /**
   * This algorithm updates tokens related to the given token so the latter can be removed from
   * the database. Related tokens are:
   * 1. Master 
   * 2. Active Slaves
   * 3. Merged Slaves
   * 4. Supported Tokens
   * @return true if all related tokens are removed. Otherwise false.
   * @see DbCore::archive
   */
  bool DbCore::updateRelatedTokens(const TokenId& token){
    checkError(token->isCommitted(), "Should be committed before this call.");

    if(!propagate())
      return false;


    // If the tokens end time is >= the current tick, defer termination handling since still in the synchronization
    // window
    if(token->end()->baseDomain().getUpperBound() >= getCurrentTick())
      return false;

    // Update base domains on uncommitted slaves.
    const TokenSet slaves = token->slaves();
    for(TokenSet::const_iterator it = slaves.begin(); it != slaves.end(); ++it){
      static unsigned int sl_counter(0);

      TokenId slave = *it;

      TREX_INFO("DbCore:updateRelatedTokens", nameString() << "Evaluating slave " << slave->toString() << "[" << sl_counter++ << "]");

      // If the slave is merged onto an active token, we can restrict the bounds of the
      // active token accordingly to persist the impact of the merge
      if(slave->isMerged()){
	TokenId activeToken = slave->getActiveToken();

	// It is possible that the active token has already been terminated in which case we
	// need do nothing
	if(!activeToken->isTerminated()){
	  const std::vector<ConstrainedVariableId>& sourceVars = slave->getVariables();
	  const std::vector<ConstrainedVariableId>& targetVars = activeToken->getVariables();
	  unsigned int max_i = sourceVars.size();

	  // Restrict bounds on the active token. We skip the state variable at position 0 since they will
	  // differ between merged and active tokens.
	  for(unsigned int i = 1; i < max_i; i++){
	    targetVars[i]->restrictBaseDomain(sourceVars[i]->baseDomain());
	    if(!propagate())
	      return false;
	  }
	}

	terminate(slave);
	continue;
      }

      // If the slave is committed, unlink it. It will be handled directly in archiving
      if(slave->isCommitted()){
	slave->removeMaster(token);
	continue;
      }

      // If the slave is not in scope, terminate it
      if(!inScope(slave)){
	terminate(slave);
	continue;
      }

      // If in the future we should retain it, but disconnect it.
      if(slave->end()->lastDomain().getLowerBound() > getCurrentTick()){
	slave->restrictBaseDomains();
	slave->incRefCount();
	slave->removeMaster(token);
	continue;
      }

      terminate(slave);
    }

    // If there are any tokens merged onto this token which are orphaned, they may be terminated. If there are merged
    // tokens that have masters, then we cannot terminate the current token.
    const TokenSet mergedTokens = token->getMergedTokens();
    bool canTerminate = true;
    for(TokenSet::const_iterator it = mergedTokens.begin(); it != mergedTokens.end(); ++it){
      TokenId m = *it;

      if(m->master().isNoId())
	terminate(m);
      else
	canTerminate = false;
    }

    return canTerminate;
  }

  /**
   * @brief We deactivate the solver if it has failed to find a plan in time, or when it has terminated.
   */
  bool DbCore::deactivateSolver(){
    TREX_INFO("DbCore:deactivateSolver", nameString());
    m_solver->clear();

    // Now bring our tick up to speed
    m_currentTickCycle = getCurrentTick();
    m_lastCompleteTick = getCurrentTick();

    if(!timelinesAreComplete()){
      markInvalid("Incomplete after planning. Need to further constrain the model to require the planner to integrate the plan to the execution frontier.",true);
      return false;
    }

    return true;
  }

  bool DbCore::hasEntity(const EntityId& entity){
    return s_foreignKeyRelation.find(entity->getKey()) != s_foreignKeyRelation.end();
  }

  void DbCore::addEntity(const EntityId& foreign, const EntityId& local){
    s_foreignKeyRelation.insert(std::pair<int, EntityId>(foreign->getKey(), local));
  }

  void DbCore::removeEntity(const EntityId& foreign){
    s_foreignKeyRelation.erase(foreign->getKey());
  }

  EntityId DbCore::getLocalEntity(const EntityId& foreign){
    std::map<int, EntityId>::const_iterator it = s_foreignKeyRelation.find(foreign->getKey());
    if(it != s_foreignKeyRelation.end()){
      EntityId e = it->second;
      checkError(e.isValid(), e << " is a stale id for " << foreign->toString());
      return e;
    }
    else
      return EntityId::noId();
  }

  EntityId DbCore::getForeignEntity(const EntityId& local){
    for(std::map<int, EntityId>::iterator it = s_foreignKeyRelation.begin(); it != s_foreignKeyRelation.end(); ++it){
      if(it->second == local){
	EntityId foreignEntity = Entity::getEntity(it->first);
	return foreignEntity;
      }
    }

    return EntityId::noId();
  }

  void DbCore::purgeOrphanedKeys(){
    std::map<int, EntityId>::iterator it = s_foreignKeyRelation.begin();
    while(it != s_foreignKeyRelation.end()){
      int foreignKey = it->first;
      if(it->second->isDiscarded() || Entity::getEntity(foreignKey).isNoId())
	s_foreignKeyRelation.erase(it++);
      else
	++it;
    }
  }

  bool DbCore::onTimeline(const TokenId& token, const LabelStr& timelineMode){
    if(token->getObject()->lastDomain().isSingleton()){
      ObjectId object = token->getObject()->lastDomain().getSingletonValue();
      if(token->getPlanDatabase()->getSchema()->isA(object->getType(), Agent::TIMELINE())){
	ConstrainedVariableId mode = object->getVariables()[0];
	if(mode->lastDomain().isMember(timelineMode))
	  return true;
      }
    }

    return false;
  }

  /**
   * @brief process the goals to update information for goals not already planned. Will reject goals
   * @see handleTickStart
   */
  void DbCore::updateGoals(){
    if(m_state == DbCore::INVALID)
      return;

    std::vector<TokenId> garbage;

    for(TokenSet::iterator it = m_goals.begin(); it != m_goals.end(); ++it){
      TokenId goalToken = *it;
      checkError(goalToken.isValid(), goalToken);

      // Restrict the lower bound of the end time based on the current tick. Since this method is 
      // called at the start of a new tick, we can ensure that the goal must not end before the next tick
      TICK latestStart = (TICK) goalToken->start()->lastDomain().getUpperBound();

      // If the goal is inactive, we can update its bounds or reject it.
      if(goalToken->isInactive()){
	TREX_INFO("DbCore:updateGoals", nameString() << "Updating unplanned goal " << goalToken->toString());

	// If the goal is impossible based on time then reject it
	if (latestStart < getCurrentTick()) {
	  TREX_INFO("DbCore:updateGoals", nameString() << "Rejecting timed out goal " << goalToken->toString());
	  goalToken->reject();
	}
	else {
	  TREX_INFO("DbCore:updateGoals", nameString() << "Pushing end for " << goalToken->toString());
	  //goalToken->end()->restrictBaseDomain(IntervalIntDomain(getCurrentTick() + 1, PLUS_INFINITY));
	  goalToken->start()->restrictBaseDomain(IntervalIntDomain(getCurrentTick(), PLUS_INFINITY));
	}

	// Propagate effects
	if(!propagate())
	  return;
      }

      // If the goal is rejected, through this update or as a result of a planner decision, terminate it
      if(m_state == DbCore::INACTIVE && goalToken->isRejected())
	garbage.push_back(goalToken);

      // If the goal is active, we can restrict the start time base domain to be greater than or equal to the current tick. This is
      // important so facilitate archiving where tokens in the future have ordering relations to those in the past.
      if(goalToken->isActive() && !goalToken->isCommitted() && latestStart >= getCurrentTick())
	goalToken->start()->restrictBaseDomain(IntervalIntDomain(getCurrentTick(), PLUS_INFINITY));
    }

    for(std::vector<TokenId>::const_iterator it = garbage.begin(); it != garbage.end(); ++it)
      terminate(*it);
  }

  /**
   * @brief Commit the execution frontier
   * @see synchronize
   */
  void DbCore::commit(){
    if(m_state == DbCore::INVALID)
      return;

    // Observations
    for(TokenSet::iterator it = m_observations.begin(); it != m_observations.end(); ++it){
      TokenId observation = *it;
      checkError(observation.isValid(), observation);
      checkError(observation->isActive() || observation->isMerged(), 
		 observation->toString() << " has state " << observation->getState()->toString() <<
		 " This implies that the plan is incomplete (i.e. not stitched into the execution frontier)" <<
		 " since we were unable to unambiguously integrate the observation.");

      TokenId activeToken = (observation->isActive() ? observation : observation->getActiveToken());

      checkError(activeToken.isValid(), observation->toString());

      if(!activeToken->isCommitted())
	commitAndRestrict(activeToken);

      if(!propagate())
	return;
    }
  }

  /**
   * @brief
   */
  void DbCore::updateActions(){
    TREX_INFO("trex:debug:synchronization:updateActions", nameString());

    if(m_state != DbCore::INACTIVE){
      TREX_INFO("trex:debug:synchronization:updateActions", nameString() << "Skipping action update as planning is active.");
      return;
    }

    std::vector<TokenId> activeUncontrollableEvents;
    bool initialized(false);

    const IntervalIntDomain horizon(getCurrentTick(), getCurrentTick());

    // Iterate over all active actions. If one can be started, do it.
    const TokenSet& actions = m_db->getActiveTokens("AgentAction.Active");
    for(TokenSet::iterator it = actions.begin(); it != actions.end(); ++it){
      TokenId action = *it;
      checkError(action.isValid(), action);
      const IntervalIntDomain& startTime = action->start()->lastDomain();
      const IntervalIntDomain& endTime = action->end()->lastDomain();

      TREX_INFO("trex:debug:synchronization:updateActions", 
		"Evaluating " << action->toString() << " from " << startTime.toString() << " to " << endTime.toString());

      if(action->start()->isSpecified() || !horizon.intersects(startTime) || !isInternal(action))
	continue;

      if(!initialized){
	getActiveUncontrollableEvents(activeUncontrollableEvents);
	initialized = true;
      }

      // Now either nudge the action or start it
      if(!action->start()->lastDomain().isSingleton() && !hasPendingPredecessors(action, activeUncontrollableEvents)){
	TREXLog() << nameString() << "Starting " << action->toString() << std::endl;
	TREX_INFO("trex:debug:synchronization:updateActions", nameString() << "Starting " << action->toString());

	// Start the action in this tick and commit to it. In the event of an inconsistency we have to repair anyway
	// and the action will be discarded.
	action->start()->specify(getCurrentTick());

	// If now inconsistent, this will trigger a repair during synchronization
	if(!propagate())
	  return;
      }
    }
  }
  
  /**
   * @brief Clear out the crud of tokens in the past that that can not effect the present or the future
   */
  void DbCore::archive(){
    if(m_state != DbCore::INACTIVE)
      return;

    // Propagate the database
    if(!propagate())
      return;

    // Now we process any committed tokens that may be up for termination. These will be cleaned up on further ticks.
    TokenSet committedTokens = m_committedTokens;
    for(TokenSet::iterator it = committedTokens.begin(); it != committedTokens.end(); ++it){
      TokenId token = *it;
      checkError(token.isValid(), token);

      TREX_INFO("DbCore:archive", nameString() << "Evaluating " << tokenToString(token));

      if(restrict(token) && updateRelatedTokens(token)){
	TREX_INFO("DbCore:archive", nameString() << tokenToString(token) << " is a candidate for termination.");

	disconnectConstraints(token);

	m_committedTokens.erase(token);
	m_terminableTokens.insert(token);

	continue;
      }
    }

    for(TokenSet::iterator it = m_terminableTokens.begin(); it != m_terminableTokens.end(); ++it){
      TokenId token = *it;
      checkError(token.isValid(), token);
      if(canBeTerminated(token))
	terminate(token);
    }

    // Clean terminated tokens
    Entity::discardAll(m_terminatedTokens);
    purgeOrphanedKeys();
    Entity::garbageCollect();

    condDebugMsg(m_db->getConstraintEngine()->isRelaxed(), "trex:error", nameString() << "Should be no relaxation in garbage collection");
  }

  void DbCore::setHorizon(){
    TICK horizonStart, horizonEnd;
    getHorizon(horizonStart, horizonEnd);
    DeliberationFilter::update(horizonStart, horizonEnd);
  }

  void DbCore::getHorizon(TICK& horizonStart, TICK& horizonEnd) const {
    TICK finalTick = Agent::instance()->getFinalTick();
    horizonStart =  std::min(finalTick-1, getCurrentTick() + 1);
    horizonEnd = std::min(finalTick, horizonStart + getLookAhead());
  }

  /**
   * @brief Apply Inertial Value Assumption to external timelines. This will require extension of current values if not suppprted
   * by observation. Failure to extend such a value implies an expected observation has not been received.
   */
  bool DbCore::completeExternalTimelines(){
    // Go thru all external timelines and extend current values if they have not changed in this tick.
    for(std::map< int, TimelineContainer >::const_iterator it = m_externalTimelineTable.begin(); it != m_externalTimelineTable.end(); ++it){
      const TimelineContainer& tc = it->second;
      if(tc.lastObserved() < getCurrentTick())
	if(!extendCurrentValue(tc.getTimeline())){
	  TREX_INFO("DbCore:completeExternalTimelines", nameString() << "Failed to extend " << tc.getTimeline()->toString());
	  return false;
	}
    }

    return true;
  }

  void DbCore::restrict(const ConstrainedVariableId& var, const AbstractDomain& dom){
    // If the parameter is an object variable then we must do some conversion
    if(ObjectVarId::convertable(var)){
      const ObjectDomain& objDom = static_cast<const ObjectDomain&>(dom);
      ObjectDomain localDom(objDom.getDataType());
      std::list<ObjectId> sourceObjects = objDom.makeObjectList();
      for(std::list<ObjectId>::const_iterator it = sourceObjects.begin(); it != sourceObjects.end(); ++it){
	ObjectId foreignObj = *it;
	ObjectId localObj = m_db->getObject(foreignObj->getName());
	checkError(localObj.isValid(), "No corresponding object for '" << foreignObj->getName().toString() << "'");
	localDom.insert(localObj);
      }

      localDom.close();
      var->restrictBaseDomain(localDom);
    }
    else{
      var->restrictBaseDomain(dom);
    }
  }

  /**
   * @brief E2 implementatin should avoid a copy
   */
  void DbCore::getTimelines(ObjectSet& results){
    static const LabelStr sl_timeline("Timeline");
    std::set<ObjectId> timelines = m_db->getObjectsByType(sl_timeline);
    for(std::set<ObjectId>::const_iterator it = timelines.begin(); it != timelines.end(); ++it){
      results.insert(*it);
    }
  }

  void DbCore::migrateConstraints(const TokenId& source, const TokenId& target){
    checkError(source->isCommitted(), "Should only migrate if committed.");
    checkError(source->getPredicateName() == target->getPredicateName(), source->toString() << " incompatible with " << target->toString());

    const std::vector<ConstrainedVariableId>& variables = source->getVariables();
    const unsigned int max_i = variables.size();
    std::set<ConstraintId> constraints;
    for (unsigned int i = 0; i < max_i; i++){
      ConstrainedVariableId var = variables[i];
      var->constraints(constraints);
    }
    
    // Go through all constraints, and migrate each, adjusting variables as you go.
    for(std::set<ConstraintId>::const_iterator it = constraints.begin(); it != constraints.end(); ++it){
      const ConstraintId constraint = *it;

      // Ignore if a standard constraint
      if(source->isStandardConstraint(constraint))
	continue;

      std::vector<ConstrainedVariableId> newScope;
      const std::vector<ConstrainedVariableId>& oldScope = constraint->getScope();
      unsigned int j_max = oldScope.size();
      for(unsigned int j = 0; j < j_max; j++){
	const ConstrainedVariableId& var = oldScope[j];

	// If the variable is from the source, switch it
	if(var->parent() == source)
	  newScope.push_back(target->getVariables()[var->getIndex()]);
	else
	  newScope.push_back(var);
      }

      ConstraintId newConstraint =  m_assembly.getConstraintEngine()->createConstraint(constraint->getName(), newScope);
      newConstraint->setSource(constraint);
    }
  }

  bool DbCore::isInternal(const TokenId& token){
    return onTimeline(token, Agent::INTERNAL_TIMELINE());
  }

  bool DbCore::isExternal(const TokenId& token){
    return onTimeline(token, Agent::EXTERNAL_TIMELINE());
  }

  void DbCore::applyFacts(const std::vector<TokenId>& facts){
    for(std::vector<TokenId>::const_iterator it = facts.begin(); it != facts.end(); ++it){
      TokenId token = *it;

      checkError(isInternal(token), "Cannot only state facts on internal timelines. " << tokenToString(token));

      // Restrict start bound base domain
      IntervalIntDomain startBound(0, 0);
      startBound.intersect(token->start()->baseDomain());
      checkError(!startBound.isEmpty(), "Invalid start bound " << token->start()->lastDomain().toString());
      token->start()->restrictBaseDomain(startBound);

      // Restrict end bound base domain
      IntervalIntDomain endBound(1, PLUS_INFINITY);
      endBound.intersect(token->end()->baseDomain());
      checkError(!endBound.isEmpty(), "Invalid end bound " << token->end()->lastDomain().toString());
      token->end()->restrictBaseDomain(endBound);

      // Insert facts first
      token->activate();
      TimelineId timeline = (TimelineId) token->getObject()->lastDomain().getSingletonValue();
      timeline->constrain(token, token);
    }
  }

  std::map<PlanDatabaseId, DbCoreId>& DbCore::instancesByDb(){
    static std::map<PlanDatabaseId, DbCoreId> sl_table;
    return sl_table;
  }

  DbCoreId DbCore::getInstance(const TokenId& token){
    if(instancesByDb().empty())
      return DbCoreId::noId();

    PlanDatabaseId db = token->getPlanDatabase();
    std::map<PlanDatabaseId, DbCoreId>::const_iterator it = instancesByDb().find(db);
    checkError(it != instancesByDb().end(), "No db core for " << tokenToString(token));
    DbCoreId dbCore = it->second;
    checkError(dbCore.isValid(), dbCore);
    return dbCore;
  }

  /**
   * @brief Synchronize the plan and the observations.
   */
  bool DbCore::synchronize(){

    processPendingTokens();
    
    bool solver_timed_out = isSolverTimedOut();
    bool external_timeline_complete_fail = !completeExternalTimelines();
    bool resolve_fail = !m_synchronizer.resolve();

    if(solver_timed_out || external_timeline_complete_fail || resolve_fail){
      // Undo any impacts of solver. Reset this before making any deletions to avoid corrupting the stack
      m_solver->reset();

      // Remove any tokens that have been recalled for this reactor
      bool discardCurrentValues = processRecalls();

      // Now bring our tick up to speed
      m_currentTickCycle = getCurrentTick();

      // Revert to INACTIVE state
      m_state = DbCore::INACTIVE;
      
      // First just try to relax and resolve. 
      bool relax_fail = !m_synchronizer.relax(false);
      bool resolve_fail = !m_synchronizer.resolve();

      // If this fails the first time, apply a stronger relaxation where we discard current values that are not persistent.
      if(discardCurrentValues || relax_fail || resolve_fail){
	// Cleare the state again
	m_state = DbCore::INACTIVE;

	bool relax_fail = !m_synchronizer.relax(true);
	bool resolve_fail = !m_synchronizer.resolve();
	if( relax_fail || resolve_fail) {
	  return false;
	}
      }

      TREX_INFO("DbCore:synchronize", nameString() <<  "Repaired Database Below" << std::endl << PlanDatabaseWriter::toString(m_db));

      debugMsg("trex:info:planning", logPlan("Repaired Plan"));
    }

    // These final steps must succeed or synchronization will fail. If they do not succeed
    // the database will be marked invalid. Each operation below will be a NOP if attempted
    // on an invalid database
    commit();
    notifyObservers();
    updateGoals();
    archive();

    TREX_INFO("DbCore:synchronize", nameString() <<  "Synchronized Database Below" << std::endl << PlanDatabaseWriter::toString(m_db));
    
    // Write the nominal reactor state files (low bandwidth)
    TREX_INFO("trex:monitor:nominal", nameString() << dumpState(false));

    return m_state != DbCore::INVALID;
  }

  bool DbCore::processRecalls(){
    bool recalls = false;
    for(unsigned int i=0;i<m_recallBuffer.size(); i++){
      int tokenKey = m_recallBuffer[i];
      EntityId entity = Entity::getEntity(tokenKey);
      checkError(entity.isNoId() || (entity.isValid() && TokenId::convertable(entity)), "Buffer out of synch for " << tokenKey);
      if(entity.isId()){
	TokenId token = (TokenId) entity;
	token->discard();
	recalls = true;
      }
    }

    m_recallBuffer.clear();
    return recalls;
  }


  bool DbCore::inScope(const TokenId& token) const {
    std::map<int, bool>::const_iterator it = m_tokenScope.find(token->getKey());
    checkError(it != m_tokenScope.end(), 
	       tokenToString(token) << " not handled correctly on creation. Must process pending tokens before calling this");

    return it->second;
  }

  bool DbCore::supportedByObservation(const TokenId& token){
    if(isObservation(token))
      return true;

    const TokenSet& mergedTokens = token->getMergedTokens();
    for(TokenSet::const_iterator it = mergedTokens.begin(); it != mergedTokens.end(); ++it){
      TokenId mergedToken = *it;
      if(isObservation(mergedToken))
	return true;
    }

    return false;
  }

  /**
   * @brief This can be greatly optimized. The exact test every time is expensive in principle but may not
   * matter for our problem set at this time.
   */
  bool DbCore::hasPendingPredecessors(const TokenId& ctoken, const std::vector<TokenId>& uncontrollables, bool requireDifferentObject){
    ConstrainedVariableId c = ctoken->start();
    for(std::vector<TokenId>::const_iterator it = uncontrollables.begin(); it != uncontrollables.end(); ++it){
      TokenId  token = *it;
      ConstrainedVariableId u = token->end();

      if (getObjectName(token) == getObjectName(ctoken) && requireDifferentObject) {
	continue;
      }

      // Finally, if the candidate end time is a positive distance from the actions start time then this action
      // must wait until the predecessor is finished
      const IntervalIntDomain distanceBounds(m_db->getTemporalAdvisor()->getTemporalDistanceDomain(u, c, true));

      TREX_INFO("DbCore:hasPendingPredecessors",
	       "Distance between " << u->toString() << " and " << c->toString() << " is " << distanceBounds.toString());

      // It is possible that the temporal network is inconsistent in which case it will give the result of an empty domain. 
      // It would be ideal if propagation caught that but it does not appear to!
      if(distanceBounds.isEmpty()){
	markInvalid("Detected an inconsistency in the temporal network when evaluating actions for execution. To investigate, enable ConstraintEngine in Debug.cfg",true);
	return true;
      }

      // We have a distance bound
      if(distanceBounds.getLowerBound() >= 0 && !distanceBounds.isSingleton()){
	TREX_INFO("trex:warning:dispatching", tokenToString(token) << " must finish before " << tokenToString(ctoken) << " can be dispatched. The temporal distance between them is:" << distanceBounds.toString());
	TREX_INFO("DbCore:hasPendingPredecessors", tokenToString(token) << " must finish first.");
	return true;
      }
    }

    return false;
  }

  bool DbCore::isCurrentObservation(const TokenId& token){
    if(m_observations.find(token) == m_observations.end())
      return false;

    TimelineId timeline =  (TimelineId) token->getObject()->baseDomain().getSingletonValue();
    std::map<int, TimelineContainer>::const_iterator it = m_externalTimelineTable.find(timeline->getKey());
    checkError(it !=  m_externalTimelineTable.end(), tokenToString(token));
    return (it->second.lastObserved() == token->start()->baseDomain().getUpperBound());
  }

  void DbCore::handleAddition(const TokenId& token){
    m_pendingTokens.insert(token);
  }

  void DbCore::handleMerge(const TokenId& token){
    removeFromTokenAgenda(token);
  }

  void DbCore::handleSplit(const TokenId& token){
    addToTokenAgenda(token);
  }

  void DbCore::handleActivated(const TokenId& token){
    removeFromTokenAgenda(token);
  }

  void DbCore::handleDeactivated(const TokenId& token){
    addToTokenAgenda(token);
  }

  void DbCore::handleRemoval(const TokenId& token){
    m_tokenScope.erase(token->getKey());
    m_goals.erase(token);
    m_observations.erase(token);
    removeFromTokenAgenda(token);
    m_pendingTokens.erase(token);

    removeEntity(token);

    if(token->isCommitted()){
      m_committedTokens.erase(token);
    }

    m_terminableTokens.erase(token);

    // Finally, if the token belongs to an external timeline, and it was dispatched, we want to
    // unbuffer it.
    if(token->getObject()->lastDomain().isSingleton()){
      ObjectId object = token->getObject()->lastDomain().getSingletonValue();
      std::map<int, TimelineContainer>::iterator it = m_externalTimelineTable.find(object->getKey());
      if(it != m_externalTimelineTable.end())
	it->second.handleRemoval(token);
    }
  }

  void DbCore::handleCommitted(const TokenId& token){
    m_committedTokens.insert(token);
  }

  void DbCore::handleTerminated(const TokenId& token){
    m_terminatedTokens.insert(token);
  }

  void DbCore::handleRejected(const TokenId& token){
    TREXLog() << nameString() << "Rejected " << tokenToString(token) << std::endl;
    TREX_INFO("trex:warning", nameString() << tokenToString(token) << " was rejected.\n\n" <<
	      "   If this is a surprise, then you need to enable planner debug messages to investigate: \n" << 
	      "     :Solver:step - useful for observing the backtracking search.\n" << 
	      "     :trex:debug:planning - gives additional trex related information in the search in terms of flaw filtering etc.");

    removeFromTokenAgenda(token);
  }

  /**
   * Restrict base domain for all but the end time. That can only be restricted by the current tick. All others
   * can be restricted because the past is monotonic.
   */
  void DbCore::commitAndRestrict(const TokenId& token){
    TREX_INFO("trex:debug:synchronization:commitAndRestrict", "Committing " << tokenToString(token));

    // Commit the token and touch the state variable to trigger commit event based propagation
    token->commit();
    token->getState()->touch();

    // Propagate constraints before binding attribute base domains.
    propagate();

    token->getObject()->restrictBaseDomain(token->getObject()->lastDomain());

    // Restrict the start to its current bounds
    token->start()->restrictBaseDomain(token->start()->lastDomain());
    
    // restrict parameters
    restrictParameterBaseDomains(token);

    // Restrict the end. If in the past just restrict it to current bounds. If current, then the base domain can be restricted
    // just up till the current tick, even if the lower bound of the current domain is more restrictive.
    if(token->end()->lastDomain().getUpperBound() < getCurrentTick()){
      token->end()->restrictBaseDomain(token->end()->lastDomain());
    }
    else
      token->end()->restrictBaseDomain(IntervalIntDomain(getCurrentTick(), PLUS_INFINITY));
  }

  /**
   * Utility to iterate over parameters to restict base domains of each
   */
  void DbCore::restrictParameterBaseDomains(const TokenId& token){
    for(std::vector<ConstrainedVariableId>::const_iterator it = token->parameters().begin(); it != token->parameters().end(); ++it){
      ConstrainedVariableId p = *it;
      p->restrictBaseDomain(p->lastDomain());
    }
  }

  std::ostream& DbCore::writeDomain(std::ostream &out, 
				    AbstractDomain const &domain,
				    bool singletonOnly) {
    if( domain.isEmpty() )
      return out<<"{}";
    else if( domain.isSingleton() ) 
      return out<<domain_val_to_str(domain, domain.getSingletonValue(), true);
    else if( singletonOnly )
      return out<<'*';
    else if( domain.isEnumerated() )
      return out<<"{...}";
    else if( domain.isInterval() ) {
      double lower, upper;
      
      domain.getBounds(lower, upper); 
      if( MINUS_INFINITY==lower && PLUS_INFINITY==upper )
	  return out<<'*';
      else
	return out<<"["<<domain_val_to_str(domain, lower, true)
		  <<" "<<domain_val_to_str(domain, upper, true)
		  <<"]"; 
    } else
      return out<<"???";
  }

  void DbCore::logToken(TokenId const &tok) {
    // Only log active tokens
    if(!tok->isActive())
      return;

    TempVarId tokStart = tok->start(), tokEnd = tok->end();

//     if( tokEnd->lastDomain().getUpperBound()<getCurrentTick() )
//       return; // I don't display the past
//     else {
    std::vector<ConstrainedVariableId> const &vars = tok->parameters();
    std::vector<ConstrainedVariableId>::const_iterator vit = vars.begin();
    std::vector<ConstrainedVariableId>::const_iterator const endv = vars.end();

    writeDomain(m_planLog<<"\tfrom ", tokStart->lastDomain(), false);
    writeDomain(m_planLog<<" to ", tokEnd->lastDomain(), false);
    m_planLog<<" "<<tok->getPredicateName().toString()<<'(';
    if( endv!=vit ) {
      writeDomain(m_planLog, (*vit)->lastDomain(), true);
      ++vit;
      for(; endv!=vit ;++vit)
	writeDomain(m_planLog<<", ", (*vit)->lastDomain(), true);
    }
    m_planLog<<")"<<std::endl;
//     }
  }

  void DbCore::logTimeLine(TimelineId const &tl, std::string const &type) {
    std::list<TokenId> const &tokens = tl->getTokenSequence();
    std::list<TokenId>::const_iterator tokit = tokens.begin();
    std::list<TokenId>::const_iterator const endtok = tokens.end();
    
    m_planLog<<"  - ["<<type<<"] "<<tl->getName().toString()<<" :\n";
    for( ;endtok!=tokit; ++tokit )
      logToken(*tokit);
  }

  std::string DbCore::logPlan(const std::string& msg) {
    m_planLog <<'['<<getCurrentTick()<<"] " << msg << " :\n";

    // Internals 
    std::vector< std::pair<TimelineId, TICK> >::const_iterator 
      intit = m_internalTimelineTable.begin();
    std::vector< std::pair<TimelineId, TICK> >::const_iterator const
      endint = m_internalTimelineTable.end();
    
    for( ; endint!=intit; ++intit )
      logTimeLine(intit->first, "internal");

    // Externals
    std::map<int, TimelineContainer>::const_iterator 
      cextit = m_externalTimelineTable.begin();
    std::map<int, TimelineContainer>::const_iterator const
      endcext = m_externalTimelineTable.end();
    
    for( ; endcext!=cextit; ++cextit )
      logTimeLine(cextit->second.getTimeline(), "external");

    m_planLog << std::endl;

    return msg;
  }

  bool DbCore::verifyEntities(){
    std::set<EntityId> entities;
    Entity::getEntities(entities);
    for(std::set<EntityId>::const_iterator it = entities.begin(); it != entities.end(); ++it){
      EntityId entity = *it;
      checkError(entity.isValid(), entity);
    }

    return true;
  }

  void DbCore::deactivateIgnoredToken(const TokenId& token){
    token->start()->deactivate();
    token->end()->deactivate();
    token->duration()->deactivate();
    const std::vector<ConstrainedVariableId>& params = token->parameters();
    for (unsigned int i=1; i<params.size(); i++){
      ConstrainedVariableId var = params[i];
      var->deactivate();
    }
  }

  void DbCore::bufferObservation(const TokenId& token){
    // Then we place the new observation
    m_observations.insert(token);
  }

  bool DbCore::canBeTerminated(const TokenId& token) const {
    TokenId master = token->master();
    if(master.isNoId())
      return true;

    // If the master is terminated or in the past we can forget about its slave
    if(master->isTerminated() || (getCurrentTick() > master->end()->baseDomain().getUpperBound()))
       return true;

    // This token cannot be terminated if there are outstanding rule instances belonging to the master that depend
    // on this token
    const RulesEngineId& re = m_assembly.getRulesEngine();
    std::set<RuleInstanceId> rule_instances;
    re->getRuleInstances(master, rule_instances);
    for(std::set<RuleInstanceId>::const_iterator it = rule_instances.begin(); it != rule_instances.end(); ++it){
      RuleInstanceId ri = *it;
      const std::vector<ConstrainedVariableId>& guards = ri->getGuards();
      for(std::vector<ConstrainedVariableId>::const_iterator g_it = guards.begin(); g_it != guards.end(); ++g_it){
	ConstrainedVariableId guard = *g_it;
	checkError(guard.isValid(), "Invalid guard");
	if(guard->getParent() == token)
	  return false;
      }
    }

    return !re->hasPendingRuleInstances(master);
  }

  // Is there a current observation that matches the token
  bool DbCore::observedNow(const TokenId token) const {

    const std::vector<ConstrainedVariableId>& token_scope = token->getVariables();

    for(TokenSet::iterator it = m_observations.begin(); it != m_observations.end(); ++it){
      TokenId observation = *it;

      // If the predicates match, evaluate all the bounds for an intersection
      if(token->getPredicateName() == observation->getPredicateName()){
	const std::vector<ConstrainedVariableId>& observation_scope = observation->getVariables();
	checkError(observation_scope.size() == token_scope.size(), observation_scope.size() << " != " << token_scope.size());
	unsigned int i = token_scope.size() - 1;
	while (i > 0 && token_scope[i]->lastDomain().intersects(observation_scope[i]->lastDomain())){
	  i--;
	}

	// A match if we get to zero
	if(i == 0){
	  return true;
	}
      }
    }

    return false;
  }

  void DbCore::disconnectConstraints(const TokenId& token){
    // Compile all vars and constraints
    std::set<int> allVars;
    ConstraintSet constraints;

    for(std::vector<ConstrainedVariableId>::const_iterator it = token->getVariables().begin(); it != token->getVariables().end(); ++it){
      ConstrainedVariableId var = *it;
      var->constraints(constraints);
      allVars.insert(var->getKey());
    }

    for(ConstrainedVariableSet::const_iterator it = token->getLocalVariables().begin(); it != token->getLocalVariables().end(); ++it){
      ConstrainedVariableId var = *it;
      var->constraints(constraints);
      allVars.insert(var->getKey());
    }

    // Now go through all constraints
    for(ConstraintSet::const_iterator it = constraints.begin(); it != constraints.end(); ++it){
      ConstraintId constraint = *it;
      checkError(constraint.isValid(), constraint);

      // No problem if the constraint has been deactivated already
      if(!constraint->isActive() || constraint->isRedundant())
	continue;

      // If it is active, then we should ensure it has at least one external variable
      const std::vector<ConstrainedVariableId>& scope = constraint->getScope();
      for(unsigned int i=0;i<scope.size();i++){
	ConstrainedVariableId var = scope[i];

	// If the variable has no parent, its scope is not defined temporally. This is typically only arising
	// in initialization.
	if (var->parent().isNoId())
	  continue;

	// If not a variable of this token, and its base domain is not a singleton, restrict it
	if(allVars.find(var->getKey()) == allVars.end() && !var->baseDomain().isSingleton())
	  var->restrictBaseDomain(var->lastDomain());
      }
    }
  }


  TokenId DbCore::getValue(const TimelineId& timeline, TICK tick){
    TokenId v;

    const std::list<TokenId>& tokenSequence = timeline->getTokenSequence();
    for(std::list<TokenId>::const_iterator it = tokenSequence.begin(); it != tokenSequence.end(); ++it){
      TokenId token = *it;
      checkError(token.isValid(), token);
      TICK latestStart = (TICK) token->start()->lastDomain().getUpperBound();
      TICK latestEnd = (TICK) token->end()->lastDomain().getUpperBound();

      // If we find a token in the future then we are done
      if(latestStart > tick)
	break;

      // If it is in the past move on
      if(latestEnd < tick)
	continue;

      v = token;
    }

    return v;
  }

  bool DbCore::isSolverTimedOut() {
    // Test for conflict with the planner horizon. Don't want to integrate
    // interim sub-goals into synchronization
    if(m_state == DbCore::ACTIVE){
      TICK lb, ub;
      getHorizon(lb, ub);
      if(lb < getCurrentTick()){
	TREX_INFO("trex:warning:planning",  nameString() << "Timed out finding a plan.");
	TREXLog() << nameString() << "Planning failed to complete in time." << std::endl;
	markInvalid("The solver could not complete in time. You might have excessive logging, or excessive search. To investigate the latter, enable Solver:step in Debug.cfg",true);
	return true;
      }
    }

    return false;
  }

  bool DbCore::inDeliberation(const TokenId& token) const {
    // If the token is rejectable, definitely is
    if(token->getState()->lastDomain().isMember(Token::REJECTED))
      return true;

    // If planning is inactive, or the token is committed, there is no problem.
    if(m_state != DbCore::ACTIVE || token->isCommitted())
      return false;

    // Now check the decision stack
    if(m_solver->inDeliberation(token))
      return true;

    return false;
  }

  void DbCore::markInvalid(const std::string& comment, const bool dump_state, const std::string& analysis) {
    // Recall dispatched goals if transitioning into this state
    if(m_state != DbCore::INVALID)
      dispatchRecalls();

    // Output log line
    TREX_INFO("trex:warning", nameString() << " is marked invalid. Hint:" << comment);
    if(dump_state) {
      TREX_INFO("trex:monitor:conflicts:nominal", nameString() << "Dumping conflict: " << writeConflict(comment, analysis)); 
    }
    m_state = DbCore::INVALID;
  }


  /**
   * @brief Cleans up keys floating around when a goal is removed
   */
  void DbCore::cleanupGoal(const TokenId& goal){
    // Finally, handle foreign key mapping if posted from another reactor
    EntityId foreign = getForeignEntity(goal);
    TokenId foreignToken;
    if(foreign.isId()){
      foreignToken = foreign;
      removeEntity(foreign);
    }

    const std::vector<ConstrainedVariableId>& goalVars = goal->getVariables();
    unsigned int count = goalVars.size();
    for(unsigned int i = 0; i < count; i++){
      if(foreignToken.isId())
	removeEntity(foreignToken->getVariables()[i]);
    }

    m_notificationKeys.erase(goal->getKey());
  }

  /**
   * @brief Iterate over committed external tokens with a pending end time. These are the uncontrollable events of interest
   */
  void DbCore::getActiveUncontrollableEvents(std::vector<TokenId>& results){
    for(TokenSet::const_iterator it = m_committedTokens.begin(); it != m_committedTokens.end(); ++it){
      TokenId token = *it;
      checkError(token.isValid(), token);

      const AbstractDomain& endDom = token->end()->lastDomain();
      if(isExternal(token) && endDom.isMember(getCurrentTick()) && !endDom.isSingleton()){
	TREX_INFO("DbCore:getActiveUncontrollableEvents", "Adding " << tokenToString(token));
	results.push_back(token);
      }
    }
  }

  /**
   * @brief Iterate over pending tokens and process as actions or goals as appropriate
   */
  void DbCore::processPendingTokens(){
    for(TokenSet::const_iterator it = m_pendingTokens.begin(); it != m_pendingTokens.end(); ++it){
      TokenId token = *it;
      bool inScope = true;

      checkError(m_assembly.getSchema()->isA(token->getBaseObjectType(), Agent::TIMELINE()), tokenToString(token));
      inScope = !DbCore::onTimeline(token, Agent::IGNORE_TIMELINE());

      // If in scope, and rejectable it must be a goal
      if(inScope && token->master().isNoId() && token->getState()->baseDomain().isMember(Token::REJECTED)){
	m_goals.insert(token);
	TREX_INFO("DbCore:handleAddition", nameString() << "Adding Goal:" << tokenToString(token));
	token->start()->restrictBaseDomain(IntervalIntDomain(0, Agent::instance()->getFinalTick() - 1));

	// Restrict parameters since it is a goal.
	restrictParameterBaseDomains(token);

	// If not a condition, then we will not require any end time. It can be made explicit
	if(!TestMonitor::isCondition(token->getKey()))
	  token->end()->restrictBaseDomain(IntervalIntDomain(0, Agent::instance()->getFinalTick()));
      }
 
      m_tokenScope.insert(std::pair<int, bool>(token->getKey(), inScope));

      // If not in scope, deactivate all its variables
      if(!inScope)
	deactivateIgnoredToken(token);
      else
	addToTokenAgenda(token);
    }

    m_pendingTokens.clear();
  }

  void DbCore::setDispatchTime(const TokenId& token) const {
    ConstrainedVariableId var = dispatch_time(token);
    if(var.isId())
      var->specify(getCurrentTick());
  }

  void DbCore::resetDispatchTime(const TokenId& token) const {
    ConstrainedVariableId var = dispatch_time(token);
    if(var.isId())
      var->reset();
  }

  ConstrainedVariableId DbCore::dispatch_time(const TokenId& token) {
    static const LabelStr PARAM_DISPATCH_TIME("dispatch_time");
    static const LabelStr PARAM_TYPE_INT("int");
    ConstrainedVariableId var = token->getVariable(PARAM_DISPATCH_TIME);
    checkError(var.isNoId() || var->lastDomain().getTypeName() == PARAM_TYPE_INT, var->toString());
    return var;
  }

  bool DbCore::timelinesAreComplete(){
    std::list<ObjectId> objects;
    m_db->getObjectsByType(Agent::TIMELINE(), objects);

    for(std::list<ObjectId>::const_iterator it = objects.begin(); it != objects.end(); ++it){
      TimelineId timeline = (TimelineId) *it;
      const std::list<TokenId>& tokenSequence = timeline->getTokenSequence();
      TokenId pred;
      for(std::list<TokenId>::const_iterator t_it = tokenSequence.begin(); t_it != tokenSequence.end(); ++t_it){
	TokenId token = *t_it;
	checkError(token.isValid(), token);

	// If the token is not committed, and it has a predecessor, then there should be no distance allowed between it and its predecessor
	if(!token->isCommitted() && pred.isId()){
	  IntervalIntDomain separationDistance = 
	    m_db->getTemporalAdvisor()->getTemporalDistanceDomain(pred->end(), token->start(), true);

	  // If there is a gap, bridge it
	  if(separationDistance.getUpperBound() > 0){
	    TREX_INFO("trex:warning:planning", nameString() << tokenToString(token) << " is not constrained to succeed " << pred->toString() <<
		     " Posting concurrency constraint to integrate the plan.");

	    m_db->getConstraintEngine()->createConstraint("concurrent", makeScope(pred->end(), token->start()));

	    if(!propagate())
	      return false;

	    break;
	  }
	}

	pred = token;
      }
    }

    return true;
  }

  /**
   * If we are missing an expected observation, then we want to know:
   * The time
   * If it is because of an expected end time - and a hole
   * Was the expected token dispatched, and if so, when.
   */ 
  std::string DbCore::missingObservation(const TimelineId& timeline) const {
    std::stringstream ss;

    ss << "At tick [" << getCurrentTick() << ") - Missing Observation on " << timeline->getName().toString() << ". Analysis below:" << std::endl;

    // Find the expected token
    TokenId expectedToken;
    TokenId predecessor;
    const std::list<TokenId>& tokenSequence = timeline->getTokenSequence();
    for(std::list<TokenId>::const_iterator it = tokenSequence.begin(); it != tokenSequence.end(); ++it){
      TokenId token = *it;
      checkError(token.isValid(), token);

      if(token->start()->lastDomain().getUpperBound() == getCurrentTick()){
	expectedToken = token;
	break;
      }

      predecessor = token;
    }

    if(expectedToken.isId()){
      ss << std::endl << "Expected to get an observation for " << expectedToken->toString();
      if(inDeliberation(expectedToken))
	ss << " which is still part of active deliberation. Check if the solver is converging as expected. Solver stats: depth[" << m_solver->getDepth() << "] step_count[" << m_solver->getStepCount() << "]";
      ss << std::endl;
    }

    if(predecessor.isId()){
      ss << std::endl << "Continuing predecessor value " << predecessor->toString() << std::endl;
      ss << std::endl << m_synchronizer.localContextForConstrainedVariable(predecessor->end());
    }

    std::map<int, TimelineContainer>::const_iterator it = m_externalTimelineTable.find(timeline->getKey());
    const TimelineContainer& tc = it->second;
    const TokenSet& dispatched_tokens = tc.getDispatchedTokens();

    if(dispatched_tokens.empty()){
      ss << std::endl << "No dispatched tokens buffered. Perhaps the dispatch window is configured incorrectly or else we are not planning ahead sufficiently to dispatch expected values.";

      if(m_lastCompleteTick == MINUS_INFINITY)
	ss << " Planning was never completed.";
      else
	ss << " Note that planning was last completed at tick [" << m_lastCompleteTick << "]";

      ss << std::endl << std::endl;
    }
    else {
      for(TokenSet::const_iterator it = dispatched_tokens.begin(); it != dispatched_tokens.end(); ++it){
	TokenId token = *it;
	checkError(token.isValid(), token);
	ss << "Dispatched " << tokenToString(token) << " start == " << token->start()->lastDomain().toString() << " && end == " <<  token->end()->lastDomain().toString() << std::endl;
      }
    }

    ss << std::endl << std::endl;

    // Find the last observation
    for(TokenSet::iterator it = m_observations.begin(); it != m_observations.end(); ++it){
      TokenId observation = *it;
      if(observation->getObject()->getSpecifiedValue() == timeline)
	ss << "Observed " << observation->toString() << " at tick[" << observation->start()->lastDomain().toString() << "]" << std::endl;
    }

    ss << std::endl << "Database Below" << std::endl << PlanDatabaseWriter::toString(m_db);

    ss << "*****************************************************" << std::endl;

    return ss.str();
  }

  void DbCore::addToTokenAgenda(const TokenId& token){
    if(!token->isDiscarded())
      m_tokenAgenda.insert(token);

    TREX_INFO("trex:debug:tokenAgenda:addToTokenAgenda", tokenToString(token));
  }

  void DbCore::removeFromTokenAgenda(const TokenId& token){
    m_tokenAgenda.erase(token);
    TREX_INFO("trex:debug:tokenAgenda:removeFromTokenAgenda", tokenToString(token));
  }

  /**
   * Analysis of the execution frontier to identify problems in archiving tokens
   *
  std::string DbCore::analyzeCandidatesForArchiving(){
    std::stringstream ss;

    // Iterate over all tokens in the database. If any are strictly in the past, then we should wonder why they are not archived

    const TokenSet& all_tokens = m_db->getTokens();
    for(TokenSet::const_iterator it = all_tokens.begin(); it != all_tokens.end(); ++it){
      TokenId token = *it;
      if(token->end().lastDomain().getUpperBound() < getCurrentTick() - 1){


      }

    }
  }
  */


  TREX_REGISTER_REACTOR(DbCore, DeliberativeReactor);
}
