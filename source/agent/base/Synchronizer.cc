/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2007. MBARI.
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

#include "Synchronizer.hh"
#include "DbCore.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "PlanDatabaseWriter.hh"
#include "PlanDatabase.hh"
#include "Timeline.hh"
#include "Agent.hh"
#include "Utilities.hh"


namespace TREX {

  Synchronizer::Synchronizer(const DbCoreId& _core) 
    : m_core(_core), 
      m_db(m_core->m_db),
      m_timelines(m_core->m_timelines),
      m_goals(m_core->m_goals), 
      m_observations(m_core->m_observations), 
      m_tokenAgenda(m_core->m_tokenAgenda),
      m_committedTokens(m_core->m_committedTokens){}

  /**
   * Will be in the horizon if start.ub <= (tao) && end.lb >= tao
   */
  bool Synchronizer::inTickHorizon(const TokenId& token, int currentTick){
    return token->start()->lastDomain().getUpperBound() <= currentTick && token->end()->lastDomain().getLowerBound() >= currentTick && token->end()->lastDomain().getLowerBound() > 0;
  }

  /**
   * @brief Will be in scope if in the tick horizon and in scope for this reactor, and a unit decision
   */
  bool Synchronizer::inSynchScope(const TokenId& token, TokenId& mergeCandidate){
    if(inTickHorizon(token, m_core->getCurrentTick()) && 
       m_core->inScope(token) && !m_core->inDeliberation(token) && isUnit(token, mergeCandidate))
	return true;

    return false;
  }

  /**
   * @brief Will be a unit decision if it has only one option to be resolved or if it has a specific position
   * in the plan. The latter case arises where we have a token that must go in a specific time slot but which
   * could merge onto the plan or insert and nudge the plan. The resolution model will only try one. If that fails, it will
   * relax the plan. 
   * @note Any token that is rejectable is not in scope.
   */
  bool Synchronizer::isUnit(const TokenId& token, TokenId& merge_candidate){
    if(!token->getObject()->lastDomain().isSingleton())
      return false;

    // Compute compatible tokens, using an exact test
    std::vector<TokenId> compatible_tokens;
    m_db->getCompatibleTokens(token, compatible_tokens, PLUS_INFINITY, true);

    // Iterate over tokens to find one suitable for merging in synchronization
    unsigned int merge_choice_count(0);
    for(std::vector<TokenId>::const_iterator it = compatible_tokens.begin(); it != compatible_tokens.end(); ++it){
      TokenId candidate = *it;

      // If the token in question is in deliberation, continue
      if(m_core->inDeliberation(candidate)){
	TREX_INFO("trex:warning:synchronization", candidate->toString() << " cannot be used because it is in delibertion.");
	continue;
      }

      merge_choice_count++;
      merge_candidate = candidate;

      // If we have more than one option, break out of loop since this is not a unit decision
      if(merge_choice_count > 1)
	break;
    }

    // If we have only one option to merge onto, and we have nowhere to insert the token, then we will have a unit decision
    if(merge_choice_count == 1 && !m_db->hasOrderingChoice(token)){
      TREX_INFO("trex:debug:synchronization", "Found unit decision for " << token->toString() <<
	       " with start = " << token->start()->lastDomain().toString() << ". One spot to merge it.");
      return true;
    }

    if(merge_choice_count == 0){
      merge_candidate = TokenId::noId();

      TREX_INFO("trex:debug:synchronization", "Found unit decision for " << token->toString() <<
	       " with start = " << token->start()->lastDomain().toString() << ". No ordering choice.");

      return true;
    }

    TREX_INFO("trex:debug:synchronization", "Excluding " << token->toString());
    return false;
  }

  ConstrainedVariableId Synchronizer::getActiveGuard(const ConstrainedVariableId& var){
    EntityId parent = var->parent();
    if(parent.isId() && TokenId::convertable(parent)){
      TokenId token = parent;

      // If the token has been merged, we want the underlying variable
      if(token->isMerged())
	return token->getActiveToken()->getVariables()[var->getIndex()];

      // No binding rules on inactive tokens
      if(token->isInactive())
	return ConstrainedVariableId::noId();
    }

    return var;
  }

  /*
   * @brief Resolve unit flaws at the execution frontier
   */
  bool Synchronizer::resolve(){
    // Use a counter to aid with settng debug breakpoints
    static unsigned int sl_counter(0);
    sl_counter++;

    if(!m_core->propagate()){

      TREX_INFO("trex:debug:synchronization", m_core->nameString() << 
		"Constraint network inconsistent after propagation. Cannot output database.");

      return false;
    }

    TREX_INFO("trex:debug:synchronization", m_core->nameString() << 
	      "Database prior to synchronization:\n" << PlanDatabaseWriter::toString(m_db));

    checkError(m_core->isValidDb(), "Invalid database before synchronization.");

    m_stepCount = 0; // Reset step counter for stats
    if(resolveTokens(m_stepCount) &&
       completeInternalTimelines(m_stepCount) &&
       resolveTokens(m_stepCount)){
      TREX_INFO("trex:debug:synchronization", m_core->nameString() << 
		"Database after successful synchronization:\n" << PlanDatabaseWriter::toString(m_db));
      return true;
    }

    return false;
  }

  /**
   * @brief Relax the plan at the execution frontier, but keep what is entailed by prior state.
   * the model, and current observations..
   */
  bool Synchronizer::relax(bool discardCurrentValues) {
    TREXLog() << m_core->nameString() << "Beginning database relax." << std::endl;

    TREX_INFO("trex:debug:synchronization:relax", m_core->nameString() << "START");

    // Reset observations to base values. It is important that we do this before processing
    // other tokens as we want to recover the current observation and the easiest way to
    // do that is to evaluate the end time of committed observations or observations that are
    // merged onto committed tokens. We cannot do the latter if they have been split, which
    // can happen when we reset goals or other commitments. We do not want any propagation to be
    // done during this reset since we will just be relaxing the database and re-propagating and
    // the system will be in an incomplete state in the interim while all resets are done.
    resetObservations();

    // Reset the goals, clearing out the crud
    resetGoals(discardCurrentValues);

    // Reset remaining tokens
    resetRemainingTokens(discardCurrentValues);

    // Purge bad links in foreign key table.
    m_core->purgeOrphanedKeys();

    checkError(m_core->verifyEntities(), "Bug somewhere.");

    TREX_INFO("trex:debug:synchronization:relax", m_core->nameString() << "Prior to insertion of copied values" << std::endl << PlanDatabaseWriter::toString(m_db));

    // Final step before trying again to resolve
    if(insertCopiedValues()){
      TREX_INFO("trex:debug:synchronization:relax", m_core->nameString() << "Relaxed Database Below" << std::endl << PlanDatabaseWriter::toString(m_db));
      return true;
    }

    TREXLog() << m_core->nameString() << "Relax failed." << std::endl;

    return false;
  }

  /**
   * @brief Relaxes goal commitments and removes those goals that are no longer achievable
   * @see relax
   */
  void Synchronizer::resetGoals(bool discardOpenGoals){
    TREX_INFO("trex:debug:synchronization:resetGoals", m_core->nameString() << 
	     "START with 'discard open goals' " << (discardOpenGoals ? "enabled." : "disabled."));

    std::vector<TokenId> past; /*!< Necessarily in the past */
    std::vector<TokenId> present; /*!< Committed goals we will want to make  a relaxed copy of */

    // Iterate over the goals and place in different buckets. We make no mods in this iteration since they can impact the
    // contents of the map, possibly removing goals altogether. This can be the case since some goals are sub-goals and can thus be deleted
    // if the master is relaxed or removed.
    TokenSet goals = m_goals;
    for(TokenSet::const_iterator it = goals.begin(); it != goals.end(); ++it){
      TokenId goal = *it;
      checkError(goal.isValid(), "Invalid goal:" << goal);
      checkError(goal->master().isNoId(), 
		 "Should only have orphans in the goal buffer. " << 
		 goal->toString() << " has master " << goal->getMaster()->toString());

      assertTrue(!goal->start()->baseDomain().isEmpty(), "Bad");

      const IntervalIntDomain& endTime = (goal->isMerged() ? goal->getActiveToken()->end()->baseDomain() : goal->end()->baseDomain());

      TREX_INFO("trex:debug:synchronization:resetGoals", m_core->nameString() << "Evaluating " << goal->toString() << " ending in " << endTime.toString());


      // Case 2: The goal is a current value it will be handled when we reset remaining tokens
      if(isCurrent(goal)){
	m_goals.erase(goal);
	TREX_INFO("trex:debug:synchronization:resetGoals", m_core->nameString() << "Removing goal but keeping value for:" << goal->toString());
	continue;
      }

      // Case 0: We are discarding open goals
      if(discardOpenGoals == true){
	TREX_INFO("trex:debug:synchronization:resetGoals", m_core->nameString() << "Open goal will be removed: " << goal->toString());
	past.push_back(goal);
	continue;
      }

      // Case 1: The goal must end in the past
      if(endTime.getUpperBound() <= m_core->getCurrentTick()){
	TREX_INFO("trex:debug:synchronization:resetGoals", m_core->nameString() << "Ends in the past: " << goal->toString());
	past.push_back(goal);
	continue;
      }

      // Case 3: The goal was previously rejected and cannot be started. This is also considered the past.
      if(goal->isRejected() && goal->start()->baseDomain().getUpperBound() < m_core->getCurrentTick()){
	TREX_INFO("trex:debug:synchronization:resetGoals", m_core->nameString() << "Rejected: " << goal->toString());
	past.push_back(goal);
	continue;
      }

      // Case 4: The goal is merged with a current value. We will remove the goal as it has already been started.
      if(goal->isMerged() && isCurrent(goal->getActiveToken())){
	TREX_INFO("trex:debug:synchronization:resetGoals", m_core->nameString() << "Merged with current value." << goal->toString());
	past.push_back(goal);
	continue;
      }

      // Case 5: The goal is active. It will be in the future so and it's parameters should be reset
      if(goal->isActive()){
	const std::vector<ConstrainedVariableId>& vars = goal->getVariables();
	for(std::vector<ConstrainedVariableId>::const_iterator it = vars.begin(); it != vars.end(); ++it){
	  ConstrainedVariableId var = *it;
	  if(var->canBeSpecified() && var->isSpecified())
	    var->reset();
	}
      }

      // Case 6: The goal cannot be planned in time
      if(goal->start()->baseDomain().getUpperBound() < m_core->getCurrentTick()){
	TREX_INFO("trex:debug:synchronization:resetGoals", m_core->nameString() << "Might have to start before we have time to plan.");
	past.push_back(goal);
	continue;
      }

      // Finally, if the goal remains !inActive, it must be cancelled and fixed in the future.
      if(!goal->isInactive()){
	goal->cancel();
	goal->start()->restrictBaseDomain(IntervalIntDomain(m_core->getCurrentTick(), PLUS_INFINITY));
      }
    }

    for(std::vector<TokenId>::const_iterator it = past.begin(); it != past.end(); ++it){
      TokenId token = *it;
      TREX_INFO("trex:debug:synchronization:resetGoals", m_core->nameString() << "Discarding goal: " << token->toString());
      m_core->cleanupGoal(token);
      token->discard();
    }


    TREX_INFO("trex:debug:synchronization:resetGoals", m_core->nameString() << "END");
  }

  /**
   * @brief Clear out observations, and set only the current observation. Assumes no splitting has
   * occurred yet. So must precede goal reset.
   */
  void Synchronizer::resetObservations(){
    static int sl_counter(0);
    sl_counter++;

    TREX_INFO("trex:debug:synchronization:resetObservations", m_core->nameString() << "[" << sl_counter << "]START");

    TokenSet observations = m_observations;
    for(TokenSet::iterator it = observations.begin(); it != observations.end(); ++it){
      TokenId observation = *it;
      checkError(observation.isValid(), observation);

      TREX_INFO("trex:debug:synchronization:resetObservations", m_core->nameString() << "Evaluating " << observation->toString());

      // If the observation contains merged tokens, they should all be cancelled.
      TokenSet mergedTokens = observation->getMergedTokens();
      for(TokenSet::const_iterator it = mergedTokens.begin(); it != mergedTokens.end(); ++it){
	TokenId m = *it;
	m->cancel();
      }

      if(m_core->isCurrentObservation(observation)){
	TREX_INFO("trex:debug:synchronization:resetObservations", m_core->nameString() << "Handling current observation " << observation->toString());

	// Relax if we can and if we must. If committed, see remaining Tokens
	if(!observation->isCommitted() && !observation->isInactive()){
	  TREX_INFO("trex:debug:synchronization:resetObservations", m_core->nameString() << "Relaxing " << observation->toString());
	  observation->cancel();
	}

	// We know we are synhronizing, so if this is the current value, we are extending it so that it must hold for this tick
	checkError(observation->end()->baseDomain().isMember(m_core->getCurrentTick() + 1), 
		   observation->toString() << " must end in " << observation->end()->baseDomain().toString());

	// The end variable might be specified, so reset if it is.
	if(observation->end()->isSpecified())
	  observation->end()->reset();
	else
	  observation->end()->relax();

	observation->end()->restrictBaseDomain(IntervalIntDomain(m_core->getCurrentTick() + 1, PLUS_INFINITY));
      }
      else
	observation->discard();
    }

    TREX_INFO("trex:debug:synchronization:resetObservations", m_core->nameString() << "END");
  }

  /**
   * @brief Handle the remainder
   * Assumes we have already reset goals and observations. Will also force a copy of current committed tokens to re-apply the model.
   * @see relax
   */
  void Synchronizer::resetRemainingTokens(bool discardCurrentValues){
    TREX_INFO("trex:debug:synchronization:resetRemainingTokens", m_core->nameString() << 
	     "START with discarding current internal values " << (discardCurrentValues ? "enabled." : "disabled."));

    std::vector<TokenId> tokensToDiscard;
    const TokenSet allTokens = m_db->getTokens();
    for(TokenSet::const_iterator it = allTokens.begin(); it!= allTokens.end(); ++it){
      TokenId token = *it;
      checkError(token.isValid(), token);

      const IntervalIntDomain& endTime = token->end()->baseDomain();

      TREX_INFO("trex:debug:synchronization:resetRemainingTokens", m_core->nameString() << "Evaluating " << token->toString() << " ending in " << endTime.toString());

      // Case 1: The value is current. This means it is committed, which implies it was previously found to be consistent
      // during synchronization. 
      if(isCurrent(token)){
	if(!discardCurrentValues || m_core->isObservation(token) || isPersistent(token))
	  copyValue(token);

	TREX_INFO("trex:debug:synchronization:resetRemainingTokens", "Scheduling discard for current value " << token->toString());
	tokensToDiscard.push_back(token);
	continue;
      }

      // Case 3: After excluding the above cases, as long as it is not a goal or observation, which will have been dealt with expliticly in
      // resetGoals and resetObservations, then we want to remove the token. New tokens will be regenerated by the model if implied.
      if(!m_core->isGoal(token) && !m_core->isObservation(token)){
	TREX_INFO("trex:debug:synchronization:resetRemainingTokens", "Scheduling discard for stale value" << token->toString());
	tokensToDiscard.push_back(token);
      }
    }

    // Now we clean up all the tokens we plan to discard.
    TREX_INFO("trex:debug:synchronization:resetRemainingTokens", "Discarding " << tokensToDiscard.size() << " tokens");

    Entity::discardAll(tokensToDiscard);

    TREX_INFO("trex:debug:synchronization:resetRemainingTokens", m_core->nameString() << "END");
  }

  /**
   * @brief True if a current value.
   */
  bool Synchronizer::isCurrent(const TokenId& token) {
    bool result = token->isCommitted() && !token->isTerminated();

    // Should not be in the past. Condition varies for internal vs. external tokens
    bool is_internal = m_core->isInternal(token);
    result = result && ( (is_internal && token->end()->baseDomain().getUpperBound() >= m_core->getCurrentTick()) ||
			 (!is_internal && token->end()->baseDomain().getUpperBound() > m_core->getCurrentTick()));

    // Can ignore actions -  we just regenerate them
    result = result &&  !m_core->isAction(token);
 
    return result;
  }

  /**
   * @brief Copies a current value to a new token which is also active and committed. We relax the new token
   * in its end time and in the set of applicable constraints
   */
  void Synchronizer::copyValue(const TokenId& source){
    // Allocate a token
    TokenId token = m_db->getClient()->createToken(source->getPredicateName().c_str(), DbCore::NOT_REJECTABLE);
    token->activate();

    // Pin all variables with published values of the token
    for(unsigned int i = 0; i < (unsigned int) source->parameters().size(); i++){
      token->parameters()[i]->restrictBaseDomain(source->parameters()[i]->baseDomain());
    }

    token->getObject()->restrictBaseDomain(source->getObject()->baseDomain());
    token->start()->restrictBaseDomain(source->start()->baseDomain());
    token->end()->restrictBaseDomain(IntervalIntDomain(m_core->getCurrentTick(), PLUS_INFINITY));

    if(m_core->isObservation(source)){
      // Current observations should have their values extended in case this did not happen correctly before pre-emption was required
      checkError(m_core->isCurrentObservation(source), "Should only be copying current observations. But copied " << source->toString());
      token->end()->restrictBaseDomain(IntervalIntDomain(m_core->getCurrentTick() + 1, PLUS_INFINITY));
      m_core->bufferObservation(token);
    }
    else if(m_core->isGoal(source)){
	token->end()->restrictBaseDomain(source->end()->baseDomain());
	m_goals.insert(token);
    }

    // Update the duration also, based on base domain values of start and end
    int durationMin = std::max((int) ( token->end()->baseDomain().getLowerBound() - token->start()->baseDomain().getUpperBound()), 0);
    token->duration()->restrictBaseDomain(IntervalIntDomain(durationMin, PLUS_INFINITY));

    // Finally, we commit the value. This is reasonable since we are copying a current value. All this approach allows
    // us to do is re-apply the model with internal decisions free to be taken anew. Public variables of the token
    // remain bound to prior valaues since the past is monotonic. Since we have already done the base domain restriction
    // and we want to prevent propagation while we are relaxing, we just commit directly
    token->commit();

    TREX_INFO("trex:debug:synchronization:copyValue",m_core->nameString() << "Replaced " << source->toString() << " with " << token->toString());
  }

  /**
   * Processes the token agenda and makes insertion or merge choices
   */
  bool Synchronizer::resolveTokens(unsigned int& stepCount){
    static unsigned int sl_counter;

    unsigned int lastCount = PLUS_INFINITY;
    while(lastCount != m_stepCount){
      lastCount = m_stepCount; // Update this to track changes on this iteration

      // Process tokens on the agenda that are in scope
      const TokenSet agenda = m_core->m_tokenAgenda;

      if(!m_core->propagate())
	return false;

      for(TokenSet::const_iterator it = agenda.begin(); it != agenda.end(); ++it){
	// Debugging Aid
	sl_counter++;

	TokenId token = *it;

	TREX_INFO("trex:debug:synchronization:resolveTokens", 
		  m_core->nameString() << "[" << sl_counter << "] Evaluating " << token->toString() <<
		 " Start = " << token->start()->toString() << " End = " << token->end()->toString());

	// Tokens that are unbound are ignored since they are not unit decisions
	if(!token->getObject()->lastDomain().isSingleton())
	  continue;

	// If not inactive, then it must have been resolved already
	if(!token->isInactive())
	  continue;

	// If outside the horizon or out of scope, skip it
	TokenId merge_candidate;
	if(!inSynchScope(token, merge_candidate))
	  continue;

	stepCount++;

	TREX_INFO("trex:debug:synchronization:resolveTokens", 
		 m_core->nameString() << "Resolving " << token->toString() << " IN " << 
		 std::endl << PlanDatabaseWriter::toString(m_db));

	// Resolve the token and ensure consistency in order to continue.
	if(!resolveToken(token, stepCount, merge_candidate) || !m_core->propagate())
	  return false;
      }
    }

    return true;
  }

  bool Synchronizer::resolveToken(const TokenId& token, unsigned int& stepCount, const TokenId& merge_candidate){
    if(mergeToken(token, merge_candidate) || insertToken(token, stepCount))
      return true;

    std::string explanation_str;
    TREX_INFO("trex:monitor:conflicts", m_core->nameString() << (explanation_str = tokenResolutionFailure(token, merge_candidate)));

    m_core->markInvalid(std::string("Could not insert ") + token->toString() + 
			" into the plan. The plan is not compatible with observations and must be relaxed. Enable all DbCore messages and also enable Synchronizer messages in the Debug.cfg file.", 
			true, explanation_str);
    return false;
  }

  /**
   * Notes:
   * 1. All copied values will have been committed
   * 2. Insertion has been deferred till after all cleanup has occured of prior tokens to avoid restrictions mingling
   *    with relaxations.
   * 3. We assume no copied values have been inserted yet
   */
  bool Synchronizer::insertCopiedValues(){
    TREX_INFO("trex:debug:synchronization:insertCopiedValues", 
	     m_core->nameString() << "Relaxed Database Prior to insertion of copied values" << std::endl << PlanDatabaseWriter::toString(m_db));

    for(TokenSet::const_iterator it = m_committedTokens.begin(); it != m_committedTokens.end(); ++it){
      TokenId token = *it;
      unsigned int stepCount = 0;
      if(!insertToken(token, stepCount)){
	TREX_INFO("trex:debug:synchronization:insertCopiedValues", m_core->nameString() << "Failed to insert " << token->toString());
	std::string explanation_str;
	TREX_INFO("trex:debug:synchronization:insertCopiedValues", m_core->nameString() << (explanation_str = tokenResolutionFailure(token, TokenId::noId())));
	m_core->markInvalid(std::string("Failed to insert ") + token->toString() + 
			    "This is bad. After relaxing the plan and restoring necessary state, we still can't synchronize. " + 
			    "There is probably a bug in the model. Enable PlanDatabase and DbCore messages in Debug.log",
			    true, explanation_str);
	return false;
      }
    }

    return true;
  }

  /**
   * Only merge it. True if we tried to merge, false if we did not.
   */
  bool Synchronizer::mergeToken(const TokenId& token, const TokenId& merge_candidate){
    if(m_core->isInvalid() || merge_candidate.isNoId())
      return false;

    // No need to try if already active, or if cannot be merged
    if(!token->isInactive() || !token->getState()->lastDomain().isMember(Token::MERGED))
      return false;


    token->merge(merge_candidate);

    TREX_INFO("trex:debug:synchronization:mergeToken", 
	     m_core->nameString() << "Merging " << token->toString() << " onto " << merge_candidate->toString());

    m_core->propagate();

    return true;
  }

  /**
   * Tail recursive to follow the slaves
   */
  bool Synchronizer::insertToken(const TokenId& token, unsigned int& stepCount){
    if(m_core->isInvalid() || m_core->inDeliberation(token))
      return false;

    // If the token is in the past, we will not insert it. Just remove it from the agenda and return OK
    if(token->end()->lastDomain().getUpperBound() == Agent::instance()->getCurrentTick()){
      m_core->m_tokenAgenda.erase(token);
      TREX_INFO("trex:debug:synchronization:insertToken", m_core->nameString() << "Skipping insertion of " << token->toString() << " which is in the past.");
      return true;
    }

    if(token->isInactive())
      token->activate();

    condDebugMsg(!token->getObject()->lastDomain().isSingleton(), "trex:error",  
		 "Expecting " << token->toLongString() << " to be bound to a timeline for synchronization");

    checkError(token->getObject()->lastDomain().isSingleton(), 
	       "Expecting " << token->toLongString() << " to be bound to a timeline for synchronization");

    ObjectId object = token->getObject()->lastDomain().getSingletonValue();

    // If not a timeline - no ordering requirement
    if(!TimelineId::convertable(object))
      return m_core->propagate();

    std::vector<OrderingChoice> results;
    m_db->getOrderingChoices(token, results, 1);

    if (results.empty())
      return false;

    const OrderingChoice& choice = results[0];
    checkError(choice.first == object, choice.first->toString() << " != " << object->toString());

    TokenId p = choice.second.first;
    TokenId s = choice.second.second;

    TREX_INFO("trex:debug:synchronization:insertToken", m_core->nameString() << "Inserting " << token->toString());

    object->constrain(p, s);

    m_core->propagate();

    TREX_INFO_COND(m_core->isInvalid(), "trex:debug:synchronization:insertToken", m_core->nameString() << 
		   "Inconsistent after inserting " << token->toString());

    const TokenSet& slaves = token->slaves();

    for(TokenSet::const_iterator it = slaves.begin(); it != slaves.end(); ++it){

      if(m_core->isInvalid())
	return false;

      TokenId slave = *it;
      TokenId merge_candidate;

      if(!inSynchScope(slave, merge_candidate))
	continue;

      stepCount++;

      if(!resolveToken(slave, stepCount, merge_candidate))
	return false;
    }

    return true;
  }

  /**
   * @brief Iterates over all internal timelines. If it finds a gap at the current execution frontier, it will allocate
   * and insert an 'undefined' token which starts at this tick, and whose end is open.
   */
  bool Synchronizer::completeInternalTimelines(unsigned int& stepCount){
    if(m_core->isInvalid())
      return false;

    TREX_INFO("trex:debug:synchronization:completeInternalTimelines", m_core->nameString() << "START");

    unsigned int max_i = m_core->m_internalTimelineTable.size();
    TICK tick = m_core->getCurrentTick();

    for(unsigned int i = 0; i < max_i; i++){
      TimelineId timeline =  m_core->m_internalTimelineTable[i].first;
      const std::list<TokenId>& tokens = timeline->getTokenSequence();

      // Advance the iterator to the execution frontier
      std::list<TokenId>::const_iterator it = tokens.begin();
      TokenId token;
      while(it != tokens.end()){
	TokenId candidate = *it;
	TREX_INFO("trex:debug:synchronization:completeInternalTimelines", m_core->nameString() << "Evaluating " << candidate->toString() << " for processing at the execution frontier");

	// Terminate if we have moved past tao
	if(candidate->start()->lastDomain().getLowerBound() > tick)
	  break;

	// Terminate, selecting the candidate, if it contains tao
	if(candidate->start()->lastDomain().getLowerBound() <= tick && 
	   candidate->end()->lastDomain().getUpperBound() > tick){
	  TREX_INFO("trex:debug:synchronization:completeInternalTimelines", m_core->nameString() << candidate->toString() << " has been selected for processing at the execution frontier");
	  token = candidate;
	  break;
	}

	++it;
      }

      // If no token, then we must insert
      if(token.isNoId()){
	if(!insertDefaultValue(timeline, stepCount))
	  return false;

	continue;
      }
 
      // If the end time exceeds the current tick, the token can be extended
      if(token->start()->lastDomain().getUpperBound() < tick && token->end()->lastDomain().getUpperBound() > tick)
	m_core->extendCurrentValue(token);
      else // We should start the next token
	token->start()->specify(tick);

      // Propagate since we have restricted the database and this can have knock-on effects
      if(!m_core->propagate())
	return false;
    }

    TREX_INFO("trex:debug:synchronization:completeInternalTimelines", m_core->nameString() << "END");

    return true;
  }

  /**
   * @brief Insert a new undefined token relative to the given token
   */
  bool Synchronizer::insertDefaultValue(const TimelineId& timeline, unsigned int& stepCount){
    static const int sl_defaultPredicateIndex(1); // Mode is first.

    // Allocate a token - it should be inactive but not rejectable. We are definitievly setting the state
    // to undefined, though we may merge in doing so.
    std::string predicate = timeline->getType().toString();
    ConstrainedVariableId defaultPredicateName = timeline->getVariables()[sl_defaultPredicateIndex];
    checkError(defaultPredicateName.isValid(), "Invalid NDDL class for " << timeline->toString());
    checkError(defaultPredicateName->lastDomain().isSingleton(), defaultPredicateName->toString());

    LabelStr predLabel = (LabelStr) defaultPredicateName->lastDomain().getSingletonValue();
    predicate += ".";
    predicate += predLabel.toString();

    TREX_INFO("trex:debug:synchronization:insertDefaultValue", m_core->nameString() << "Insert " << predicate << " On " << timeline->toString());
 
    TokenId token = m_db->getClient()->createToken(predicate.c_str(), DbCore::NOT_REJECTABLE);
    token->activate();
    token->start()->restrictBaseDomain(IntervalIntDomain(m_core->getCurrentTick(), m_core->getCurrentTick()));
    token->end()->restrictBaseDomain(IntervalIntDomain(m_core->getCurrentTick() + 1, PLUS_INFINITY));
    token->getObject()->specify(timeline);

    return insertToken(token, stepCount);
  }

  /**
   * @see TREX.nddl for definition of parameters of AgentTimeline
   */
  bool Synchronizer::isPersistent(const TokenId& token){
    if(token->getObject()->lastDomain().isSingleton()){
      ObjectId object = token->getObject()->lastDomain().getSingletonValue();
      if(token->getPlanDatabase()->getSchema()->isA(object->getType(), Agent::TIMELINE())){
	ConstrainedVariableId mode = object->getVariables()[2];
	if(mode->lastDomain().isMember(true))
	  return true;
      }
    }

    return false;
  }

  std::string Synchronizer::propagationFailure() const {
    std::stringstream ss;

    ConstraintEngineId ce = m_db->getConstraintEngine();

    if(!ce->constraintConsistent()){
      ss <<  "Constraint Network is inconsistent. " << std::endl;

      const ConstrainedVariableSet& variables = ce->getVariables();
      for(ConstrainedVariableSet::const_iterator v_it = variables.begin(); v_it != variables.end(); ++v_it){
	ConstrainedVariableId var = *v_it;

	if(var->lastDomain().isEmpty() && var->lastDomain().isClosed()){
	  ss << localContextForConstrainedVariable(var);
	}
      }
    }
    return ss.str();
  }

  std::string Synchronizer::tokenResolutionFailure(const TokenId& tokenToResolve, const TokenId& merge_candidate) const {
    std::stringstream ss;

    ss << "Failed to resolve " << tokenToResolve->toString() << std::endl << 
      tokenToResolve->toLongString() << std::endl << std::endl << "Analysis results below" << std::endl << std::endl;

    if(m_core->isObservation(tokenToResolve)){
      ss << tokenToResolve->toString() << " is an observation." << std::endl;
    }

    if(tokenToResolve->master().isId()){
      const TokenId master = tokenToResolve->master();
      ss << tokenToResolve->toString() << " is implied by the model and a slave of " << master->toString() << std::endl << std::endl;
    }

    // If the network is inconsistent then we want to analyze the neighborhood of the empty variable
    if(!m_db->getConstraintEngine()->constraintConsistent()){
      ss << propagationFailure();   
    }
    else if(merge_candidate.isId()){
      ss << merge_candidate->toString() << " is not compatible in database below:" << std::endl;
      ss << PlanDatabaseWriter::toString(m_db) << std::endl;
    }
    else {
      ss << "No compatible tokens and no locations for insertion." << std::endl;

      // Output an analysis of the blocking token, if there is one.
      ss << analysisOfBlockingToken(tokenToResolve);

      ss << std::endl << "Current Partial Plan:" << std::endl;
      ss << PlanDatabaseWriter::toString(m_db) << std::endl;
    }

    return ss.str();
  }

  std::string Synchronizer::localContextForConstrainedVariable(const ConstrainedVariableId& var) const {
    std::stringstream ss;

    ss << std::endl << "Local context for variable " << var->getName().toString() << "(" << var->getKey() << "):" << std::endl <<
      "Base Domain is:" << var->baseDomain().toString() << std::endl <<
      "Derived domain is:" << var->lastDomain().toString() << std::endl << std::endl;

    TokenId token = getParentToken(var);
    if(token.isId())
      ss << "Variable belongs to :" << PlanDatabaseWriter::simpleTokenSummary(token);
    else
      ss << "Variable does not belong to any token.";

    ss << std::endl << std::endl;

    ConstraintSet constraints;
    ConstrainedVariableSet variables;
    TokenSet tokens;
    var->constraints(constraints);
    for(ConstraintSet::const_iterator c_it = constraints.begin(); c_it != constraints.end(); ++c_it){
      ConstraintId constraint = *c_it;
      const std::vector<ConstrainedVariableId>& scope = constraint->getScope();
      for(unsigned int i=0; i<scope.size(); i++){
	variables.insert(scope[i]);
	TokenId parent = getParentToken(scope[i]);
	if(parent.isId())
	  tokens.insert(parent);
      }
    }

    ss << "Related Tokens: " << std::endl;

    for(TokenSet::const_iterator t_it = tokens.begin(); t_it != tokens.end(); ++t_it){
      TokenId token = *t_it;
      ss << "  " << PlanDatabaseWriter::simpleTokenSummary(token) << std::endl;
    }

    ss << std::endl << "Related Variables:" << std::endl;
    for(ConstrainedVariableSet::const_iterator it = variables.begin(); it != variables.end(); ++it){
      ConstrainedVariableId v = *it;
      ss << "  " << v->getName().toString() << "(" << v->getKey() << ")"; 
      TokenId parent = getParentToken(v);
      if(parent.isId())
	ss << " of " << PlanDatabaseWriter::simpleTokenSummary(parent);
      ss << std::endl;
    }

    ss << std::endl << "Related Constraints: " << std::endl;

    for(ConstraintSet::const_iterator c_it = constraints.begin(); c_it != constraints.end(); ++c_it){
      ConstraintId constraint = *c_it;
      ss << constraint->toString() << std::endl << std::endl ;
    }

    return ss.str();
  }

  std::string Synchronizer::tokenExtensionFailure(const TokenId& expectedToken) const{
    std::stringstream ss;

    ss << "Expected to have a value given by:" << expectedToken->toString() << 
      " which starts at " << expectedToken->start()->lastDomain().toString() << " and ends at " << expectedToken->end()->lastDomain().toString() << std::endl;

    // The tmepoint of interest is the start or end at the current tick
    ConstrainedVariableId timepointOfInterest;

    if(expectedToken->start()->lastDomain().isSingleton() && expectedToken->start()->lastDomain().getSingletonValue() == m_core->getCurrentTick())
      timepointOfInterest = expectedToken->start();
    else
      timepointOfInterest = expectedToken->end();

    // Output the local context for this timepoint"
    ss << localContextForConstrainedVariable(timepointOfInterest);

    return ss.str();
  }

  /**
   * Search for the location in the database where this token would like to be and see which
   * tokens are present and why there is no merge possible.
   */
  std::string Synchronizer::analysisOfBlockingToken(const TokenId& tokenToResolve) const {
    std::stringstream ss;

    const TokenSet& activeTokens = m_db->getActiveTokens(tokenToResolve->getPredicateName());
    const AbstractDomain& startDom = tokenToResolve->start()->lastDomain();
    const AbstractDomain& endDom = tokenToResolve->end()->lastDomain();

    for(TokenSet::const_iterator it = activeTokens.begin(); it != activeTokens.end(); ++it){
      TokenId token = *it;

      // Skip if it is this token
      if(token == tokenToResolve)
	continue;

      if(!token->getObject()->lastDomain().intersects(tokenToResolve->getObject()->lastDomain()))
	continue;

      // Report the token if it overalps in time but does not allow merging
      if(token->start()->lastDomain().intersects(startDom) && token->end()->lastDomain().intersects(endDom)){
	ss << "Found a conflict with " << token->toLongString() << std::endl << std::endl;
	std::vector<ConstrainedVariableId> p_a = token->parameters();
	std::vector<ConstrainedVariableId> p_b = tokenToResolve->parameters();

	for(unsigned int i = 0; i < p_a.size(); i++){
	  ConstrainedVariableId v_a = p_a[i];
	  ConstrainedVariableId v_b = p_b[i];
	  if(!v_a->lastDomain().intersects(v_b->lastDomain()))
	    ss << "    " << v_a->toLongString() << " conflicts with " << v_b->toLongString() << std::endl;
	}

	ss << std::endl;

	break;
      }
    }

    return ss.str();
  }

  /**
   * @brief Accessor to debug stream for debug output
   */
  std::ostream& Synchronizer::getStream(){ return m_core->getStream(); }
}
