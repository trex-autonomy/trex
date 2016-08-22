#ifndef H_Synchronizer
#define H_Synchronizer

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

#include "TREXDefs.hh"
#include "PlanDatabaseDefs.hh"
#include "RuleInstance.hh"

namespace TREX {

  /**
   * @brief Implements the synhronization algorithm and gathers all the components required for it
   */
  class Synchronizer {
  public:
    Synchronizer(const DbCoreId& _core);

    /**
     * @brief Resolve observations, goals and plan at the execution frontier.
     */
    bool resolve();

    /**
     * @brief Relax the database. Must retain commited values at the execution frontier, and keep current observations,
     * but want to relax goals and their implications.
     * @param discardCurrentValues If true, then current tokens, that belong to internal, non-persistent timelines, will be discarded.
     * @see resetRemainingTokens
     * @return true if successful, otherwise false
     */
    bool relax(bool discardCurrentValues);


    /** UTILITIES FOR ANALYSIS OF FAILURES **/
    std::string tokenResolutionFailure(const TokenId& tokenToResolve, const TokenId& merge_candidate) const;
    std::string propagationFailure() const;
    std::string localContextForConstrainedVariable(const ConstrainedVariableId& var) const;
    std::string tokenExtensionFailure(const TokenId& expectedToken) const;
    std::string analysisOfBlockingToken(const TokenId& tokenToResolve) const;
  private:

    /**
     * @brief Resolve all tokens at the execution frontier.
     */
    bool resolveTokens(unsigned int& stepCount);

    /**
     * @brief Resolve a specific token at the execution frontier
     */
    bool resolveToken(const TokenId& token, unsigned int& stepCount, const TokenId& mergeCandidate);

    /**
     * @brief Merge a token if the option is there
     * @return true if it tried a merge, false if it did not
     */
    bool mergeToken(const TokenId& token, const TokenId& mergeCandidate);

    /**
     * @brief Insert a token and resolve its slaves
     */
    bool insertToken(const TokenId& token, unsigned int& stepCount);

    /**
     * @brief Batch insert of copied values for repair algorithm
     */
    bool insertCopiedValues();

    /**
     * @brief Utiltiy to place default tokens in empty slots.
     */
    bool completeInternalTimelines(unsigned int& stepCount);

    /**
     * @brief Utility to create and insert and undefined token in the given timeline and location
     */
    bool insertDefaultValue(const TimelineId& timeline, unsigned int& stepCount);

    /**
     * @brief Test for horizon part of synchronization scope.
     */  
    bool inTickHorizon(const TokenId& token, int currentTick);

    /**
     * @brief Test for a unit decision in the synchronization scope
     */
    bool isUnit(const TokenId& token, TokenId& mergeCandidate);

    /**
     * @brief Test for synchronization scope
     */
    bool inSynchScope(const TokenId& token, TokenId& mergeCandidate);

    /**
     * @brief Reset goals as part of repair
     */
    void resetGoals(bool discardOpenGoals);

    /**
     * @brief Reset the buffered observations as part of repair
     */
    void resetObservations();

    /**
     * @brief Reset remaining tokens
     * @param discardCurrentValues If true, then current tokens, that belong to internal, non-persistent timelines, will be discarded.
     */
    void resetRemainingTokens(bool discardCurrentValues);

    /**
     * @brief test if the current value
     */
    bool isCurrent(const TokenId& token);

    /**
     * @brief Helper method to test if the token is on a persistent object
     */
    static bool isPersistent(const TokenId& token);

    /**
     * @brief Copy a value at the execution frontier. Used to re-apply the model.
     */
    void copyValue(const TokenId& source);

    /**
     * @brief Accessor to debug stream for debug output
     */
    std::ostream& getStream();

    /**
     * Utility to extract the underlying variable if this one is part of a merged token
     */
    static ConstrainedVariableId getActiveGuard(const ConstrainedVariableId& var);

    unsigned int m_stepCount;

    DbCoreId m_core;
    PlanDatabaseId m_db;
    const std::vector<TimelineId>& m_timelines;
    TokenSet& m_goals; /*!< Store all goals */
    TokenSet& m_observations; /*!< Store received observations received */
    TokenSet& m_tokenAgenda; /*!< Buffer of tokens available for synchronization */
    TokenSet& m_committedTokens; /*!< Buffer of committed tokens */
  };

}
#endif
