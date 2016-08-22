#ifndef H_DbCore
#define H_DbCore
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
 * @file Provides declarations for the DbCore class - database related instance data.
 */
#include "TeleoReactor.hh"
#include "Solver.hh"
#include "Assembly.hh"
#include "FlawFilter.hh"
#include "Synchronizer.hh"
#include "LogManager.hh"
#include "DbSolver.hh"

using namespace EUROPA;
using namespace EUROPA::SOLVERS;

namespace TREX {

  /**
   * @brief Specialized filter to enforce deliberation horizon and other standardized policies that arise for the
   * planner.
   */
  class DeliberationFilter: public FlawFilter {
  public:
    DeliberationFilter(const TiXmlElement& configData);

    bool test(const EntityId& entity);

    std::ostream& getStream();

    static void update(TICK start, TICK end);

    static const IntervalIntDomain& getHorizon();

  private:
    DbCoreId m_core;
    static IntervalIntDomain& horizon();
    static TICK currentTick();
  };

  /**
   * @brief Stores buffered observations for a given timeline
   * Observations are received by the reactor and they are immediately turned into inactive tokens. These tokens
   * are buffered so that as we respond to tick events we can quickly determine if a new observation has been received. 
   * If not, we must extend the current value at the execution frontier in line with the Inertial Value Assumption
   *
   * @see handleTickCleanup, extendCurrentValue, archive
   */
  class TimelineContainer {
  public:
    TimelineContainer(const TimelineId& timeline);

    const TimelineId& getTimeline() const;

    /**
     * @brief Used to log an observation.
     * @param tick The tick when the obsevation occurred
     * @note updates lastObserved()
     */
    void updateLastObserved(TICK tick);

    /**
     * @brief Returns the tick at which the latest observation was posted.
     */
    TICK lastObserved() const;

    /**
     * @brief Accessor for the server owning this timeline
     */
    const ServerId& getServer() const;

    /**
     * @brief Assign the server. Not available at construction
     */
    void setServer(const ServerId& server);

    /**
     * @brief Test if the token has already been dispatched
     */
    bool isDispatched(const TokenId& token);

    /**
     * @brief Get the set of dispatched tokens still in memory
     */
    const TokenSet& getDispatchedTokens() const;

    /**
     * @brief Record that it has been dispatched
     */
    void markDispatched(const TokenId& token);

    /**
     * @brief Reset the dispatch flag.
     */
    void clearDispatched(const TokenId& token);

    /**
     * @brief Handle removal of a token. Unbuffer
     */
    void handleRemoval(const TokenId& token);

  private:
    const TimelineId m_timeline; /*!< Id for the timeline we buffer observations for */
    ServerId m_server;
    TICK m_lastObserved; /*!< Used to say how current the latest observation is. */
    TokenSet m_dispatchedTokens; /*!< The set of buffered dispatches. Used to dispatch once only. */
  };

  /**
   * @brief The major component for deliberation over large and small time scales.
   */
  class DbCore: public TeleoReactor {
  public:

    /**
     * @brief Get the global clock variable for the database the given token is in.
     */
    static ConstrainedVariableId getAgentClockVariable(const PlanDatabaseId db);

    class DbListener: public PlanDatabaseListener {
    public:
      DbListener(DbCore& dbCore);

      /**
       * @brief Handle creation - scope specification
       */
      void notifyAdded(const TokenId& token);

      /**
       * @brief Handle merge
       */
      void notifyMerged(const TokenId& token);

      /**
       * @brief Handle activation
       */
      void notifyActivated(const TokenId& token);

      /**
       * @brief handle deletion - scope buffer removal
       */
      void notifyRemoved(const TokenId& token);

      /**
       * @brief Handle notification of commitment
       */
      void notifyCommitted(const TokenId& token);

      /**
       * @brief Handle notification of cancel
       */
      void notifyDeactivated(const TokenId& token);

      /**
       * @brief Handle notification of cancel
       */
      void notifySplit(const TokenId& token);

      /**
       * @brief For logging purposes, notify when rejecting a token
       */
      void notifyRejected(const TokenId& token);

      /**
       * @brief Handle notification of termination
       */
      void notifyTerminated(const TokenId& token);

    private:

      DbCore& m_dbCore;
    };

    class PlanDescription {
    public:
      friend class DbCore;

      struct TokenDescription {
	int key;
	LabelStr name;
	double start[2], end[2];
      };

      struct TimelineDescription {
	LabelStr name;
	std::vector<TokenDescription> tokens;
      };

      void clear() {
	m_internalTimelines.clear();
	//m_actions.clear();
	m_externalTimelines.clear();
      }

      TICK m_tick;
      LabelStr m_reactorName;
      std::vector<TimelineDescription> m_internalTimelines, m_actions, m_externalTimelines;
    };

    friend class DbListener;
    friend class Synchronizer;
    friend class DeliberationFilter;

    DbCore(const LabelStr& agentName, const TiXmlElement& configData);

    ~DbCore();

    /**
     * @brief Accessor to retrieve the dbcore for the given token.
     */
    static DbCoreId getInstance(const TokenId& token);

    /**
     * @brief Test if the given toke is on a timeline that is in scope
     */
    bool inScope(const TokenId& token) const;

    /**
     * @brief Tests if the given token is in active deliberation scope
     */
    bool inDeliberation(const TokenId& token) const;

    /**
     * @brief Test if the given token is a current observation
     */
    bool isCurrentObservation(const TokenId& token);

    void notify(const Observation& observations);

    bool handleRequest(const TokenId& goal);

    void handleRecall(const TokenId& goal);

    void queryTimelineModes(std::list<LabelStr>& externals, std::list<LabelStr>& internals);

    /**
     * @brief Get a description of the plan at the current tick
     */
    void getPlanDescription(PlanDescription &planDesc) const;

    /**
     * @brief Write a description of a conflict to disk.
     */
    std::string writeConflict(std::string brief_description, std::string analysis);

    /**
     * @brief Output the assembly to file at the current tick
     */
    std::string dumpState(bool export_assembly = false);

    /**
     * @brief Add a PlanDatabaseListener to the internal EUROPA PlanDatabase
     */
    void addDbListener(EUROPA::PlanDatabaseListener& listener);

    /**
     * @brief Get the value for the given tick
     * @return if there is a value then return it. If not, return a noId
     */
    TokenId getValue(const TimelineId& timeline, TICK tick);

  protected:
    /**
     * @brief Used to hook up observer for dispatch of observations and servers for dispatch of goals
     */
    virtual void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer);

    /**
     * @brief Must address new tick and conduct appropriate propagation and dispatch
     */
    virtual void handleTickStart();

    /**
     * @brief Handle synchronization. If it fails it is a system error.
     */
    virtual bool synchronize();

    /**
     * @brief Return true if there are flaws to resolve
     */
    virtual bool hasWork();

    /**
     * @brief Step the reactor to resolve flaws
     */
    virtual void resume();

    enum State {
      INACTIVE = 0,
      ACTIVE,
      INVALID
    };


    virtual bool isInvalid() const {return m_state == INVALID;}

    void resetState() { m_state = INACTIVE;}

    /**
     * @brief Encapsulation of change to invalidate db state
     * @param comment Provide context and hints for debugging
     */
    void markInvalid(
	const std::string& comment,
	const bool dump_state=false,
	const std::string& analysis = std::string(""));

    /**
     * @brief Accessor to goal set
     */
    TokenSet& getGoals(){return m_goals;}

    /**
     * @brief Access the whole assembly
     */
    Assembly& getAssembly(){return m_assembly;}

  private:
    /**
     * @brief Apply inertial value assumption to external timelines
     * @see synchronize
     */
    bool completeExternalTimelines();

    /**
     * @brief Archive the database.
     */
    void archive();

    /**
     * @brief Dispatch observations to other components
     */
    void notifyObservers();

    /**
     * @brief Dispatch goals to server. Invoked on a tick
     */
    void dispatchCommands();

    /**
     * @brief Recall dispatched commands. Invoked when the plan fails.
     */
    void dispatchRecalls();

    /**
     * @brief Extends the current value of the timeline - Inertial Value Assumption
     */
    bool extendCurrentValue(const TimelineId& timeline);

    /**
     * @brief Extend current value of the token
     */
    bool extendCurrentValue(const TokenId& token);

    /**
     * @brief Helper method to handle constraint migration when receiving a goal
     * @see handleRequest
     */
    void applyConstraints(const TokenId& goal);

    /**
     * @brief Helper method to propagate the database.
     * @return true if constraint consistent, else false
     */
    bool propagate();

    /**
     * @brief Helper method to restrict base domains of a restricted token
     */
    bool restrict(const TokenId& token);

    /**
     * @brief Helper method to cleanup a token as part of termination
     */
    void terminate(const TokenId& token);

    /**
     * @brief Updated merged tokens and slaves for a committed token.
     * @return true if successful (i.e. consistent)
     */ 
    bool updateRelatedTokens(const TokenId& token);

    /**
     * @brief Add to agenda
     */
    void addToTokenAgenda(const TokenId& token);

    /**
     * @brief Remove from agenda
     */
    void removeFromTokenAgenda(const TokenId& token);

    /** Handle foreign/local keys. Can be static as there should only be one server per timeline **/
    static bool hasEntity(const EntityId& foreign);

    static void addEntity(const EntityId& foreign, const EntityId& local);

    static void removeEntity(const EntityId& foreign);

    static EntityId getLocalEntity(const EntityId& foreign);

    static EntityId getForeignEntity(const EntityId& local);

    /**
     * @brief To prevent memory growth due to lost entries we provide a way to purge
     * entries whose keys no longer map to entities.
     */
    static void purgeOrphanedKeys();

    static std::map<int, EntityId> s_foreignKeyRelation; /*!< Link by key for copied token when dispatching goals */

    /**
     * @brief Utility to dactive the main solver when done with it.
     */
    bool deactivateSolver();

    /**
     * @brief Utility for construction
     */
    void configure();

    /**
     * @brief Helper method to test if the token is on a timeline of the given type.
     */
    static bool onTimeline(const TokenId& token, const LabelStr& timelineMode);

    /**
     * @brief Update goals on new tick
     * @see handleTickStart
     */
    void updateGoals();

    /**
     * @brief Commit values at the execution frontier
     */
    void commit();

    /**
     * @brief Start actions if possible.
     * @see handleTickStart
     */
    void updateActions();

    /**
     * @brief Utility to set the horizon based on latency, lookAhead, and current tick cycle.
     */
    void setHorizon();

    /**
     * @brief Get current horizon
     */
    void getHorizon(TICK& lb, TICK& ub) const;

    /**
     * @brief Utility to handle domain copy across databases
     */
    void restrict(const ConstrainedVariableId& var, const AbstractDomain& dom);

    /**
     * @brief Utility in lieu of api change in E2
     */
    void getTimelines(ObjectSet& results);

    /**
     * @brief Integrity check utility. Only use in debug mode, and only use for check errors.
     */
    bool isValidDb() const;

    bool isGoal(const TokenId& tok) const;

    bool isObservation(const TokenId& tok) const;

    static bool isAction(const TokenId& tok);

    /**
     * @brief Test if the solver is timed out. If true, will reset automatically
     * @see synchronize
     */
    bool isSolverTimedOut();

    /**
     * @brief Get the number of planning attempts that have been made.
     */
    unsigned int getPlanAttempts();

    /**
     * @brief Fix buffers and key maps
     */
    void cleanupGoal(const TokenId& goal);

    /**
     * DbListener Event Handlers
     */
    void handleAddition(const TokenId& token);
    void handleMerge(const TokenId& token);
    void handleSplit(const TokenId& token);
    void handleActivated(const TokenId& token);
    void handleDeactivated(const TokenId& token);
    void handleRemoval(const TokenId& token);
    void handleCommitted(const TokenId& token);
    void handleRejected(const TokenId& token);
    void handleTerminated(const TokenId& token);

    /**
     * @brief Utility to migrate constraints from one token to another
     */
    void migrateConstraints(const TokenId& source, const TokenId& destination);

    /**
     * @brief Utility to check if a token is supported by an observation directly
     * or indirectly
     */
    bool supportedByObservation(const TokenId& tok);

    /**
     * @brief Utility to check if an action has active actions that must finish before it starts
     */
    bool hasPendingPredecessors(const TokenId& c, const std::vector<TokenId>& uncontrollables, bool requireDifferentObject = false);

    /**
     * @brief Test if the token is internal
     */
    bool isInternal(const TokenId& token);

    /**
     * @brief Test if the token is external
     */
    bool isExternal(const TokenId& token);

    /**
     * @brief Test if we can start an internal token. Based on assessing uncontrollability
     */
    bool canStartInternalToken();

    /**
     * @brief Apply facts. Enforces the semantics of initial conditions for the agent
     */
    void applyFacts(const std::vector<TokenId>& facts);

    /**
     * @brief Process buffered goal keys that have been recalled.
     * @return true if a recall was made
     * @see Synchronize
     */
    bool processRecalls();

    /**
     * @brief Utility to cmmit a token and restrict its base domains based on current time. Will propagate as it goes.
     */
    void commitAndRestrict(const TokenId& token);

    /**
     * @brief Utility to restrict all parameter base domains of a token
     */
    void restrictParameterBaseDomains(const TokenId& token);

    /**
     * @brief Utility to disconnect constraints, binding connected variables prior to termination
     */
    void disconnectConstraints(const TokenId& token);

    /**
     * @brief Utility to wrap components of termination tests
     */
    bool canBeTerminated(const TokenId& token) const;

    void bufferObservation(const TokenId& token);

    void getActiveUncontrollableEvents(std::vector<TokenId>& results);

    /**
     * @brief Utility to evaluate the scope for new tokens and buffer as appropriate
     */
    void processPendingTokens();

    /**
     * @brief Utility to deactivate a token that is to be ignored. We do this on token addition. It serves to
     * enable archiving
     */
    static void deactivateIgnoredToken(const TokenId& token);

    /**
     * @brief Check if a given token is matched by a current observation
     */
    bool observedNow(const TokenId token) const;

    /**
     * @brief Utilities for handling dispatch time settings for dispatched and received tokens
     */
    static ConstrainedVariableId dispatch_time(const TokenId& token);
    void setDispatchTime(const TokenId& token) const;
    void resetDispatchTime(const TokenId& token) const;

    /** UTILITIES FOR ANALYSIS **/
    bool timelinesAreComplete();
    std::string missingObservation(const TimelineId& timeline) const;

    Assembly m_assembly; /*!< Contains the components for the EUROPA Database - can be cut down, customized */

    const PlanDatabaseId m_db; /*!< EUROPA Database contained in the assembly */

    DbListener m_dbListener; /*!< Connects token events to buffers in the core */

    ObserverId m_observer; /*!< This observer is the output for changes on internal timelines */

    Synchronizer m_synchronizer; /*!< Encapsulates synchronization algorithm */

    std::vector<TimelineId> m_timelines; /*!< All timelines */

    std::vector< std::pair<TimelineId, TICK> > m_internalTimelineTable; /*!< Internal Timelines */

    std::map< int, TimelineContainer > m_externalTimelineTable; /*!< Logs arrival of observations per external timeline. 
								  Should be garbage collected when we archive */

    TokenSet m_goals; /*!< Store all goals */

    TokenSet m_observations; /*!< Store received observations received */

    TICK m_currentTickCycle; /*!< Tracks the current tick we are solving for. */

    int m_lastCompleteTick; /*! Tracks the last tick where the planner was complete. Used to limit dispatch of partial results. */

    State m_state; /*!< Tracks the current reactor state. */

    DbSolverId m_solver; /*!< The solver used to plan goals. It is the only one that can look ahead in time */

    const LabelStr m_solverCfg; /*!< Name for solver configuration file */

    std::list<LabelStr> m_internalLabels, m_externalLabels;

    std::set<int> m_notificationKeys; /*!< Buffer for notifications published */

    std::vector<int> m_recallBuffer; /*!< Buffer goal keys for cleanup */

    std::map<int, bool> m_tokenScope; /*!< Ignored token status. See 'inScope' which is used as a workaround for lack of selectivity in rules engine. */

    TokenSet m_pendingTokens; /*!< Buffer to process new tokens after propagation */
    TokenSet m_tokenAgenda; /*!< Buffer of tokens available for synchronization */

    TokenSet m_committedTokens; /*!< Buffer of committed tokens */

    TokenSet m_terminableTokens; /*!< Buffer of committed tokens that are pending termination */
    TokenSet m_terminatedTokens; /*!< Buffer of terminated tokens ready to discard */

    unsigned int m_sync_stepCount; /* Number of steps for synchronisation */

    unsigned int m_search_depth;
    unsigned int m_search_stepCount;

    static const bool REJECTABLE = true;
    static const bool NOT_REJECTABLE = false;

    static std::map<PlanDatabaseId, DbCoreId>& instancesByDb();

    static bool verifyEntities();    

    std::string logPlan(const std::string& msg);
    void logTimeLine(TimelineId const &tl, std::string const &type);
    void logToken(TokenId const &tok);

    void fillTimelineDescription(const TimelineId tl, PlanDescription::TimelineDescription &tlDesc) const;

    /**
     * @brief Write a lightweight timeline description to disk. Used by writeDbstate 
     */
    void writeTimeline(const TimelineId tl, const char mode, std::ofstream &db_out) const;
    
    static std::ostream &writeDomain(std::ostream &out,
				     AbstractDomain const &dom,
				     bool singletonsOnly);

    std::string m_statePath;
    std::string m_conflictPath;
    std::ofstream m_planLog;
    unsigned int m_lastRecalled;
  };
}

#endif
