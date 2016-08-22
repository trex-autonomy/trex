#ifndef H_Agent
#define H_Agent

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
 * @file Provides declaration for the Agent class. This is basically a composition of TeleoReactors. The Agent will
 * allocate reactors and connect them together according to their configuratiun requirements. It will also play the role
 * of middle-man to route observations from sender to receiver.
 * @status Documented
 * @note The Agent class is not thread safe.
 */

#include "TREXDefs.hh"
#include "XMLUtils.hh"
#include "TeleoReactor.hh"
#include "AgentClock.hh"
#include "ObservationLogger.hh"
#include "PerformanceMonitor.hh"
#include "RStat.hh"
#include <vector>
#include <map>

namespace TREX {

  /**
   * @brief The Agent is an observer of messages from TeleoReactors. It is the message bus for distribution of observations
   * @see TeleoReactor
   * @see Clock
   */
  class Agent {
  public:

    /**
     * @brief There are three different types of event routed through the agent for sharing information between reactors.
     */
    enum EventType {
      Notify = 0, /*!< For observations posted from a reactor */
      Request, /*!< For requests made on a reactor */
      Recall /*!< For recalls made on a reactor */
    };

    /**
     * @brief An Agent Event is used for logging. This is particularly useful when applying an event log for regression testing to validate
     * current execution against prior stored values.
     */
    class Event {
    public:
      Event(TICK tick, EventType evType, const LabelStr& objectName, const LabelStr& predicateName);
      Event(const Event& org);
      TICK m_tick; /*!< The tick at which the event occurred */
      EventType m_eventType; /*!< The type of event */
      LabelStr m_objectName; /*!< What object the event related to. The object should be a timeline */
      LabelStr m_predicateName; /*!< The predicate name. This is only partial information for the information content in the event, 
				  but it is enough to do alot of good validation against without incurring excessive overhead */
    };

    /**
     * @brief The Agent is a singleton per process.
     * @param configData The Agent XML Configuration file.
     * @param clock The clock to be used by the agent. Different clocks are used to provide different run-time behavior.
     * @param timeLimit The maximum tick to run for. Agetr this time, the agent will terminate. The timeLimit defined here will over-ride
     * any value provided in the configuration file.
     * @param enableEventLog Set this true if event logging should be enabled. If true, it will store events for all observations, requests and recalls
     * in memory.
     * @see Agent::Agent
     */
    static AgentId initialize(const TiXmlElement& configData, Clock& clock, TICK timeLimit = 0, bool enableEventLog = false);

    /**
     * @brief Accessor for the singleton instance
     */
    static const AgentId& instance();

    /**
     * @brief Allow a reset of the current instance - deallocate it.
     */
    static void reset();

    /**
     * Returns the TICK value corresponding to 'forever'. This number is as large as possible, but must be distinguishable from the EUROPA
     * constant indicating infinity. We ensure forever < infinity. This may be wierd, but there you have it!
     */
    static TICK forever();

    /**
     * @brief Terminate the agent
     */
    static void terminate();

    /**
     * @brief Access termination flag
     */
    static bool terminated();

    /**
     * @brief Run the agent. Will block till it is done with its mission. Run it on a separate thread if you need to do more than just run the agent.
     * @see doNext
     */
    void run();

    /**
     * @brief Run the agent for a next cycle of execution. Allows for externally timing control in testing
     * @return true if not yet complete, otherwise false.
     */
    bool doNext();

    /**
     * @brief Accessor
     */
    const AgentId& getId() const;

    /**
     * @brief Accessor
     */
    const LabelStr& getName() const;

    /**
     * @brief Retrieve Reactor by name
     */
    const TeleoReactorId& getReactor(const LabelStr& name);

    /**
     * @brief Retrieve Onwer reactor for timeline
     */
    const TeleoReactorId& getOwner(const LabelStr& timeline);

    /**
     * @brief Get the reactor count
     */
    int getReactorCount() const {return m_reactors.size();}

    /**
     * @brief Accessor for debug stream
     */
    std::ostream& getStream();

    /**
     * @brief Called by Reactors when the post observations. The agent will route to 0 or more reactors who track this observation timeline.
     * @param observation The observation reported.
     * @see Observer
     */
    void notify(const Observation& observation);

    /**
     * @brief Call back to log a request being sent to a reactor
     */
    void logRequest(const TokenId& goal);

    /**
     * @brief Call back to log a recall being sent to a reactor
     */
    void logRecall(const TokenId& goal);

    /**
     * @brief When a goal has been rejected
     */
    void notifyRejected(const TokenId token);

    /**
     * @brief When a goal has been completed
     */
    void notifyCompleted(const TokenId& token);

    /**
     * @brief Test if the mission is over. Exposed publically to facilitate testing
     */
    bool missionCompleted() const;

    /**
     * @brief Get the current clock value
     */
    TICK getCurrentTick() const;

    /**
     * @brief Retrieve the final tick for the agent. Used to limit the planning horizon
     */
    TICK getFinalTick() const;

    /**
     * @brief Accessor for the clock
     */
    const Clock& getClock() const;

    /**
     * Over-write default, allowing different statistics collector
     */
    void setMonitor(PerformanceMonitor& monitor);

    /**
     * @brief Accessor for a performance monitor
     */
    const PerformanceMonitor& getMonitor() const;

    /**
     * @brief Accessor for the event log. This event log is mostly used in the regression testing suite.
     */
    const std::vector<Event>& getEventLog() const;

    /**
     * @brief Register a listener for token rejection or completion messages
     */
    void registerListener(const AgentListenerId& listener);

    /**
     * @brief Unregister a listener.
     */
    void unregisterListener(const AgentListenerId& listener);

    /**
     * @brief Retrieve the latest log directory for this agent
     */
    const std::string& getLogDir() const;

    /**
     * @brief Utility to stringify an event log.
     * @param eventLog the vector of events of interest
     * @param use_tick true if you want tick values to be output. False if we just want the ordering.
     */
    static std::string toString(const  std::vector<Event>& eventLog, bool use_tick = true);

    /**
     * @brief Constant for Timeline base class
     */
    static const LabelStr& TIMELINE();

    /**
     * @brief Constant for the Action base class
     */
    static const LabelStr& ACTION();

    /**
     * @brief Accessor for Mode constant
     */
    static const LabelStr& TIMELINE_MODE();

    /**
     * @brief Accessor for Internal Timeline
     */
    static const LabelStr& INTERNAL_TIMELINE();

    /**
     * @brief Accessor for External Timeline
     */
    static const LabelStr& EXTERNAL_TIMELINE();

    /**
     * @brief Accessor for Ignore Timeline
     */
    static const LabelStr& IGNORE_TIMELINE();


    void incrementAttempts();

    unsigned int getCurrentAttempt() const;

    /**
     * Method to cause an Agent to write all it's reactors assemlies to disk.
     */
    std::string dumpState(const bool export_assembly);

  private:

    static LabelStr buildLogName(LabelStr const &prefix);

    /**
     * @brief Destructor
     */
    virtual ~Agent();

    /**
     * @brief Instantiated by singleton initialization function
     */
    Agent(const TiXmlElement& configData, Clock& clock, TICK timelimit, bool enableLogging = true);

    /**
     * @brief execute the next reactor for a step.
     * @return true if more work to do
     */
    bool executeReactor();

    /**
     * @brief This is called just before we move to the next tick.
     */
    void handleTickEnd();

    /**
     * @brief Called at the start of a new tick. Dispatches commands
     */
    void handleTickStart();

    /**
     * @brief Called to synchronize values at the execution frontier across all reactors.
     */
    void synchronize();

    /**
     * @brief Select the next reactor to work on
     * @return reactor The next reactor to work on. If no work required, returns a noId()
     */
    TeleoReactorId nextReactor();

    /**
     * Helper method to obtain the correct final tick value from the input parameter string
     */
    static TICK getFinalTick(const char * valueStr);

    static AgentId s_id; /*!< Store for the singleton instance */
    AgentId m_id; /*!< This Id */
    const LabelStr m_name; /*! Name - from configuration file. */
    ObserverId m_thisObserver; /*!< A connector to allow the agent to play as a middleman by intercepting observations from Reactors */
    unsigned int m_currentTick; /*!< Set by the clock */
    unsigned int m_finalTick; /*!< Determines mission end */
    unsigned int m_attempts; /*!< Tracks the number of times this tick has been attempted to be resolved */
    std::multimap<LabelStr, ObserverId> m_observersByTimeline; /*!< Routing table for observations */
    std::vector<TeleoReactorId> m_reactors; /*!< The reactors in order of allocation */
    std::map< double, TeleoReactorId> m_reactorsByName; /*!< The set of reactors */
    std::map< double, TeleoReactorId> m_ownersByTimeline; /*!< Lookup table for getting owners */
    std::vector<TeleoReactorId> m_sortedReactors; /*!< Sorted by dependency for synchronization */
    std::vector<TeleoReactorId> m_deliberators; /*!< The agenda of reactors for deliberation. Refreshed on every tick. */
    std::list<AgentListenerId> m_listeners; /*!< For monitoring events by external listeners */

    Clock& m_clock; /*!< The clock used to drive agent ticks. */

    /* Support for performance tracking */
    PerformanceMonitor m_monitor;
    RStat m_synchUsage;
    RStat m_deliberationUsage;

    /* Logging support */
    const bool m_enableEventLogger; /*!< If true, the agent will store events */
    std::vector<Event> m_eventLog; /*!< Used for analysis and testing */
    ObservationLogger m_obsLog;
    std::ostream& m_standardDebugStream; /*!<Stores debug stream to allow it to be reset on destruction */

    static bool s_terminated; /*!< Marker for termination */    
  };

}

#endif
