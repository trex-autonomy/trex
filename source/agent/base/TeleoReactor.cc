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
 * @brief Provides implementation for TeleoReactor, a small abstract base class.
 */

#include "TeleoReactor.hh"
#include "Observer.hh"
#include "Server.hh"
#include "XMLUtils.hh"
#include "Error.hh"
#include "Agent.hh"
#include "StringExtract.hh"
#include "LogManager.hh"
#include "Token.hh"
#include "Utils.hh"
#include "Utilities.hh"

#include <time.h>


namespace TREX {

  /**
   * @brief Connector to channel observations to the reactor
   */
  class TeleoObserver: public Observer {
  public:
    TeleoObserver(TeleoReactorId reactor): Observer(), m_reactor(reactor){}

    virtual void notify(const Observation& observation) {
      m_reactor->notify(observation);
    }

  private:
    TeleoReactorId m_reactor;
  };

  /**
   * @brief Connector to channel goal requests to the reactor
   */
  class TeleoServer: public Server {
  public:
    TeleoServer(TeleoReactorId reactor): Server(), m_reactor(reactor){}

    /**
     * @brief Commands the server to handle a request expressed as a goal network.
     * @param goal The goal token to accomplish.
     */
    bool request(const TokenId& goal) {
      return m_reactor->request(goal);
    }

    /**
     * @brief Commands the server to discard a goal previously requested
     * @param goal The goal token to recalls
     */
    void recall(const TokenId& goal) {
      m_reactor->recall(goal);
    }

    /**
     * @brief Retrieve the latency it takes to respond. This is used in planning when to dispatch goals. It is a lower bound on dispatch window.
     */
    TICK getLatency() const {return m_reactor->getLatency();}

    /**
     * @brief Retrieve the look-ahead window. This is used in deciding how far ahead to commit goals. It is an upper bound on the dispatch window.
     */
    TICK getLookAhead() const {return m_reactor->getLookAhead();}

  private:
    TeleoReactorId m_reactor;
  };

  TICK TeleoReactor::getLookAheadFromXML(const TiXmlElement& configData) {
    if (!configData.Attribute("lookAhead")) {
      debugMsg("TeleoReactor:TeleoReactor", "Lookahead is zero, defaulting to the agent's finalTick.");
      return Agent::instance()->getFinalTick();
    }
    return atoi(extractData(configData, "lookAhead").c_str());
  }

  std::string TeleoReactor::debugFileName(const LabelStr& agentName, const LabelStr& reactorName){
    std::string log_path = LogManager::instance().get_log_path();
    std::string name = LogManager::instance().reactor_file_path(agentName.toString(),reactorName.toString(),"debug.log");
    return name;
  }

  TeleoReactor::TeleoReactor(const LabelStr& agentName, const TiXmlElement& configData, bool logDefault)
    : m_id(this),
      m_name(extractData(configData, "name")),
      m_agentName(agentName),
      m_lookAhead(getLookAheadFromXML(configData)),
      m_latency(atoi(extractData(configData, "latency").c_str())),
      m_thisObserver(new TeleoObserver(m_id)),
      m_thisServer(new TeleoServer(m_id)),
      m_syncUsage(RStat::zeroed), m_searchUsage(RStat::zeroed),
      m_shouldLog(string_cast<bool>(logDefault, checked_string(configData.Attribute("log")))),
      m_debugStream(debugFileName(m_agentName, m_name).c_str()) {
    TREX_INFO("TeleoReactor:TeleoReactor", "Allocating '" << agentName.toString() << "." << m_name.toString());
  }

  TeleoReactor::TeleoReactor(const LabelStr& agentName, const LabelStr& name, TICK lookAhead, TICK latency, bool log)
    : m_id(this),
      m_name(name),
      m_agentName(agentName),
      m_lookAhead(lookAhead),
      m_latency(latency),
      m_thisObserver(new TeleoObserver(m_id)),
      m_thisServer(new TeleoServer(m_id)),
      m_syncUsage(RStat::zeroed), m_searchUsage(RStat::zeroed),
      m_shouldLog(log),
      m_debugStream(debugFileName(m_agentName, m_name).c_str())
 {
    DebugMessage::setStream(getStream());
    TREX_INFO("TeleoReactor:TeleoReactor", "Allocating '" << agentName.toString() << "." << m_name.toString());
  }


  TeleoReactor::TeleoReactor(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead, TICK latency, bool logDefault)
    : m_id(this),
      m_name(extractData(configData, "name")),
      m_agentName(agentName),
      m_lookAhead(lookAhead),
      m_latency(latency),
      m_thisObserver(new TeleoObserver(m_id)),
      m_thisServer(new TeleoServer(m_id)),
      m_syncUsage(RStat::zeroed), m_searchUsage(RStat::zeroed),
      m_shouldLog(string_cast<bool>(logDefault, checked_string(configData.Attribute("log")))), 
      m_debugStream(debugFileName(m_agentName, m_name).c_str()){
    DebugMessage::setStream(getStream());
    TREX_INFO("TeleoReactor:TeleoReactor", "Allocating '" << agentName.toString() << "." << m_name.toString());
  }

  TeleoReactor::~TeleoReactor(){
    DebugMessage::setStream(Agent::instance()->getStream());
    m_thisObserver.release();
    m_thisServer.release();
    m_id.remove();
  }

  const TeleoReactorId& TeleoReactor::getId() const {return m_id;}

  const LabelStr& TeleoReactor::getName() const {return m_name;}

  const LabelStr& TeleoReactor::getAgentName() const {return m_agentName;}

  TICK TeleoReactor::getCurrentTick() const {

    // On initialization, may not have set up the agent yet so have to allow for the initialization phase.
    AgentId agent = Agent::instance();
    if(agent.isNoId())
      return 0;

    return Agent::instance()->getCurrentTick();
  }

  std::string TeleoReactor::nameString() const {
    std::stringstream ss;
    ss << "[" << getName().toString() << "][" << getCurrentTick() << "]";
    return ss.str();
  }

  int TeleoReactor::getPriority(int callCount){
    std::list<LabelStr> externals, internals;
    queryTimelineModes(externals, internals);

    checkError(callCount < Agent::instance()->getReactorCount(), "Cycle detected in reactor specification");

    // If has no externals, then 0
    if(externals.empty())
      return 0;

    int priority = 0;
    for(std::list<LabelStr>::const_iterator it = externals.begin(); it != externals.end(); ++it){
      const LabelStr& timeline = *it;
      TeleoReactorId owner = Agent::instance()->getOwner(timeline);
      checkError(owner.isValid(), "Invalid id for " << timeline.toString());
      priority = std::max(priority, owner->getPriority(callCount + 1));
    }

    return 1 + priority;
  }

  void TeleoReactor::sort(std::vector<TeleoReactorId>& reactors){
    bool swapped = true;
    int count = reactors.size()-1;
    while(swapped){
      swapped = false;
      for(int i=0;i<count;i++){
	TeleoReactorId a = reactors[i];
	TeleoReactorId b = reactors[i+1];
	if(b->getPriority() < a->getPriority()){
	  reactors[i] = b;
	  reactors[i+1] = a;
	  swapped = true;
	}
      }
    }
  }

  bool TeleoReactor::doSynchronize() {
    DebugMessage::setStream(getStream());
    ++m_syncCount;    
    RStatLap chrono(m_syncUsage, RStat::self);
    { // To be "sure" that chrono is created before we call synchronize
      TREX_INFO("trex:debug:timing", "BEFORE synchronization:" << timeString());
      bool result = synchronize();
      TREX_INFO("trex:debug:timing", "AFTER synchronization:" << timeString());
      return result;
    }
  }

  void TeleoReactor::doResume() {
    DebugMessage::setStream(getStream());

    ++m_searchCount;
    RStatLap chrono(m_searchUsage, RStat::self);
    {
      TREX_INFO("trex:debug:timing", "BEFORE resume:" << timeString());
      resume();
      TREX_INFO("trex:debug:timing", "AFTER resume:" << timeString());
    }
  }

  void TeleoReactor::doHandleInit(TICK initialTick, 
				   std::map<double, ServerId> const &serversByTimeline, 
				   ObserverId const &observer) {
    DebugMessage::setStream(getStream());

    m_syncCount = 0;
    m_syncUsage.reset();
    m_searchCount = 0;
    m_searchUsage.reset();
    TickLogger *log = LogManager::instance().getTickLog(CPU_STAT_LOG);

    log->addField(getName().toString()+".sync.nSyncs", m_syncCount);
    log->addField(getName().toString()+".sync.userTime", m_syncUsage.user_time());
    log->addField(getName().toString()+".search.nResume", m_searchCount);
    log->addField(getName().toString()+".search.userTime", m_searchUsage.user_time());

    handleInit(initialTick, serversByTimeline, observer);
  }

  void TeleoReactor::doHandleTickStart() {
    DebugMessage::setStream(getStream());

    m_syncCount = 0;
    m_syncUsage.reset();
    m_searchCount = 0;
    m_searchUsage.reset();
    handleTickStart();
  }

  /**
   * @brief Handle in the derived class if provided
   */
  void TeleoReactor::notify(const Observation& observation){}

  /**
   * @brief Log the request prior to delegation
   */
  bool TeleoReactor::request(const TokenId& goal){
    DebugMessage::setStream(getStream());
    Agent::instance()->logRequest(goal);
    TREXLog() << nameString() << "Request received: " << tokenToString(goal);
    return handleRequest(goal);
  }

  /**
   * @brief Handle in the derived class if provided
   */
  bool TeleoReactor::handleRequest(const TokenId& goal){return true;}

  /**
   * @brief Log the recall prior to delegation
   */
  void TeleoReactor::recall(const TokenId& goal){
    Agent::instance()->logRecall(goal);
    DebugMessage::setStream(getStream());
    TREXLog() << nameString() << "Recall received: " << tokenToString(goal) << std::endl;
    handleRecall(goal);
  }

  /**
   * @brief Handle in the derived class if provided
   */
  void TeleoReactor::handleRecall(const TokenId& goal){}

  TICK TeleoReactor::getLatency() const {
    return m_latency;
  }

  TICK TeleoReactor::getLookAhead() const {
    return m_lookAhead;
  }

  std::ostream& TeleoReactor::getStream(){
    return m_debugStream;
  }

  TeleoReactorId TeleoReactor::createInstance(const LabelStr& agentName, const LabelStr& component, const TiXmlElement& configData){
    Id<TeleoReactor::Factory> factory = getFactory(component);
    checkError(factory != Id<TeleoReactor::Factory>::noId(), component.toString() << " has not been registered.");
    return factory->createInstance(agentName, configData);
  }

  void TeleoReactor::registerFactory(const LabelStr& name, const Id<TeleoReactor::Factory>& factory){
    checkError(TeleoReactor::getFactory(name) == Id<Factory>::noId(), "Already registered " << name.toString());
    lookupTable().insert(std::pair<double, Id<TeleoReactor::Factory> >(name, factory));
  }

  void TeleoReactor::purgeAll(){
    std::map<double, Id<TeleoReactor::Factory> >& m = lookupTable();
    while(!m.empty()){
      Id<TeleoReactor::Factory> f = m.begin()->second;
      m.erase(m.begin());
      checkError(f.isValid(), f);
      delete (TeleoReactor::Factory*) f;
    }
  }

  std::map<double, Id<TeleoReactor::Factory> >& TeleoReactor::lookupTable(){
    static std::map<double, Id<TeleoReactor::Factory> > sl_instance;
    return sl_instance;
  }

  Id<TeleoReactor::Factory> TeleoReactor::getFactory(const LabelStr& name){
    std::map<double, Id<TeleoReactor::Factory> >::const_iterator it = lookupTable().find(name);
    if(it != lookupTable().end())
      return it->second;
    else
      return Id<TeleoReactor::Factory>::noId();
  }

  TeleoReactor::Factory::Factory(const LabelStr& name): m_id(this), m_name(name) { registerFactory(m_name, m_id);}

  TeleoReactor::Factory::~Factory(){
    m_id.remove();
  }

  const Id<TeleoReactor::Factory>& TeleoReactor::Factory::getId() const {return m_id;}
}
