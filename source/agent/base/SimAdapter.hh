/* -*- C++ -*-
 * $Id$
 */
/** @file "SimAdapter.hh"
 * @brief Declaration of SimAdapter
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef _SIMADAPTER_HH
#define _SIMADAPTER_HH


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

#include <set>

#include "TeleoReactor.hh"
#include "DataTypes.hh"

namespace TREX {
  
  /** @brief A log play %TeleoReactor
   *
   * This class is able to play any Timeline declared in a log file produced by ObservationLogger.
   *
   * @author Frederic Py <fpy@mbari.org>
   */
  class SimAdapter :public TeleoReactor {
  public:
    enum DomainType { ANY = 0, BOOL, INT, FLOAT, STRING, SYMBOL, OBJECT, REAL_INTERVAL, INT_INTERVAL } ;

    /** @brief Constructor
     *
     * @param agentName name of the agent
     * @param configData Configuration data for this instance
     */
    SimAdapter(LabelStr const &agentName, TiXmlElement const &configData);
    /** @brief Destructor
     */
    ~SimAdapter();

    /** @brief Handle intialization signel sent by the Agent
     */
    void handleInit(TICK initialTick, 
		    std::map<double, ServerId> const &serversByTimeline,
		    ObserverId const &observer);

    /** @brief Synchronization
     *
     * This method will play all the Observation for current tick
     * as defined in the log file.
     */
    bool synchronize();

    void notify(Observation const &observation);
    bool handleRequest(TokenId const &token);
    void handleRecall(TokenId const &token);

    void queryTimelineModes(std::list<LabelStr> &externals, 
			    std::list<LabelStr> &internals);
    
    void terminate() {}

    bool isExhausted() const {return m_lastBacktracked == (int) getCurrentTick();}
    void backtrack(){m_lastBacktracked = getCurrentTick();}

  private:
    ObserverId m_observer; //!< Observer connect to the Agent
    std::set<LabelStr> m_internals; /*!< The timelines it will accept goals on and issue observations */
    std::multimap<TICK, Observation *> m_log; //!< Observations extracted from log file
    std::multimap<TICK, Observation *>::iterator m_nextObs; //!< next observation to play
    int m_lastBacktracked;
    DataTypeId m_floatDT;
    DataTypeId m_intDT;
    DataTypeId m_boolDT;
    DataTypeId m_stringDT;
    DataTypeId m_symbolDT;

    bool hasWork() { return false; }
    void resume() {}
    void archive() {}

    /** @brief Observations loading
     *
     * @param file A log file content
     *
     * This file extract all the Observation of @e file corresponding to the timeline
     * in m_internals into m_log to prepare the reactor to play them.
     */
    void loadObservations(TiXmlDocument const &file);

    /** @brief Parsing EnumeratedDomain from XML
     *
     * @param elem A XML element
     *
     * @return The result of parsing
     */
    EnumeratedDomain *xmlAsEnumeratedDomain(TiXmlElement const &elem);

    /** @brief Parsing IntervalDomain from XML
     *
     * @param elem A XML element
     *
     * @return The result of parsing
     */
    IntervalDomain *xmlAsIntervalDomain(TiXmlElement const &elem);

    /** @brief Parsing AbstractDomain from XML
     *
     * @param elem A XML element
     *
     * @return The result of parsing
     */
    AbstractDomain *xmlAsAbstractDomain(TiXmlElement const &elem);

    /** @brief Parsing Observation from XML
     *
     * @param elem A XML element
     *
     * @return The result of parsing
     */
    Observation *xmlAsObservation(TiXmlElement const &elem);

    /** Utilities for type conversion **/
    DataTypeId getFactory(SimAdapter::DomainType t);
    DataTypeId getFactory(const std::string& t);

  }; // TREX::SimAdapter

} // TREX

#endif // _SIMADAPTER_HH
