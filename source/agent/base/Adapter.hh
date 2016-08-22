#ifndef H_Adapter
#define H_Adapter
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
 * @file Provides declaration of Adapter which is a TeleoReactor used to integrate external control components
 */
#include <set>

#include "TeleoReactor.hh"

namespace TREX {

  class Adapter: public TeleoReactor {
  public:
    /**
     * @brief Simple constructor.
     * @param agentName the name of the agent in which it is to be included.
     * @param configData the configuration xml element to set up connection. Extendable from base data elements.
     * @param logDefault logging flag value in the absence of the log attribute
     */
    Adapter(const LabelStr& agentName, const TiXmlElement& configData, bool logDefault =false);

    /**
     * @brief Constructor
     * @param agentName The name of the agent in which it belongs.
     * @param name The name of the reactor
     * @param lookead The lookahead
     * @param latency The latency
     * @param configSrc The configuration file to configure timelines
     * @param logDefault logging flag value in the absence of the log attribute
     */
    Adapter(const LabelStr& agentName, const LabelStr& name, TICK lookAhead, TICK latency, const LabelStr& configSrc,
	    bool logDefault=false);

    /**
     * @brief Constructor for use when config data in the derived class is standardized to avoid unnecessary verbiage in the
     * configuration file
     * @param agentName the name of the agent in which it is to be included.
     * @param configData the configuration xml element to set up connection. Extendable from base data elements.
     * @param lookead The lookahead
     * @param latency The latency
     * @param logDefault logging flag value in the absence of the log attribute
     */
    Adapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead, TICK latency, bool logDefault =false);

    virtual ~Adapter();

    /**
     * @brief Provide internals and clients only. That is, it can be the source for observations and the sink for goals
     * @param externals Append to this list any timeline names for which the adapter wishes to subscribe to updates.
     * @param internals Append to this list any timeline names for which this adapter is the owner. It can receive requests and publish
     * observations for values on this timeline.
     */
    void queryTimelineModes(std::list<LabelStr>& externals, std::list<LabelStr>& internals);

    void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer);

    /**
     * Helper method to re-use code to populate timelines
     */
    static void getTimelines(std::set<LabelStr>& results, const TiXmlElement& configData);

    static const TiXmlElement& externalConfig(const TiXmlElement& sourceConfig);

    static const TiXmlElement& getConfig(const LabelStr& configFile);
  protected:
    /* STUBS since adapter should handle immediately */
    bool hasWork() {return false;}
    void resume(){}
    bool sendNotify(Observation const &obs);

  private:
    ObserverId m_observer;
    std::set<LabelStr> m_internals; /*!< The timelines it will accept goals on and issue observations */
  };
}
#endif
