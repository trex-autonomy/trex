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
 */
#include "Agent.hh"
#include "Adapter.hh"
#include "StringExtract.hh"

namespace TREX {

  Adapter::Adapter(const LabelStr& agentName, const TiXmlElement& configData, bool logDefault)
    : TeleoReactor(agentName, configData, 
		   string_cast<bool>(logDefault, 
				     checked_string(externalConfig(configData).Attribute("log")))) {
    getTimelines(m_internals, externalConfig(configData));
  }

  Adapter::Adapter(const LabelStr& agentName, const LabelStr& name, TICK lookAhead, TICK latency, const LabelStr& configFile, bool logDefault)
    : TeleoReactor(agentName, name, lookAhead, latency, logDefault) {
    getTimelines(m_internals, getConfig(configFile));
  }

  Adapter::Adapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead, TICK latency, bool logDefault)
    : TeleoReactor(agentName, configData, lookAhead, latency, logDefault) {
    getTimelines(m_internals, configData);
  }

  Adapter::~Adapter(){}

  void Adapter::getTimelines(std::set<LabelStr>& results, const TiXmlElement& configSource){
    // Iterate over internal and external configuration specifications
    bool foundAttribute = false;
    LabelStr foundName;
    if (configSource.Attribute("timelineName")) {
      foundAttribute = true;
      foundName = extractData(configSource, "timelineName");
      results.insert(foundName);
    }
    for (TiXmlElement * child = configSource.FirstChildElement();
           child != NULL;
           child = child->NextSiblingElement()) {

        if(strcmp(child->Value(), "Timeline") == 0) {
	  LabelStr name = extractData(*child, "name");
	  if (foundAttribute && foundName == name) {
	    TREXLog() << extractData(configSource, "name") << ": Note: timelineName attribute implies a <timeline> tag. The tag is being ignored.\n";
	  } else {
	    results.insert(name);
	  }
	}
    }
  }

  void Adapter::queryTimelineModes(std::list<LabelStr>& externals, std::list<LabelStr>& internals){
    checkError(!m_internals.empty(), "Adapter configuration error for " << getName().toString() << ". There must be at least one timeline.");

    // Add the standard elements for an adapter
    internals.assign(m_internals.begin(), m_internals.end());
  }

  void Adapter::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, 
			   const ObserverId& observer) {
    m_observer = observer;
  }

  bool Adapter::sendNotify(Observation const &obs) {
    if( m_internals.find(obs.getObjectName())!=m_internals.end() ) {
      checkError( m_observer.isValid(), "Adapter error : unable to notify (Adapter::handleInit was not called)");
      m_observer->notify(obs);
      return true;
    }
    return false;
  }

  const TiXmlElement& Adapter::externalConfig( const TiXmlElement& configSrc){
    if(configSrc.Attribute("config") != NULL){
      const std::string newFile =  extractData(configSrc, "config").c_str();
      return getConfig(newFile);
    }

    return configSrc;
  }

  const TiXmlElement& Adapter::getConfig(const LabelStr& configFile){
    static TiXmlElement* sl_xml = 0;
    static LabelStr sl_fileName;

    // If no element, allocate
    if(sl_xml == NULL){
      sl_xml = LogManager::initXml(configFile.toString());
      sl_fileName = configFile;
    }

    // If wrong element, delete and allocate
    if(!(sl_fileName == configFile)){
      delete sl_xml;
      sl_xml = LogManager::initXml(configFile.toString());
      sl_fileName = configFile;
    }

    return *sl_xml;
  }

}
