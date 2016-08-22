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

/* -*- C++ -*-
 * $Id$
 */
/** @file "ObserverReactor.cc"
 *
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#include "ObserverReactor.hh"
#include "StringExtract.hh"

#include "Token.hh"

using namespace TREX;

/*
 * class ObserverReactor
 */

// Statics :

// Just a copy/paste from Adapter.cc
TiXmlElement const &ObserverReactor::externalConfig(TiXmlElement const &sourceConfig) {
  static TiXmlElement *sl_xml = NULL;
  static std::string sl_fileName;

  std::string const newFile = extractData(sourceConfig, "config").c_str();
  
  if( NULL==sl_xml ) {
    sl_xml = LogManager::initXml(newFile);
    sl_fileName = newFile;
  }

  if( sl_fileName.find(newFile)!=0 ) {
    delete sl_xml;
    sl_xml = LogManager::initXml(newFile);
    sl_fileName = newFile;    
  }
  return *sl_xml;
}

// Just a copy/paste from Adapter.cc
void ObserverReactor::getTimelines(std::map<LabelStr, ServerId> &result, 
				 TiXmlElement const &configData) {
  TiXmlElement const &configSource = externalConfig(configData);

  for(TiXmlElement * child = configSource.FirstChildElement();
      NULL!=child; child = child->NextSiblingElement() ) {
    if( 0==strcmp(child->Value(), "Timeline") ) {
      ServerId noServer;
      LabelStr name = extractData(*child, "name");
      result.insert(std::make_pair(name, noServer));
    }
  }
}

// Structors :

ObserverReactor::ObserverReactor(LabelStr const &agentName, TiXmlElement const &configData)
  :TeleoReactor(agentName, configData) {
  debugMsg("ObserverReactor",nameString() << " Loading configuration.");
  getTimelines(m_externals, configData);
}

ObserverReactor::~ObserverReactor() {
  debugMsg("ObserverReactor:~ObserverReactor", " EOP");
}

// Handlers :

void ObserverReactor::handleInit(TICK initialTick, 
			       std::map<double, ServerId> const &serversByTimeline,
			       ObserverId const &observer) {
  debugMsg("ObserverReactor:handleInit",nameString() << " Connecting to servers.");
  for(std::map<LabelStr, ServerId>::iterator i = m_externals.begin(), 
	endi = m_externals.end(); endi!=i; ++i ) {
    std::map<double, ServerId>::const_iterator it =serversByTimeline.find(i->first);
    
    checkError(serversByTimeline.end()!=it, 
	       "Could not find a server for "<<i->first.toString());
    i->second = it->second; 
  }
}

void ObserverReactor::notify(Observation const &obs) {
  debugMsg("ObserverReactor:notify",nameString() << obs.toString() 
	   << " received at " << getCurrentTick());
}

bool ObserverReactor::handleRequest(const TokenId& goal) {
  // Normally it will be never called
  debugMsg("ObserverReactor:handleRequest",nameString() << " Request received for " 
	   << goal->toString());

  return true;
}

void ObserverReactor::handleRecall(const TokenId& goal) {
  // Normally it will be never called
  debugMsg("ObserverReactor:handleRecall",nameString() << " Recall received for " 
	   << goal->toString());
}

// Observers :

void ObserverReactor::queryTimelineModes(std::list<LabelStr> &externals,
				       std::list<LabelStr> &internals) {
  for(std::map<LabelStr, ServerId>::const_iterator i = m_externals.begin(),
	endi=m_externals.end(); endi!=i; ++i)
    externals.push_back(i->first);
}

std::string ObserverReactor::nameString() const {
    std::ostringstream oss;
    oss << "[" << getName().toString() << "][" << getCurrentTick() << "]";
    return oss.str();
}
