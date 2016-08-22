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
/** @file "SimAdapter.cc"
 *
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#include "Utils.hh"
#include "Domains.hh"
#include "Token.hh"
#include "Adapter.hh"
#include "Assembly.hh"
#include "SimAdapter.hh"
#include "Agent.hh"
#include "Utilities.hh"

using namespace TREX;

/*
 * class TREX::SimAdapter
 */

// Statics :

#define GET_DATA_TYPE(dt) getPlanDatabase()->getSchema()->getCESchema()->getDataType(dt)

EnumeratedDomain *SimAdapter::xmlAsEnumeratedDomain(TiXmlElement const &elem) {
  std::string typeName;
  SimAdapter::DomainType type = SimAdapter::ANY;

  // Guess the type :
  for(TiXmlElement *child_el = elem.FirstChildElement(); 
      NULL!=child_el; child_el=child_el->NextSiblingElement()) {
    std::string thisType;

    if( strcmp(child_el->Value(), "value")==0 ) 
      thisType = child_el->Attribute("type");
    else
      thisType = child_el->Value();

    checkError(!thisType.empty(), 
	       "SimAdapter:xmlAsEnumeratedDomain error : unable to determine the type of domain.");
    if( "bool"==thisType || "BOOL"==thisType || 
	BoolDT::NAME()==thisType ) {
      if( ANY==type ) {
	type = BOOL;
	typeName = "bool";
      } 
      if( BOOL==type )
	continue;
    } else if( "int"==thisType || "INT"==thisType || "INT_INTERVAL"==thisType ||
	       IntDT::NAME()==thisType ) {
      if( ANY==type ) {
	type = INT;
	typeName = "int";
      } 
      if( INT==type || SimAdapter::FLOAT==type )
	continue;
    } else if( "float"==thisType || "FLOAT"==thisType || "REAL_INTERVAL"==thisType ||
	       FloatDT::NAME()==thisType ) {
      if( ANY==type || INT==type ) {
	type = SimAdapter::FLOAT;
	typeName = "float";
      } 
      if( SimAdapter::FLOAT==type )
	continue;
    } else if( "string"==thisType || "STRING"==thisType ||
	       StringDT::NAME()==thisType ) {
      if( ANY==type ) {
	type = STRING;
	typeName = "string";
      } 
      if( STRING==type )
	continue;
    } else if( "symbol"==thisType ) {
      if( ANY==type ) {
	type = SYMBOL;
	typeName = child_el->Attribute("type");
      } 
      if( SYMBOL==type ) {
	checkError(typeName==child_el->Attribute("type"), 
		   "SimAdapter:xmlAsEnumeratedDomain : symbols from different type in the same enumerated set.");
	continue;
      }
    } else if( "object"==thisType ) {
      checkError(ALWAYS_FAIL, 
		 "SimAdapter:xmlAsEnumeratedDomain : object type is not yet supported.");
    }
    checkError(ALWAYS_FAIL, 
	       "SimAdapter:xmlAsEnumeratedDomain : mixed types in the same enumerated set.");
  }
  checkError(ANY!=type, 
	     "SimAdapter:xmlAsEnumeratedDomain : Unable to guess the set type.");

  // Gather the values :
  std::list<double> values;
  
  for(TiXmlElement *child_el=elem.FirstChildElement();
      NULL!=child_el; child_el=child_el->NextSiblingElement()) {
    char const *value_st = child_el->Attribute("value");
    
    if( NULL==value_st )
      value_st = child_el->Attribute("name");
    checkError(NULL!=value_st,
	       "SimAdapter:xmlAsEnumeratedDomain : unable to extract value.");
    switch( type ) {
    case SimAdapter::FLOAT: 
    case SimAdapter::REAL_INTERVAL: 
    case SimAdapter::BOOL: 
    case SimAdapter::INT: 
    case SimAdapter::INT_INTERVAL: 
    case SimAdapter::STRING: 
    case SimAdapter::SYMBOL:
      values.push_back(m_symbolDT->createValue(value_st));
      break;
    case SimAdapter::OBJECT:
    default:
      check_error(ALWAYS_FAIL); // will never happen
    }
  }

  // Return the domain :
  switch( type ) {
  case SimAdapter::BOOL: 
  case SimAdapter::INT: 
  case SimAdapter::INT_INTERVAL: 
  case SimAdapter::FLOAT:
  case SimAdapter::REAL_INTERVAL:
    return new EnumeratedDomain(getFactory(type), values);
  case SimAdapter::STRING:
    return new StringDomain(values, getFactory(type));
  case SimAdapter::SYMBOL:
    return new SymbolDomain(values, getFactory(type));
  case SimAdapter::OBJECT:
  default:
    check_error(ALWAYS_FAIL); // will never happen
    return NULL;
  }
} // SimAdapter::xmlAsEnumeratedDomain(TiXmlElement const &)

IntervalDomain *SimAdapter::xmlAsIntervalDomain(TiXmlElement const &elem) {
  char const *min_st = elem.Attribute("min"); 
  char const *max_st = elem.Attribute("max");

  IntervalDomain *domain = dynamic_cast<IntervalDomain *>(m_floatDT->baseDomain().copy());
  
  checkError(NULL!=domain,
	     "SimAdapter:xmlAsIntervalDomain : type \""<< elem.Attribute("type") <<"\" is not an interval domain type.");

  double min = m_floatDT->createValue(min_st);
  double max = m_floatDT->createValue(max_st);
  domain->intersect(min, max);
  return domain;
} // SimAdapter::xmlAsIntervalDomain(TiXmlElement const &)

AbstractDomain *SimAdapter::xmlAsAbstractDomain(TiXmlElement const &elem) {
  std::string tag = elem.Value();
  
  if( "new"==tag ) {
    checkError(ALWAYS_FAIL, 
	       "SimAdapter:xmlAsAbstractDomain : \"new\" object domain is not supported.");
  } else if( "id"==tag ) {
    checkError(ALWAYS_FAIL,
	       "SimAdapter:xmlAsAbstractDomain : \"id\" contrained variable domain is not supported.");
  } else if( "set"==tag ) 
    return xmlAsEnumeratedDomain(elem);
  else if( "interval"==tag )
    return xmlAsIntervalDomain(elem);
  else if( "value"==tag ) {
    char const *type = elem.Attribute("type"); 
    checkError(NULL!=type,
	       "SimAdapter:xmlAsAbstractDomain : missing type for domain.");
    char const *name = elem.Attribute("name");
    checkError(NULL!=name,
	       "SimAdapter:xmlAsAbstractDomain : missing name for domain.");

    AbstractDomain *domain = getFactory(type)->baseDomain().copy();
    double val = getFactory(type)->createValue(name);
    
    if(domain->isOpen() && !domain->isMember(val))
      domain->insert(val);
    domain->set(val);
    return(domain);
  } else {
    char const *val_st = elem.Attribute("value");
    char const *type = elem.Attribute("type"); 

    checkError(NULL!=val_st,
	       "SimAdapter:xmlAsAbstractDomain : missing value for domain.");
    checkError(NULL!=type,
	       "SimAdapter:xmlAsAbstractDomain : missing type for domain.");

    if(strcmp(type, "string") == 0){
      return new StringDomain(val_st, m_stringDT);
    }
    else {
      checkError("object"!=tag,
		 "SimAdapter:xmlAsAbstractDomain : object parsing not supported.");
      return new SymbolDomain(LabelStr(val_st), m_symbolDT);
    }
  }

  checkError(ALWAYS_FAIL, 
	     "SimAdapter:xmlAsAbstractDomain : unknown type");
  return NULL;
} // SimAdapter::xmlAsAbstractDomain(TiXmlElement const &)

Observation *SimAdapter::xmlAsObservation(TiXmlElement const &elem) {
  char const *name = elem.Attribute("on");
  checkError(NULL!=name, 
	     "SimAdapter:xmlAsObservation : missing \"on\" attribute");
  char const *pred = elem.Attribute("predicate");
  checkError(NULL!=pred, 
	     "SimAdapter:xmlAsObservation : missing \"predicate\" attribute");
  ObservationByValue *obs = new ObservationByValue(name, pred);
  
  for(TiXmlElement const *i=elem.FirstChildElement(); NULL!=i;
      i = i->NextSiblingElement()) {
    char const *name = i->Attribute("name");
    checkError(NULL!=name,
	       "SimAdapter:xmlAsObservation : missing \"name\" for observation variable.");
    AbstractDomain *dom = xmlAsAbstractDomain(*(i->FirstChildElement()));
    
    obs->push_back(name, dom);
  }

  return obs;
} // SimAdapter::xmlAsObservation(TiXmlElement const &)

// Structors :

SimAdapter::SimAdapter(LabelStr const&agentName, 
		       TiXmlElement const &configData) 
  :TeleoReactor(agentName, configData), m_lastBacktracked(-1),
   m_floatDT(FloatDT::instance()),
   m_intDT(IntDT::instance()),
   m_boolDT(BoolDT::instance()),
   m_stringDT(StringDT::instance()),
   m_symbolDT(SymbolDT::instance()){
  std::string s = agentName.toString() + ".log";
  std::string file_name = findFile(s);
  TiXmlDocument xml_log(LogManager::use(file_name));

  TREX_INFO("trex:info", "Loading log input file \""<<file_name<<'\"');
  ConfigurationException::configurationCheckError(xml_log.LoadFile(), nameString() + "Unable to load xml file \"" + file_name + '\"');


  Adapter::getTimelines(m_internals,  Adapter::externalConfig(configData));

  loadObservations(xml_log);
  
} // SimAdapter::SimAdapter

SimAdapter::~SimAdapter() {}

// Modifiers :

void SimAdapter::loadObservations(TiXmlDocument const &file) {
  debugMsg("SimAdapter", "["<<getName().toString()<<"] : loading observation history.");
  TICK maxTick = 0;


  for(TiXmlElement const *tick=firstPath(&file, "Log/Tick")->ToElement();
      NULL!=tick; tick=tick->NextSiblingElement()) 
    if( 0==strcmp(tick->Value(), "Tick") ) {
      std::pair<TICK, Observation *> entry;

      entry.first = strtol(tick->Attribute("value"), NULL, 0);

      if( entry.first>maxTick )
	maxTick = entry.first;

       debugMsg("SimAdapter", "["<<getName().toString()<<"] : observations for tick "<<entry.first);
      for(TiXmlElement const *elem = tick->FirstChildElement();
	  NULL!=elem; elem=elem->NextSiblingElement()) {
	LabelStr timeline = elem->Attribute("on");
	
	if( m_internals.find(timeline)!=m_internals.end() ) {
 	  debugMsg("SimAdapter", "["<<getName().toString()<<"] \t - on <"<<timeline.toString()<<">");

	  entry.second = xmlAsObservation(*elem);
	  m_log.insert(entry);
	}
      }
    }
  debugMsg("SimAdapter", "["<<getName().toString()<<"] : observation history loaded ("<<maxTick<<", "<<m_log.size()<<").");
  m_nextObs = m_log.begin();
}

void SimAdapter::handleInit(TICK initialTick, 
			    std::map<double, ServerId> const &serversByTimeline,
			    ObserverId const &observer) {
  m_observer = observer;
} // SimAdapter::handleInit(TICK, std::map<double, ServerId> const &, ObserverId const &)

bool SimAdapter::synchronize() {
  if( m_log.end()!=m_nextObs ) {
    TICK curTick = getCurrentTick();

    checkError(curTick<=m_nextObs->first, 
	       "SimAdapter:synchronize : playable tick ("<<m_nextObs->first<<") is in the past."); 

    for( ; m_log.end()!=m_nextObs && curTick>=m_nextObs->first; ++m_nextObs ) {
      Observation *obs = m_nextObs->second;
      m_nextObs->second = NULL;
      
      debugMsg("SimAdapter", "["<<getName().toString()<<"]["<<curTick<<"] observation on < "
	       <<obs->getObjectName().toString()<<" >");
   
      m_observer->notify(*obs);
      delete obs;
    }
  }
  else {
    Agent::terminate();
  }

  return true;
} // SimAdapter::synchronize()

// Observers :

void SimAdapter::queryTimelineModes(std::list<LabelStr> &externals, 
				    std::list<LabelStr> &internals) {
  checkError(!m_internals.empty(), 
	     "SimAdapter configuration error for " << getName().toString() 
	     << ". There must be at least one timeline.");
  // Add the standard elements for an adapter
  internals.assign(m_internals.begin(), m_internals.end());
} // SimAdapter::queryTimelineModes(std::list<LabelStr> &, std::list<LabelStr> &)


void SimAdapter::notify(Observation const &obs) {
  debugMsg("SimAdapter:notify", " Ignoring "<<obs.toString());
}

bool SimAdapter::handleRequest(TokenId const &token) {
  debugMsg("SimAdapter:handleRequest", " Ignoring token "<<token->toString());
  return true;
}

void SimAdapter::handleRecall(TokenId const &token) {
  debugMsg("SimAdapter:handleRecall", " Ignoring token"<<token->toString());
}

DataTypeId SimAdapter::getFactory(SimAdapter::DomainType t){
    switch( t ) {
    case SimAdapter::FLOAT:
    case SimAdapter::REAL_INTERVAL:
      return m_floatDT;
    case SimAdapter::INT_INTERVAL:
    case SimAdapter::INT: 
      return m_intDT;
    case SimAdapter::BOOL: 
      return m_boolDT;
    case SimAdapter::STRING: 
      return m_stringDT;
    case SimAdapter::SYMBOL:
      return m_symbolDT;
    default:
      break;
    }

   assertTrue(ALWAYS_FAIL, "No match on input type" + t);
   return m_floatDT;
}

 DataTypeId SimAdapter::getFactory(const std::string& t){
   if("float" == t || "REAL_INTERVAL" == t)
     return m_floatDT;
   if("int" == t || "INT_INTERVAL" == t)
      return m_intDT;
   if("bool" == t)
      return m_boolDT;
   if("string" == t)
      return m_stringDT;
   if("symbol" == t)
      return m_symbolDT;

   assertTrue(ALWAYS_FAIL, "No match on input type" + t);
   return m_floatDT;
 }


TREX_REGISTER_REACTOR(SimAdapter, SimAdapter);
