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

/**
 * @author Conor McGann
 * @file Declares observation related interfaces
 */

#include "Observer.hh"
#include "Token.hh"
#include "Agent.hh"
#include "Object.hh"
#include "TokenVariable.hh"
#include "Timeline.hh"

#include <sstream>

namespace TREX {

  /* SIMPLE BASE CLASS */
  Observation::~Observation(){}

  Observation::Observation(const LabelStr& objectName, const LabelStr& predicateName, unsigned int parameterCount)
    : m_parameterCount(parameterCount),
      m_objectName(objectName),
      m_predicateName(predicateName){}

  const LabelStr& Observation::getObjectName() const {return m_objectName;}

  const LabelStr& Observation::getPredicate() const {return m_predicateName;}

  unsigned int Observation::countParameters() const{ return m_parameterCount; }

  std::string Observation::toString() const{
    std::stringstream sstr;
    sstr << "[" << Agent::instance()->getCurrentTick() << "]ON " << getObjectName().toString() << " ASSERT " << getPredicate().toString();
    if(countParameters() == 0)
      sstr << "{}";
    else {
      sstr << "{ " << std::endl;
      for (unsigned int i = 0; i < countParameters(); i++){
	const std::pair<LabelStr, const AbstractDomain*> nameValuePair = operator[](i);
	sstr << "  " << nameValuePair.first.toString() << "==" << nameValuePair.second->toString() << std::endl;
      }

      sstr << "}";
    }
    return sstr.str();
  }

  /*
  TiXmlElement *Observation::toXML() const {
    TiXmlElement *result = new TiXmlElement("Observation");

    result->SetAttribute("on", getObjectName().c_str());
    result->SetAttribute("predicate", getPredicate().c_str());
    result->SetAttribute("nparams", countParameters());
    for(unsigned i=0; i<countParameters(); ++i ) {
      std::pair<LabelStr, AbstractDomain const *> nameValuePair = operator[](i);
      LabelStr typeName = nameValuePair.second->getTypeName();
      TiXmlElement *assert = new TiXmlElement("Assert");

      assert->SetAttribute("name", nameValuePair.first.c_str());
      assert->LinkEndChild(to_xml(*(nameValuePair.second)));
      result->LinkEndChild(assert);
    }
    return result;
  }
  */

  void Observation::printXML(FILE *out) const {
    size_t i, cnt  = countParameters();
    if( cnt==0 ) 
      fprintf(out, "<Observation on=\"%s\" predicate=\"%s\" />", getObjectName().c_str(), getPredicate().c_str()); // empty tag
    else {
      fprintf(out, "<Observation on=\"%s\" predicate=\"%s\">", getObjectName().c_str(), getPredicate().c_str());
      for( i=0; i<cnt; ++i ) {
	std::pair<LabelStr, AbstractDomain const *> nameValuePair = operator[](i);
	fprintf(out, "<Assert name=\"%s\">", nameValuePair.first.c_str());
	print_xml(out, *(nameValuePair.second));
	fprintf(out, "</Assert>");
      }
      fprintf(out, "</Observation>");
    }
  } 
  

  LabelStr Observation::getTimelineName(const TokenId& token){
    const ObjectDomain& dom = token->getObject()->lastDomain();
    checkError(dom.isSingleton(), "Must be a singleton to use this method. " << dom.toString() << " on " << token->toString());
    TimelineId timeline = dom.getSingletonValue();
    return timeline->getName();
  }

  /* IMPLEMENTATION FOR ObservationByReference */
  ObservationByReference::ObservationByReference(const TokenId& token)
    : Observation(getTimelineName(token), token->getPredicateName(), token->parameters().size()), m_token(token){}

  const std::pair<LabelStr, const AbstractDomain*> ObservationByReference::operator[](unsigned int index) const {
    const ConstrainedVariableId& param = m_token->parameters()[index];
    return std::pair<LabelStr, const AbstractDomain*>(param->getName(), &(param->lastDomain()));
  }


  /* IMPLEMENTATION FOR ObservationByValue */

  ObservationByValue::ObservationByValue(const LabelStr& objectName, const LabelStr& predicateName)
    : Observation(objectName, predicateName, 0) {}

  ObservationByValue::~ObservationByValue(){
    for(unsigned int i = 0; i<m_parameters.size(); i++){
      AbstractDomain* dom = m_parameters[i].second;
      delete dom;
    }
  }

  const std::pair<LabelStr, const AbstractDomain*> ObservationByValue::operator[](unsigned int index) const {
    checkError(countParameters() == m_parameters.size(), "Count: " << countParameters() << ", Size: " << m_parameters.size());
    return std::pair<LabelStr, const AbstractDomain*>(m_parameters[index]);
  }

  void ObservationByValue::push_back(const LabelStr& name, AbstractDomain* dom){
    m_parameters.push_back(std::pair<LabelStr, AbstractDomain*>(name, dom));
    m_parameterCount++;
  }
}
