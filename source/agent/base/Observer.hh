#ifndef H_Observer
#define H_Observer
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
#include "TREXDefs.hh"
#include "LabelStr.hh"
#include "AbstractDomain.hh"
#include "PlanDatabaseDefs.hh"

#include "EuropaXML.hh"

namespace TREX {

  /**
   * @brief An observation is a predicate observed at current time.
   */
  class Observation {
  public:
    /**
     * @brief Return the attribute to which the observation applies
     */
    const LabelStr& getObjectName() const;

    /**
     * @brief Return the predicate of the observation
     */
    const LabelStr& getPredicate() const;

    /**
     * @brief Return the number of arguments contained
     */
    unsigned int countParameters() const;

    /**
     * @brief Accessor for the parameter values by position. Pure virtual to allow efficient underlying implementation.
     */
    virtual const std::pair<LabelStr, const AbstractDomain*> operator[](unsigned int index) const = 0;

    /**
     * @brief Utility to help tracing
     */
    std::string toString() const;
  
    /** @brief Utility to serialize data in XML format
     *
     * @sa ObservationLogger 
     */
    //TiXmlElement *toXML() const;

    void printXML(FILE *out) const;

    /**
     * @brief Utility for getting the tokens timeline
     */
    static LabelStr getTimelineName(const TokenId& token);

    virtual ~Observation();

  protected:
    Observation(const LabelStr& objectName, const LabelStr& predicateName, unsigned int parameterCount = 0);
    unsigned int m_parameterCount;

  private:
    const LabelStr m_objectName;
    const LabelStr m_predicateName;
  };

  class ObservationByReference : public Observation {
  public:
    ObservationByReference(const TokenId& token);

    const std::pair<LabelStr, const AbstractDomain*> operator[](unsigned int) const;
  private:
    const TokenId m_token;
  };

  class ObservationByValue: public Observation {
  public:
    ObservationByValue(const LabelStr& objectName, const LabelStr& predicateName);

    ~ObservationByValue();

    const std::pair<LabelStr, const AbstractDomain*> operator[](unsigned int) const;

    void push_back(const LabelStr&, AbstractDomain* dom);

  private:
    std::vector< std::pair<LabelStr, AbstractDomain*> > m_parameters;
  };

  class Observer {
  public:
    virtual void notify(const Observation& observation) = 0;

    virtual ~Observer(){}
  };
}

#endif
