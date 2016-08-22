#ifndef H_Utilities
#define H_Utilities

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

#include <string>
#include "Constraint.hh"
#include "PlanDatabaseDefs.hh"

using namespace EUROPA;


namespace TREX {
  #define RETURN_IF_NO_EVAL if (getenv("TREX_NO_EVAL") != NULL) {  if (std::string(getenv("TREX_NO_EVAL")) == "1") { return; } }


  void computeConnectedTokens(const TokenId token, TokenSet& results);

  class ConfigurationException {
  public:
    ConfigurationException(const std::string& description);
    
    const std::string& toString() const;

    static void configurationCheckError(bool check, const std::string& description);
  private:
    const std::string m_description;
  };

  /**
   * @brief Output current time as a string
   */
  std::string timeString();

  void runAgent(const char* configFile, unsigned int stepsPerTick, const char* problemName);

  bool validateResults(const char* problemName);

  bool hasValidFile(const char* problemName);

  bool isValid(const std::string& testStr, const char* problemName);

  void makeValidFile(const std::string& validStr, const char* problemName);

  void makeInvalidFile(const std::string& validStr, const char* problemName);

  void makeFile(const std::string& outputStr, const char* prefix, const char* suffix);

  /**
   * Generate a display string for a token
   */
  std::string tokenToString(const TokenId& tok);

  /**
   * @brief Obatin the fully qualified path name for the given file by searching the local directory and then the path
   */
  std::string findFile(const std::string& fileName, bool forceRebuild = false);

  /**
   * @brief Helper method to access the object name of a token. Assumes object var is a singleton
   */
  const LabelStr& getObjectName(const TokenId& token);

  std::vector<ConstrainedVariableId> appendStateVar(const std::vector<ConstrainedVariableId>& variables);

  /**
   * @brief Utililty for accessing the parent token of a token or rule variable
   */
  TokenId getParentToken(const ConstrainedVariableId& var);

  /**
   * @brief Utility to convert an observation to an xml string.
   */
  //std::string observationToString(const Observation& obs);

  /**
   * @brief Utility convert a string to an observation
   */
  //Observation* observationFromXml(const TiXmlElement& elem);

  /**
   * @brief Binds default values for token parameters if they are not singletons.
   */
  class SetDefault : public Constraint {
  public:
    SetDefault(const LabelStr& name,
	       const LabelStr& propagatorName,
	       const ConstraintEngineId& constraintEngine,
	       const std::vector<ConstrainedVariableId>& variables);


  protected:
    void setSource(const ConstraintId& constraint);
    void handleExecute();
    virtual bool defaultGuardSatisfied();

    TokenId m_token;
    AbstractDomain& m_param;
    AbstractDomain& m_default;
  };

  /**
   * @brief Binds default values when a token is committed
   */
  class SetDefaultOnCommit : public SetDefault {
  public:
    SetDefaultOnCommit(const LabelStr& name,
		      const LabelStr& propagatorName,
		      const ConstraintEngineId& constraintEngine,
		      const std::vector<ConstrainedVariableId>& variables);
  protected:
    bool defaultGuardSatisfied();
  };

  /**
   * @brief Binds default values when a token is committed
   */
  class AbsMaxOnCommit : public Constraint {
  public:
    AbsMaxOnCommit(const LabelStr& name,
		   const LabelStr& propagatorName,
		   const ConstraintEngineId& constraintEngine,
		   const std::vector<ConstrainedVariableId>& variables);
  private:
    void setSource(const ConstraintId& constraint);
    void handleExecute();
    TokenId m_token;
    AbstractDomain& m_param;
  };

  /**
   * @brief bindMax: binds the target with the max possible value from the source
   */
  class BindMax : public Constraint {
  public:
    BindMax(const LabelStr& name,
	    const LabelStr& propagatorName,
	    const ConstraintEngineId& constraintEngine,
	    const std::vector<ConstrainedVariableId>& variables);

    void handleExecute();

  private:
    AbstractDomain& m_target;
    AbstractDomain& m_source;
    static const unsigned int ARG_COUNT = 2;
  };

  /**
   * @brief neighborhood(int a, int b, int n): computes a neighborhood n in a with b elements. n must be enumerated.
   */
  class Neighborhood : public Constraint {
  public:
    Neighborhood(const LabelStr& name,
		 const LabelStr& propagatorName,
		 const ConstraintEngineId& constraintEngine,
		 const std::vector<ConstrainedVariableId>& variables);

    void handleExecute();

  private:
    const AbstractDomain& m_element;
    const AbstractDomain& m_cardinality;
    AbstractDomain& m_neighborhood;
    std::vector<double> m_basis;
    unsigned int m_position;
  };
}
#endif
