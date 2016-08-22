#ifndef H_TestMonitor
#define H_TestMonitor

/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2009. Willow Garage.
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

#include "Constraint.hh"
#include "Token.hh"
#include "AgentListener.hh"
#include "OpenConditionDecisionPoint.hh"
#include "FlawHandler.hh"

using namespace EUROPA;

/**
 * @file Contains the class definitions for supporting TREX test specifications in NDDL.
 * Usage in NDDL:
 *  assertCompleted(tokenLabel.state);
 *  assertRejected(tokenLabel.state);
 * 
 * Usage from test client:
 *  ASSERT_EQ(TestMonitor::success())
 *  std::cout << testMonitor::toString()
 *  TestMonitor::reset();
 */
namespace TREX {
  using namespace EUROPA;
  using namespace EUROPA::SOLVERS;

  /**
   * @brief We use a custom flaw handler to resolve tokens applied as conditions. The key is the mapping
   * to match the token as part of a TestMonitor
   */
  class TestConditionHandler: public FlawHandler{
  public:
    TestConditionHandler(const TiXmlElement& config);

    std::string toString() const;

    DecisionPointId create(const DbClientId& client, const EntityId& flaw, const LabelStr& explanation) const;

    /**
     * @brief Tests for a match between this factory and the entity
     */
    bool customStaticMatch(const EntityId& entity) const;

    /**
     * @brief Sets a single custom static filer
     */
    unsigned int customStaticFilterCount() const {return 1;}
  };

  /**
   * Constraints below are used to mark tokens with expected results in terms of completed or
   * rejected. This allows assertions of expected behavior in the input NDDL file
   */
  class TestMonitorConstraintBase : public Constraint {
  public:
    virtual ~TestMonitorConstraintBase();

  protected:
    TestMonitorConstraintBase(const LabelStr& name,
			      const LabelStr& propagatorName,
			      const ConstraintEngineId& constraintEngine,
			      const std::vector<ConstrainedVariableId>& variables,
			      bool shouldBeCompleted_);

    void handleExecute(){}
  };

  class CompletionMonitorConstraint: public TestMonitorConstraintBase {
  public:
    CompletionMonitorConstraint(const LabelStr& name,
				const LabelStr& propagatorName,
				const ConstraintEngineId& constraintEngine,
				const std::vector<ConstrainedVariableId>& variables);
  };

  class RejectionMonitorConstraint: public TestMonitorConstraintBase {
  public:
    RejectionMonitorConstraint(const LabelStr& name,
			       const LabelStr& propagatorName,
			       const ConstraintEngineId& constraintEngine,
			       const std::vector<ConstrainedVariableId>& variables);
  };

  /**
   * @brief A class to aggregate expected results and integrate messages from execution to
   * determine if conditions have been met.
   */
  class TestMonitor {
  public:
    /**
     * @brief Return true if all test criteria have been resolved and have where the actual value
     * is the expected value.
     */
    static bool success();

    /**
     * @brief Allows the set of test monitors to be cleared.
     */
    static void reset();

    /**
     * @brief Outputs the expected result
     */
    static std::string toString();

    /**
     * @brief Test if a given key is a registered condition token
     * @return true if the key is an entry in the buffered list. Otherwise false.
     */
    static bool isCondition(int key);

  private:
    /**
     * @brief A Listener to route messages for update of entries in the test monitor
     */
    class AgentListener: public TREX::AgentListener {
    public:
      void notifyRejected(const TokenId& token);
      void notifyCompleted(const TokenId& token);
    };

    friend class TestMonitorConstraintBase;
    friend class TestMonitor::AgentListener;

    /**
     * @brief Call on creation of a constraint to register it for evaluation
     */
    static void registerCondition(int key, const std::string& label, bool expectedValue);

    /**
     * @brief Call on resolution of a token
     */
    static void updateValue(int key, bool completed);
				  
    /**
     * @brief Defines the row structure for test monitor entries. This class is used by the TestMonitor.
     */
    class Entry {
    public:
      Entry(int key_, const std::string& label, bool expectedValue_);
      const int key; // Key of the constraint. Will be unique through all execution
      const std::string label; // The label used to describe the constraint. Derived from the underlying token predicate
      const bool expectedValue; // True if expected to be completed, false if expected to be rejected
      bool actualValue; // Obvious
      bool resolved;// True if the result is specified, otherwise false.
    };

    static std::list< Entry >& entries(){
      static std::list< Entry > sl_entries;
      return sl_entries;
    }

  };
}

#endif
