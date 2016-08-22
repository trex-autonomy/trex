#ifndef H_ORIENTEERINGSOLVER
#define H_ORIENTEERINGSOLVER

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

#include "DbSolver.hh"
#include "GoalManager.hh"

namespace TREX {
  class DynamicGoalFilter : public FlawFilter {
  public:
    DynamicGoalFilter(const TiXmlElement& configData);
    bool test(const EntityId& entity);
  };

  class OrienteeringSolver;
  typedef Id<OrienteeringSolver> OrienteeringSolverId;

  /**
   * @brief A Steepest Ascent Hill Climbing Algorithm for Orienteering Problems.
   */
  class OrienteeringSolver : public FlawManagerSolver {
  public:
    /**
     * @brief Creates the OrienteringSolver.
     */
    OrienteeringSolver(const TiXmlElement& cfgXml);
    /**
     * @brief Destroys the OrienteringSolver.
     */
    ~OrienteeringSolver();
    /**
     * @brief Inits the solver.
     */
    void init(PlanDatabaseId db, TiXmlElement* cfgXml);
    /**
     * @brief Tests if the solver has exhausted its search space.
     */
    bool isExhausted();
    /**
     * @brief Steps the solver.
     */
    void step();
    /**
     * @brief Gets the depth of the solver.
     */
    unsigned int getDepth();
    /**
     * @brief Gets the step count of the solver.
     */
    unsigned int getStepCount();
    /**
     * @brief Tests if there are no more flaws for the solver.
     */
    bool noMoreFlaws();
    /**
     * @brief Clears current decisions on the stack without any modifications to the plan.
     */
    void clear();
    /**
     * @brief Retracts all decisions stored in the internal decision stack.
     * @note Will not force propagation.
     */
    void reset();
    /**
     * @brief Tests all orientering solvers to see if the token is the next goal.
     */
    static bool isGlobalNextGoal(TokenId token);
    /**
     * @brief Tests is the token is the next goal.
     */
    bool isNextGoal(TokenId token);
  private:
    GoalManagerId m_goalManager; /*! The goal manager. */
    static std::vector<OrienteeringSolverId> s_goalSolvers;  /*! Static list of solvers. */
    unsigned int m_stepCount; /*! Counts steps. */
  };
}



#endif

