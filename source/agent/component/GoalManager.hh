#ifndef H_GOALMANAGER
#define H_GOALMANAGER

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

#include "OpenConditionManager.hh"
#include "FlawFilter.hh"

/**
 * @brief The goal manager.
 */

using namespace EUROPA::SOLVERS;

namespace TREX {
  struct Position {
    double x;
    double y;
  };
  class CostEstimator;

  class GoalsOnlyFilter: public FlawFilter {
  public:
    GoalsOnlyFilter(const TiXmlElement& configData);
    bool test(const EntityId& entity);
  };

  class NoGoalsFilter: public FlawFilter {
  public:
    NoGoalsFilter(const TiXmlElement& configData);
    bool test(const EntityId& entity);
  }; 

  typedef Id<CostEstimator> CostEstimatorId;

  /**
   * @brief Goal Manager to manage goal flaw selection. Will involve solving an orienteering problem
   * over the relaxed problem to provide a good ordering. We can imagine this manager maintaining
   * a priority q and releasing goals 1 at a time.
   */
  class GoalManager: public OpenConditionManager {
  public:

    /**
     * @brief Solution is a sequence of tokens. None can be rejected. Each has a 2d position.
     */
    typedef std::list<TokenId> SOLUTION;

    /**
     * @brief Uses standard constructor
     */
    GoalManager(const TiXmlElement& configData);


    /**
     * @brief Destructor
     */
    ~GoalManager();

    // Assumptions about the fields in a goal
    DECLARE_STATIC_CLASS_CONST(LabelStr, X, "x");
    DECLARE_STATIC_CLASS_CONST(LabelStr, Y, "y");
    DECLARE_STATIC_CLASS_CONST(LabelStr, PRIORITY, "priority");

    // Parameters used for configuration
    DECLARE_STATIC_CLASS_CONST(LabelStr, CFG_POSITION_SOURCE, "positionSource");
    DECLARE_STATIC_CLASS_CONST(LabelStr, CFG_MAP_SOURCE, "mapSource");
    DECLARE_STATIC_CLASS_CONST(LabelStr, CFG_MAX_ITERATIONS, "maxIterations");
    DECLARE_STATIC_CLASS_CONST(LabelStr, CFG_PLATEAU, "plateau");
    /**
     * @brief True if the token is the next in the plan.
     */
    bool isNextToken(TokenId token);
    /**
     * @brief True if the planner has no more work to do.
     */
    bool noMoreFlaws();
    /**
     * @brief Steps the solver.
     */
    void step();

    /**
     * @brief Resets internal state
     */
    void reset();

  private:
    /**
     * @brief Used to synch mark current goal value as dirty
     * @see OpenConditionManager::addFlaw
     */
    void addFlaw(const TokenId& token);

    /**
     * @brief Used to synch mark current goal value as dirty
     * @see OpenConditionManager::removeFlaw
     */
    void removeFlaw(const TokenId& token);

    /**
     * @brief Used to synch mark current goal value as dirty
     */
    void handleInitialize();

    /**
     * @brief Generate an initial solution. May not be feasible.
     */
    void generateInitialSolution();

    /**
     * @brief Helper method
     */
    int getPriority(const TokenId& token);

    /**
     * @brief Evaluate the solution s. This will compute both the cost and the utility.
     * @return true if feasible, false if infeasible
     */
    bool evaluate(const SOLUTION& s, double& cost, double& utility);

    /**
     * @brief Compute a neighboring solution for s
     */
    void selectNeighbor(GoalManager::SOLUTION& s, TokenId& delta);

    /**
     * @brief Set initial conditions in terms of position, time and energy
     */
    void setInitialConditions();

    /**
     * @brief Get a distance estimate between points.
     */
    virtual double computeDistance(const Position& p1, const Position& p2);

    /**
     * @brief Accessor to get a Position. For convenience
     */
    static Position getPosition(const TokenId& token);

    /**
     * @brief Accessor to get current position (at current tick)
     */
    Position getCurrentPosition();

    /**
     * @brief Accessor for robot speed
     */
    double getSpeed() const;

    /**
     * @brief Comparator
     */
    int compare(const SOLUTION& s1, const SOLUTION& s2);

    void insert(SOLUTION&s, const TokenId& t, unsigned int pos);
    void swap(SOLUTION& s, unsigned int a, unsigned int b);
    void remove(SOLUTION& s, const TokenId& t);
    void update(SOLUTION& s, TokenId& delta, const SOLUTION& c, TokenId t);


    std::string toString(const SOLUTION& s);

    void postConstraints();

    /** The state of the system. */
    enum State {
      STATE_DONE, STATE_PLANNING, STATE_REQUIRE_PLANNING
    };

    /**
     * @brief Set the state. Encapsulate all change.
     */
    void setState(const State& s);

    // Configuration derived members
    unsigned int m_maxIterations;
    unsigned int m_plateau;
    LabelStr m_positionSourceCfg;
    TimelineId m_positionSource;

    State m_state;
    SOLUTION m_currentSolution;
    TokenSet m_ommissions;
    std::vector< std::pair<int, ConstraintId> > m_constraints;

    // Iteration variables.
    unsigned int m_iteration, m_watchDog;

    /*!< INITIAL CONDITIONS */
    int m_startTime;
    double m_timeBudget;  
    
    Position m_position; /*! Cached position. */

    // Integration with wavefront planner
    //plan_t* wv_plan;

    static const int WORSE = -1;
    static const int EQUAL = 0;
    static const int BETTER = 1;
  }; 


  typedef Id<GoalManager> GoalManagerId;

  class CostEstimator : public Component {
  public:
    CostEstimator(const TiXmlElement& configData);
    virtual ~CostEstimator();
    virtual double computeDistance(const Position& p1, const Position& p2) = 0;
  };

  class EuclideanCostEstimator : public CostEstimator {
  public:
    EuclideanCostEstimator(const TiXmlElement& configData);
    ~EuclideanCostEstimator();
    double computeDistance(const Position& p1, const Position& p2);
    
  };

}

#endif
