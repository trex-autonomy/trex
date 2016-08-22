#ifndef H_DBSOLVER
#define H_DBSOLVER

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

#include "Id.hh"
#include "Solver.hh"
#include "OpenConditionManager.hh"
#include "XMLUtils.hh"
#include <vector>

using namespace EUROPA;
using namespace EUROPA::SOLVERS;

namespace TREX {
  class DbSolver;
  class AbstractSolver;
  typedef Id<AbstractSolver> AbstractSolverId;
  typedef Id<DbSolver> DbSolverId;

  /**
   * @brief Serves as a composition of solvers.
   *
   * The DbSolver allows different solvers to be composed. For example, a
   * chronological backtracking solver (from EUROPA) can be composed with
   * an TSP solver. This provides and extensible interface for implementing
   * different planning algorithims.
   */
  class DbSolver {
  public:
    /**
     * @brief Creates the DbSolver.
     */
    DbSolver(const PlanDatabaseId& db, TiXmlElement* cfgXml);
    /**
     * @brief Destructor.
     */
    ~DbSolver();
    /**
     * @brief Tests if the search space have been exhausted. 
     */
    bool isExhausted();
    /**
     * @brief Iterates through the solvers and steps each of them.
     */
    void step();
    /**
     * @brief The size of the search stack
     */
    unsigned int getDepth();
    /**
     * @brief The total number of search steps since the Solver was previously cleared.
     */
    unsigned int getStepCount();  
    /**
     * @brief Tests if we have concluded there are no more flaws.
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
     * @brief Test if the given entity is in deliberation
     */
    bool inDeliberation(const EntityId& entity) const;

  private:
    const PlanDatabaseId m_db;
    std::vector<AbstractSolverId> m_solvers; /*! The list of solvers. */
  };  

  /**
   * @brief Provides an abstract interface for solvers.
   */
  class AbstractSolver : public Component {
  public:
    /**
     * @brief Creates the AbstractSolver.
     */
    AbstractSolver(const TiXmlElement& cfgXml);
    /**
     * @brief Destroys the AbstractSolver.
     */
    virtual ~AbstractSolver();

    /**
     * @brief Name accessor
     */
    const LabelStr& getName() const {return m_name;}

    /**
     * @brief Access the context of this Solver.
     */
    ContextId getContext() const {return m_context;}

    /**
     * @brief Initializes the solver.
     */
    virtual void init(PlanDatabaseId db, TiXmlElement* cfgXml){m_db = db;}

    /**
     * @brief Tests if the solver has exhausted its search space.
     */
    virtual bool isExhausted() = 0;
    /**
     * @brief Steps the solver.
     */
    virtual void step() = 0;
    /**
     * @brief Gets the depth of the solver.
     */
    virtual unsigned int getDepth() = 0;
    /**
     * @brief Gets the step count of the solver.
     */
    virtual unsigned int getStepCount() = 0;
    /**
     * @brief Tests if there are no more flaws for the solver.
     */
    virtual bool noMoreFlaws() = 0;
    /**
     * @brief Clears current decisions on the stack without any modifications to the plan.
     */
    virtual void clear() = 0;
    /**
     * @brief Retracts all decisions stored in the internal decision stack.
     * @note Will not force propagation.
     */
    virtual void reset() = 0;

    /**
     * @brief Test if the given entity is in deliberation
     */
    virtual bool inDeliberation(const EntityId& entity) const {return false;}
  protected:
    PlanDatabaseId m_db;
    LabelStr m_name;
    ContextId m_context; /*<! Used to share data from the Solver on down.*/
  };
  
  /**
   * @brief An interface layer for the solver from EUROPA.
   *
   * This class merely runs the methods of the same name on the EUROPA
   * solver.
   */
  class EuropaSolverAdapter : public AbstractSolver {
  public:
    /**
     * @brief Creates the EuropaSolverAdapter.
     */
    EuropaSolverAdapter(const TiXmlElement& cfgXml);

    /**
     * @brief Destroys the EuropaSolverAdapter.
     */
    ~EuropaSolverAdapter();

    /**
     * @brief Initializes the solver.
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
     * @breif Will check the solver stack
     */
    virtual bool inDeliberation(const EntityId& entity) const;

  private:
    SolverId m_solver; /*! The Europa Solver */
  };

  /**
   * @brief Provides and interface to create the solvers that use flaw managers.
   *
   * This class can be extended to create solvers that use flaw managers. The
   * class loads the flaw managers, and adds them to a list. It also creates
   * a DbListener.
   *
   * Inside of a subclass, you must put the following in your init method:
   * initDbListener(db, cfgXml), if you want a db listener.
   * initFlawManagers(db, cfgXml), if you want to load flaw managers.
   */
  class FlawManagerSolver : public AbstractSolver {
  public:
    /**
     * @brief Creates the solver.
     */
    FlawManagerSolver(const TiXmlElement& cfgXml);
    /**
     * @brief Destroys the solver.
     */
    virtual ~FlawManagerSolver();

  protected:
    /**
     * @brief Creates the DbListener. A subclass that wants to have one must call it.
     */
    void initDbListener(PlanDatabaseId db, TiXmlElement* cfgXml);
    /**
     * @brief Creates the CeListener. A subclass that wants to have one must call it.
     */
    void initCeListener(PlanDatabaseId db, TiXmlElement* cfgXml);
    /**
     * @brief Called by a subclass. Initializes the flaw managers.
     */
    void initFlawManagers(PlanDatabaseId db, TiXmlElement* cfgXml);
    /**
     * @brief Gets the I-th flaw manager.
     */
    FlawManagerId getFlawManager(unsigned int i);
    /**
     * @brief Gets the number of flaw managers.
     */
    unsigned int getFlawManagerCount();
  private:
    /**
     * @brief Called by the DbListener when a flaw is added.
     */
    void notifyAdded(const TokenId& token);
    /**
     * @brief Called by the DbListener when a flaw is removed. 
     */
    void notifyRemoved(const TokenId& token);
    /**
     * @brief Serves as a listener for the plan database.
     */
    class DbListener : public PlanDatabaseListener {
    public:
      /**
       * @brief Creates the listener.
       */
      DbListener(const PlanDatabaseId& db, FlawManagerSolver& dm);
      /**
       * @brief Called when a flaw is removed.
       */
      void notifyRemoved(const TokenId& token);
      /**
       * @brief Called when a flaw is added. 
       */
      void notifyAdded(const TokenId& token);
    private:
      FlawManagerSolver& m_solver; /*! The flaw manager solver that will recive messages. */
    };
    /**
     * @brief Plugs manager into ConstraintEngine events to synchronize flaw candidates
     */
    class CeListener: public ConstraintEngineListener {
    public:
      CeListener(const ConstraintEngineId& ce, FlawManagerSolver& dm);
      
      void notifyRemoved(const ConstrainedVariableId& variable);
      void notifyChanged(const ConstrainedVariableId& variable, const DomainListener::ChangeType& changeType);
      void notifyAdded(const ConstraintId& constraint);
      void notifyRemoved(const ConstraintId& constraint);
      
    private:
      FlawManagerSolver& m_solver;
    };
    
    void notifyRemoved(const ConstrainedVariableId& variable);
    void notifyChanged(const ConstrainedVariableId& variable, const DomainListener::ChangeType& changeType);
    void notifyAdded(const ConstraintId& constraint);
    void notifyRemoved(const ConstraintId& constraint);
    
    friend class FlawManagerSolver::DbListener;
    friend class FlawManagerSolver::CeListener;

    
    FlawManagerSolver::DbListener *m_dbListener; /*! The DbListener */
    FlawManagerSolver::CeListener *m_ceListener; /*! The DbListener */
    std::vector<FlawManagerId> m_flawManagers; /*! The list of FlawManagers. */
  };  
  


}

#endif
