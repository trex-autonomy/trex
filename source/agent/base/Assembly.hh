#ifndef _H_Assembly
#define _H_Assembly

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
 * @file Assembly.hh
 * @note Adapted from Assembly structure of EUROPA
 */

#include "PlanDatabaseDefs.hh"
#include "RulesEngineDefs.hh"
#include "Module.hh"
#include "Engine.hh"
#include "DbWriter.hh"

using namespace EUROPA;

namespace TREX {

  /**
   * @brief Provides a simple facade over standard components.
   *
   * The standard assembly includes all major EUROPA components for the PlanDatabase:
   * @li ConstraintEngine
   * @li PlanDatabase
   * @li RulesEngine
   * @li Resources
   * @li TemporalNetwork
   */
  class Assembly: public EngineBase {
  public:

    /**
     * @brief Constructor
     * @param agentName The agent name, used to direct DbWriter output
     * @param reactorName The reactor name, also used to direct DbWriter output
     */
    Assembly(const LabelStr& agentName, const LabelStr& reactorName);

    /**
     * @brief Deallocate all data associated with this instance.
     */
    virtual ~Assembly();

    /**
     * @brief Play Transactions stored in the given file
     */
    bool playTransactions(const char* txSource);

    const SchemaId& getSchema() const {return m_schema;}
        
    const ConstraintEngineId& getConstraintEngine() const { return m_constraintEngine; }

    const PlanDatabaseId& getPlanDatabase() const { return m_planDatabase; }

    const RulesEngineId&  getRulesEngine() const { return m_rulesEngine; }

    /**
     * @brief Export Plan Database for Inspection by PlanWorks
     */
    const std::string& exportToPlanWorks(TICK tick, unsigned int attempt);

    /**
     * @brief A plug-in class for schemas
     */
    class SchemaPlugIn {
    public:
      virtual ~SchemaPlugIn();
      virtual void registerComponents(const Assembly& assembly) = 0;
    protected:
      SchemaPlugIn();
    };

    /**
     * @brief Inner class to allow schema registration functions to be invoked. This is necessary because
     * component factory registration is schema specific
     */
    class Schema {
    public:
      /**
       * @brief Singleton accessor. Will allocate an instance of none present
       */
      static Schema* instance();

      /**
       * @brief A Default constructor
       */
      Schema();

      /**
       * @brief Destructor will reset the instance pointer
       */
      virtual ~Schema();

      /**
       * @brief Call back on the schema. Allows customization of schemas for an assembly.
       * @param assembly instance from which schema can be obtained.
       */
      void registerComponents(const Assembly& assembly);

      /**
       * @brief Registration call to extend the set of schema functions used
       */
      void registerPlugIn(SchemaPlugIn* plugIn);

      /**
       * @brief Unregistration call
       */
      void unregisterPlugIn(SchemaPlugIn* plugIn);

    private:
      static Schema* s_instance; /*! Singleton pointer accessor */

      std::vector<SchemaPlugIn*> m_plugIns;
    };

  protected:

    Assembly();

    /**
     * Accessor for partial plan writer. Create on demand.
     */
    DbWriter* getPPW();

    const LabelStr m_agentName;
    const LabelStr m_reactorName;
    SchemaId m_schema;
    ConstraintEngineId m_constraintEngine;
    PlanDatabaseId m_planDatabase;
    RulesEngineId m_rulesEngine;    
    DbWriter* m_ppw; // Optional. Load on demand.
  };
}

/**
 * TREX REGISTRATION MACROS
 */
#define TREX_REGISTER_SCHEMA_PLUGIN(class_name) class_name __class_name;

#define TREX_REGISTER_CONSTRAINT(assembly, class_name, label, propagator)\
  {  \
    ConstraintEngine* ce = (ConstraintEngine*) assembly.getComponent("ConstraintEngine");\
    REGISTER_CONSTRAINT(ce->getCESchema(), class_name, #label, #propagator);\
  }

#define TREX_REGISTER_FLAW_FILTER(assembly, class_name, label) \
  REGISTER_FLAW_FILTER(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

#define TREX_REGISTER_FLAW_HANDLER(assembly, class_name, label) \
  REGISTER_FLAW_HANDLER(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

#define TREX_REGISTER_FLAW_MANAGER(assembly, class_name, label) \
  REGISTER_FLAW_MANAGER(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

#define TREX_REGISTER_COMPONENT_FACTORY(assembly, class_name, label) \
  REGISTER_COMPONENT_FACTORY(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

#define TREX_REGISTER_REACTOR(class_name, label) TREX::TeleoReactor::ConcreteFactory<class_name> __label(#label); 

#endif
