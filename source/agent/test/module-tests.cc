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

#include "Agent.hh"
#include "Schema.hh"
#include "Debug.hh"
#include "Nddl.hh"
#include "Utilities.hh"
#include "TestMonitor.hh"
#include <pthread.h>
#include <time.h>
#include <errno.h>

#include <iostream>

using namespace EUROPA;

using namespace TREX;

#define runTest(test) \
  std::cout << "Running " << #test << " ....";\
  std::cout << (test() ? " SUCCESS" : "FAILED") << std::endl;

#define runTestSuite(test) { \
  try{ \
  std::cout << #test << "***************" << std::endl; \
  if (test()) \
    std::cout << #test << " PASSED." << std::endl; \
  else \
    std::cout << #test << " FAILED." << std::endl; \
  }\
  catch(TREX::ConfigurationException* e){							\
    std::cout << #test << " FAILED with an exception." <<  e->toString() << std::endl; \
    exit(-1); \
  }\
  catch(char* e){\
    std::cout << #test << " FAILED. "  << e << std::endl; \
    exit(-1); \
  }\
  catch(std::runtime_error e){\
    std::cout << #test << " FAILED with an exception." <<  e.what() << std::endl; \
    exit(-1); \
  }\
  catch(...){\
    std::cout << #test << " FAILED with an exception.";	\
    exit(-1); \
  }\
}

/**
 * Run problem and handle setup and clean up
 */
void runAgentWithSchema(const char* configFile, unsigned int stepsPerTick, const char* problemName){
  TREX::TestMonitor::reset();
  TREX::runAgent(configFile, stepsPerTick, problemName);
  bool result = TREX::TestMonitor::success();
  assertTrue(result, TREX::TestMonitor::toString().c_str());
}

class GamePlayTests {
public:
  static bool test(){ 
    runTest(testActionAdapter);
    runTest(testDispatch);
    runTest(testSqueezeObserver);
    runTest(testSimulation);
    runTest(testUndefinedSingleTimeline);
    runTest(testUndefinedDerived);
    runTest(testPersonalRobots);
    runTest(testSynch);
    runTest(testExtensions);
    runTest(testRecall);
    runTest(testRepair);
    runTest(testLogging);
    runTest(testPersistence);
    runTest(testSimulationWithPlannerTimeouts);
    runTest(testScalability);
    runTest(testTestMonitor);
    runTest(testActions);
    runTest(bugFixes); 
    runTest(testOneDeliberatorOneAdapter);
    runTest(OrienteeringSolver);
    runTest(testInconsistent);
    runTest(testOneStepAhead);
    runTest(testFileSearch);
    return true;
  }

private:

  static bool testActionAdapter(){
    runAgentWithSchema("action_adapter.0.cfg", 50, "action_adapter.0");
    return true;
  }

  static bool testPersonalRobots(){
    runAgentWithSchema("personal_robots/pr.0.cfg", 50, "pr.0");
    return true;
  }

  static bool testExtensions(){
    runAgentWithSchema("extensions.2.cfg", 50, "extensions.2");
    runAgentWithSchema("extensions.1.cfg", 50, "extensions.1");
    runAgentWithSchema("extensions.0.cfg", 50, "extensions.0");
    return true;
  }

  static bool testLogging(){
    runAgentWithSchema("LogWriting.cfg", 50, "LogWriting");
    runAgentWithSchema("LogReading.cfg", 50, "LogReading");
    return true;
  }
  
  static bool testFileSearch(){
    setenv("TREX_START_DIR", "search_tests/a", 1);
    runAgentWithSchema("st.cfg", 50, "search_test.0");
    unsetenv("TREX_START_DIR");
    setenv("TREX_START_DIR", "search_tests/b", 1);
    runAgentWithSchema("st.cfg", 50, "search_test.1");
    unsetenv("TREX_START_DIR");
    setenv("TREX_START_DIR", "search_tests", 1);
    runAgentWithSchema("st.cfg", 50, "search_test.2");
    unsetenv("TREX_START_DIR");
    return true;
  }

  static bool testTestMonitor(){
    runAgentWithSchema("test_monitor.cfg", 50, "test_monitor");
    return true;
  }
  static bool bugFixes(){
    runAgentWithSchema("bug.0.cfg", 50, "bug.0");
    return true;
  }

  static bool testScalability(){
    runAgentWithSchema("synchronize.cfg", 50, "synchronize");
    return true;
  }

  static bool testSynch(){
    runAgentWithSchema("synch.4.cfg", 50, "synch.4");
    runAgentWithSchema("synch.3.cfg", 50, "synch.3");
    runAgentWithSchema("synch.1.cfg", 50, "synch.1");
    runAgentWithSchema("synch.0.cfg", 50, "synch.0");
    runAgentWithSchema("synch.2.cfg", 50, "synch.2");
    return true;
  }

  /**
   * @brief Tests a single deliberative reactor and an adapter.
   * The adapter will simply notify an observation for each request it receives. There is a single reactor for doing
   * all the rest of the planning. There should be no backtracking, just straightforward plan, dispatch, synchronize.
   */
  static bool testOneDeliberatorOneAdapter(){
    runAgentWithSchema("GamePlay.cfg", 50, "OneDeliberatorOneAdapter");
    return true;
  }

  /**
   * @brief Now swap out the adapter and replace it with a reactor for each player. Here we will plan games centrally but not schedule
   * times. The players receive requests for play and will schedule them. The players are set up with shorter lookahead windowns so that
   * there is a delay between receiving the request and posting a reply, thus showing disctributed scheduling and forcing the synchronization
   * to extend current values. This also shows how lookahead and latency data are used to dispatch and the expected output includes times
   * when requests are dispatched. Note the lag between request and notify events.
   */
  static bool testSimulation(){
    runAgentWithSchema("Simulator.cfg", 50, "Simulation");
    return true;
  }

  /**
   * @brief Tests a case of planner timeout
   * We make the number of steps per tick very small, so that the planner cannot be done in time. The problem is set up so that 2 attempts
   * can be made to plan the goals, but both will be pre-empted by timing out, so no requests are sent and no goals planned. Expect that the only
   * notifications logged are for the initial state.
   */
  static bool testSimulationWithPlannerTimeouts(){
    runAgentWithSchema("Simulator.cfg", 3, "SimulationWithPlannerTimeouts");
    return true;
  }

  /**
   * Test that we can forward plan with unit durations on a single deliberative reactor and no failures. This test will
   * ensure that we properly handle horizons and look aheads for this very tight case. The example is set up as the first single-reactor
   * case. The only timeline is an internal timeline and so it just generates notifications for the plan on each next tick.
   */
  static bool testOneStepAhead(){
    runAgentWithSchema("OneStepAhead.cfg", 30, "OneStepAhead");
    return true;
  }

  /**
   * @brief Set up 2 reactors planning the same timeline at different lookaheads.
   */
  static bool testSqueezeObserver(){
    runAgentWithSchema("SqueezeObserver.cfg", 15, "SqueezeObserver");
    return true;
  }

  /**
   * @brief Tests the recall functionality to force dispatch of recall commands based on detected plan failure.
   * Set up a 3 reactor system. 2 of them are deliberative reactors, one is an adapter. There are 3 timelines: a, b, c. The adapter manages
   * c. Reactor A owns a and uses c and b. Reactor B owns b. A will dispacth requests to B and C. B and C will dispatch observations to A. C will
   * generate observation values as a function of the current tick. This will conflict with planned values, causing re-planning each time.
   * 
   * Expectations in event log:
   * 1. Initial plan will dispatch 2 requests from A to B and 3 requests from A to C. More are dispatched to the latter because C has zero latency.
   * 2. Notifications are send from C to A and from B to A. Neither C nor B receive notifications.
   * 3. Expect the notification from C at tick 2 to force a recall. The event pattern should reflect that of the requests.
   * 4. No further requests or recalls will be produced. The goals for A will be rejected due to lack of time to plan.
   * 5. We set up the states in reactor A to give it a chance to dispatch requests and have server (i.e. B) include the requests in the plan. The recall
   *    on B will remove these tokens.
   */
  static bool testRecall(){
    runAgentWithSchema("Recall.cfg", 50, "Recall");
    return true;
  }

  /**
   * This test will see that when an internal timeline value becomes a fact, it is persisted unless a consistent and complete plan
   * produces a new fact to replace it. Planning will be resumed to try to recover the situation but this will always fail.
   */
  static bool testPersistence(){
    runAgentWithSchema("persistence.1.cfg", 20, "persistence.1");
    runAgentWithSchema("persistence.0.cfg", 20, "persistence.0");
    runAgentWithSchema("PersistedGoal.cfg", 20, "PersistedGoal");
    return true;
  }

  /**
   * Ensure execution fills out gaps in a single internal timeline
   */
  static bool testUndefinedSingleTimeline(){
    runAgentWithSchema("Undefined.SingleTimeline.cfg", 20, "Undefined.SingleTimeline");
    return true;
  }

  /**
   * Test that we are not setting undefined values permaturely during synchronization process
   */
  static bool testUndefinedDerived(){
    runAgentWithSchema("Undefined.Derived.cfg", 20, "Undefined.Derived");
    return true;
  }


  /**
   * Test an initial configuration that is infeasible and can force a goal to be rejected and a value
   * to be laid down in contradiction.
   */
  static bool testInconsistent(){
    runAgentWithSchema("Inconsistent.cfg", 20, "Inconsistent");
    return true;
  }

  /**
   * Test handling of actions
   */
  static bool testActions(){
    runAgentWithSchema("action.3.cfg", 50, "action.3");
    runAgentWithSchema("action.2.cfg", 50, "action.2");
    runAgentWithSchema("Action.0.cfg", 50, "Action.0");
    runAgentWithSchema("action.4.cfg", 50, "action.4");
    runAgentWithSchema("Action.1.cfg", 50, "Action.1");
    return true;
  }

  /**
   * Test missing expectations
   */
  static bool testRepair(){
    runAgentWithSchema("repair.0.cfg", 50, "repair.0");
    runAgentWithSchema("repair.1.cfg", 50, "repair.1");
    runAgentWithSchema("repair.3.cfg", 50, "repair.3");
    return true;
  }

  /**
   * Tests dispatching.
   */
  static bool testDispatch(){
    runAgentWithSchema("dispatch.2.cfg", 50, "dispatch.2");
    runAgentWithSchema("dispatch.0.cfg", 50, "dispatch.0");
    runAgentWithSchema("dispatch.1.cfg", 50, "dispatch.1");
    return true;
  }

  /**
   * Tests the OrienteeringSolver..
   */
  static bool OrienteeringSolver(){
    //runAgentWithSchema("orienteering.5.cfg", 50, "orienteering.5");
    runAgentWithSchema("orienteering.0.cfg", 50, "orienteering.0");
    runAgentWithSchema("orienteering.1.cfg", 50, "orienteering.1");
    runAgentWithSchema("orienteering.2.cfg", 50, "orienteering.2");
    runAgentWithSchema("orienteering.3.cfg", 50, "orienteering.3");
    runAgentWithSchema("orienteering.4.cfg", 50, "orienteering.4");
    return true;
  }
};

class AgentTests {
public:
  static bool test(){
    runTest(testRealTimeClock);
    runTest(testForeverConfiguration);
    runTest(testTimelimitOverride);
    return true;
  }

private:
  static bool testRealTimeClock(){
    RealTimeClock clk(1.5);
    clk.start();

    // Sleep for 2 seconds and ensure we are at tick 1
    Clock::sleep(2);
    assertTrue(clk.getNextTick() == 1);

    // Sleep for 0.1 seconds and ensure we are still at tick 1
    Clock::sleep(0.1);
    assertTrue(clk.getNextTick() == 1);

    // Now sleep for 0.5 seconds and we should be at tick 2
    Clock::sleep(1.0);
    assertTrue(clk.getNextTick() == 2);

    TICK startTick = clk.getNextTick();
    TICK counter(0);
    while(counter < 5){
      clk.sleep(1.5);
      counter++;
    }

    TICK delta = clk.getNextTick() - startTick;

    assertTrue(counter == delta);

    return true;
  }

  static bool testForeverConfiguration(){
    PseudoClock clock(0.0, 1);
    TiXmlElement* root = initXml("Forever.cfg");

    Agent::initialize(*root, clock);

    assertTrue(Agent::instance()->getFinalTick() == Agent::forever());

    Agent::reset();
  
    delete root;
    return true;
  }

  static bool testTimelimitOverride(){
    PseudoClock clock(0.0, 1);
    TiXmlElement* root = initXml("Forever.cfg");

    Agent::initialize(*root, clock, 10);

    assertTrue(Agent::instance()->getFinalTick() == (TICK) 10);

    Agent::reset();
  
    delete root;
    return true;
  }
};

int main() {
  setenv("TREX_PATH", "./orienteering:./personal_robots", 1);
  runTestSuite(GamePlayTests::test);
  runTestSuite(AgentTests::test);

  return 0;
}
