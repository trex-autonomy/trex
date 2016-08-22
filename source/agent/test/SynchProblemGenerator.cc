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
 * @author Conor McGann
 * Implements experiments explore scalability of different control topologies and
 * with different partition structure and internal connectivity per reactor.
 */
#include "Agent.hh"
#include "LogManager.hh"
#include "Debug.hh"
#include "Nddl.hh"
#include <signal.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <math.h>

using namespace TREX;
using namespace EUROPA;

// Global pool of targets built up to create s.jam
std::vector<std::string> nddlTargets;
std::vector<std::string> agentTargets;

/**
 * @brief Class contains parameters for a problem instance
 */
class Problem { 
public:
  Problem(unsigned int w, 
	  unsigned int d, 
	  unsigned int g,
	  unsigned int _I, 
	  unsigned int _E, 
	  unsigned int _C, 
	  unsigned int h)
	  : width(w), depth(d), growth(g), I(_I), E(_E), C(_C), horizon(h), latency(0), lookAhead(0), B(0) {}

  Problem(unsigned int w, 
	  unsigned int d, 
	  unsigned int g,
	  unsigned int _I, 
	  unsigned int _E, 
	  unsigned int _C, 
	  unsigned int h,
	  unsigned int l,
	  unsigned int p,
	  unsigned int _B)
	  : width(w), depth(d), growth(g), I(_I), E(_E), C(_C), horizon(h), latency(l), lookAhead(p), B(_B) {}

  std::string toString() const {
    std::stringstream ss;
    ss << "s." << width << "." << depth << "." << growth << "." << I << "." << E << "." << C << "." << 
      horizon << "." <<latency << "." << lookAhead << "." << B;

    return ss.str();
  }

  unsigned int width;
  unsigned int depth;
  unsigned int growth;
  unsigned int I;
  unsigned int E;
  unsigned int C;
  unsigned int horizon;
  unsigned int latency;
  unsigned int lookAhead;
  unsigned int B;
};

/**
 * @brief Generates a reactor input file in NDDL
 */
void makeReactorInputFile(const std::string& reactorName,
			  const std::vector<int>& path,
			  unsigned int pos,
			  const Problem& p){
  std::string reactorFileName(reactorName);

  nddlTargets.push_back(reactorFileName);

  reactorFileName += ".nddl";
  std::ofstream o(reactorFileName.c_str());
  o << "#include \"GamePlay.nddl\";" << std::endl;

  std::string varPrefix;
  {
    std::stringstream ss;
    ss << "n";
    for(unsigned int i=0;i<path.size(); i++)
      ss << "_" << path[i];
    ss << "_" << pos;
    varPrefix += ss.str();
  }

  unsigned int id(0);

  // Generate Internal Variable Declarations
  o << "// Internal Timelines " << std::endl;
  int duration = pow(2, p.depth - path.size());
  int startTime = (int((p.horizon - duration)/duration)) * duration;
  for(unsigned int i=0;i<p.I; i++){
    o << "ScalabilityTestTimeline " << varPrefix << "_" << i << "= new ScalabilityTestTimeline(" << id++ << ", " << 
      duration << ", " << (duration + p.B) << ", " << p.C << ", Mode.Internal);" << std::endl;

    // Generate goals
    o << "rejectable(" << varPrefix << "_" << i << ".Holds g_" << i << ");" << std::endl;
    o << "g_" << i << ".start = "  << startTime << ";" << std::endl;
  }

  // Generate External Variable Declarations if appropriate
  if(p.depth - path.size() > 0){
    o << "// External Timelines " << std::endl;
    for(unsigned int i=0; i<p.growth;i++){
      std::stringstream ss;
      ss << "n";
      for(unsigned int j=0;j<path.size(); j++)
	ss << "_" << path[j];
      ss << "_" << pos << "_" << i;
      std::string externalVarPrefix = ss.str();
      for(unsigned int j = 0; j < p.E; j++){
	o << "ScalabilityTestTimeline " << externalVarPrefix << "_" << j << "= new ScalabilityTestTimeline(" << id++ << ", " << pow(2, p.depth - path.size() - 1) << ", " << p.C << ", Mode.External);" << std::endl;
      }
    }
  }

  o << "close();" << std::endl;
  o.close();
}

/**
 * @brief tail recursive iterator to visit all the reactors in the control structure.
 */
void visitReactor(const std::string& agentName, 
		  const std::vector<int>& path, 
		  unsigned int pos, 
		  const Problem& p,
		  std::vector<std::string>& reactors){
  std::string reactorFileName = agentName;
  reactorFileName += ".";

  // The id for a reactor will be a dot delimited path
  std::stringstream ss;
  ss << "r.";
  for(unsigned int i=0;i<path.size();i++)
    ss << path[i] << ".";

  ss << pos;
  reactors.push_back(ss.str());
  reactorFileName += ss.str();

  makeReactorInputFile(reactorFileName, path, pos, p);

  if(path.size() < p.depth){
    std::vector<int> newPath(path);
    newPath.push_back(pos);
    for(unsigned int i=0;i<p.growth;i++)
      visitReactor(agentName, newPath, i, p, reactors);
  }
}

/**
 * @brief Generates a configuration file for a problem
 */
void makeConfigFile(const std::string& agentName, const std::vector<std::string>& reactors, const Problem& p){
  std::string cfgFileName(agentName);
  cfgFileName += ".cfg";
  agentTargets.push_back(cfgFileName);

  std::ofstream o(cfgFileName.c_str());
  // First output the preamble:
  o << "<!--  Autmatically Generated Test Case for Scalability Testing: s.width.depth.growth.I.E.C.l.p.cfg -->" << std::endl;
  o << "<Agent name=\"" << agentName << "\" finalTick=\"" << p.horizon << "\">" << std::endl;
    
  for(std::vector<std::string>::const_iterator it = reactors.begin(); it != reactors.end(); ++it){
    o << "  <TeleoReactor name=\"" << *it << "\" component=\"DeliberativeReactor\" " << 
      "lookAhead=\"" << p.lookAhead << "\" latency=\"" << p.latency << "\"  solverConfig=\"exp.solver.cfg\"/>" << std::endl;
  }

  o << "</Agent>";
  o.close();
}

void makeJamFile(){
  std::ofstream o("s.jam");
  for(std::vector<std::string>::const_iterator it = nddlTargets.begin(); it != nddlTargets.end(); ++it){
    const std::string& s = *it;
    o << "Depends scalability-tests : " << s<< ".xml ;" << std::endl;
    o << "NddlParser " << s << ".xml : " << s << ".nddl ;" << std::endl;
  }

  // Now generate targets for execution
  o << std::endl << "# Executables " << std::endl;
  o << "TestCases = " << std::endl;
  for(std::vector<std::string>::const_iterator it = agentTargets.begin(); it != agentTargets.end(); ++it)
    o << "       " << *it << std::endl;
  o << ";" << std::endl << std::endl;

  o << "for test in $(TestCases) {" << std::endl;
  o << "	RunModuleMain run-$(test) : agent : $(test) ;" << std::endl;
  o << "	Depends exec-scalability-tests : run-$(test) ;" << std::endl; 
  o << "	Depends run-$(test) : scalability-tests ;" << std::endl;
  o << "}";
  o.close();
}

/**
 * @brief Main program to generate a synchronization scalability test
 * @param width The number of reactors at the top level > 0
 * @param depth The number of layers of reactors. > 0
 * @param growth The growth rate in reactors from one layer to another.
 * @param I The number of internal timelines per reactor
 * @param E the number of external timelines per reactor, not in the base.
 * @param C the number of connected timelines internal in a reactor
 */
void makeProblemInstance(const Problem& p){
  std::string agentName = p.toString();
  std::cout << "Generating Test: " << agentName << std:: endl;

  std::vector<int> path;
  std::vector<std::string> reactors;

  for(unsigned int i=0;i<p.width;i++)
    visitReactor(agentName, path, i, p, reactors);

  makeConfigFile(agentName, reactors, p);
}


/**
 * @brief Run a problem, and retrieve the results as a data set
 */
std::vector< std::pair<timeval, timeval> > runProblemInstance(const Problem& p) {
  std::ofstream dbgFile("Debug.log");
  LogManager::instance();
  std::string configStr = p.toString() + std::string(".cfg");
  TiXmlElement* root = LogManager::initXml(configStr.c_str());
  DebugMessage::setStream(dbgFile);

  PseudoClock clk(0.0, 10000);
  Agent::initialize(*root, clk);

  try{
    debugMsg("ALWAYS", "Executing the agent");
    Agent::instance()->run();
  }
  catch(void*){
    debugMsg("ALWAYS", "Caught unexpected exception.");
  }

  std::vector< std::pair<timeval, timeval> > results = Agent::instance()->getMonitor().getData();

  // Cleanup
  Agent::reset();
  Agent::cleanupLog();
  delete root;
  return results;
}

/**
 * @brief Evaluate increase of synchronization with greater I. C = 0
 */
void experiment1(bool generateProblem){
  std::ofstream of("s.exp.1.stats");
  for(unsigned int I = 1; I<=30;I++){
    for(unsigned int C = 0; C < I; C++){
      if(generateProblem)
	makeProblemInstance(Problem(1, 0, 0, I, 0, C, 50));
      else {
	std::vector< std::pair<timeval, timeval> > results = runProblemInstance(Problem(1, 0, 0, I, 0, C, 50));

	// Take the average over 50 ticks.
	double sumSynchTime = 0;
	for(unsigned int i = 0; i<results.size(); i++){
	  const timeval& v = results[i].first;
	  sumSynchTime += v.tv_usec / 1000000.0;
	}

	of << I << "," << C << "," << sumSynchTime / results.size() << std::endl;
      }
    }
  }

  of.close();
}

/**
 * @brief Now we look at the overhead of partitioning from the standpoint of synchronization. This does not
 * include costs associated with dispatching and receiving goals, nor does it address questions pertaining
 * to recalls due to decoupling state variables that are actually coupled. I expect this overhead to be linear
 * in the size of E. This experiment uses a 2 reactor setup with 20 internal timelines each. The results are clearly
 * based on E/I as the overhead in the second reactor.
 */
void experiment2(bool generateProblem){
  std::vector<double> outputList;
  for(unsigned int E = 0; E < 20; E++){
    if(generateProblem)
      makeProblemInstance(Problem(1, 1, 1, 20, E, 0, 100));
    else {
      std::vector< std::pair<timeval, timeval> > results = runProblemInstance(Problem(1, 1, 1, 20, E, 0, 100));
      double synchSum(0.0);
      for(unsigned int i = 0; i<results.size(); i++)
	synchSum += results[i].first.tv_usec;
      outputList.push_back(synchSum / (results.size() * 1000000.0));
    }
  }

  if(!outputList.empty()){
    std::ofstream of("s.exp.2.stats");
    for(unsigned int i = 0; i < outputList.size(); ++i)
      of << i << ", " << outputList[i] << std::endl;
    of.close();
  }
}

/**
 * @brief Now we examine the role of partitioning on planning. The expectation is that if a large problem
 * can be sub-divided, say because it is weakly coupled or even completely independent, then it will be more
 * efficient to split it up. This is not to make search any faster, which can also of course be an issue. Rather
 * it is just based on the overhead per step of problem solving due to the quadratic complexity of operating on
 * a shared plan structure. The experiment will have N timelines in total, with a horizon of H, and M reactors.
 * Each reactor will have N/M timelines.
 * 
 * Sure enough, substantial speed ups are seen. This is the case despite the fact that we have not changed the search
 * at all, or reduced any part of the plan. We see that some reactors are done really fast, offering incremental delivery,
 * and that the total time for all reactors is also reduced. This is the case even though the timepoints are grounded
 * and we do not backrack. In cases where timepoints are not grounded, there is more propagation. In cases where
 * we backtrack, the re-propagation is very expensive. These should lead to much greater gains by partitioning.
 */
void experiment3(bool generateProblem){
  const unsigned int N = 120;
  std::ofstream of("s.exp.3.stats");
  for (unsigned int m = 1; m<=N; m++){
    // Only pick numbers that work evenly
    if( (N % m) > 0)
      continue;

    int I = N / m;

    if(generateProblem)
      makeProblemInstance(Problem(m, 0, 1, I, 0, 0, 10, 0, 10, 0));
    else {
      double sumSynch(0.0), sumDeliberation(0.0);
      std::vector< std::pair<timeval, timeval> > results = runProblemInstance(Problem(m, 0, 1, I, 0, 0, 10, 0, 10, 0));
      for(unsigned int i = 0; i<results.size(); i++){
	sumSynch += results[i].first.tv_sec + results[i].first.tv_usec / 1000000.0;
	sumDeliberation += results[i].second.tv_sec + results[i].second.tv_usec / 1000000.0;
      }

      of << I << "," << sumSynch << "," << sumDeliberation << std::endl;
    }
  }

  of.close();
}

/**
 * @brief Test case with a multi-reactor hierarchical control structure. This example will demonstrate that the cost
 * is based on what changes. We should see results where the cpu load is dividied into buckets as a histogram
 * reflecting the costs as timelines in different reactors change concurrently. A single external timeline
 * is added per reactor
 */
void experiment4(bool generateProblem){
  if(generateProblem)
    makeProblemInstance(Problem(1, 3, 1, 10, 1, 0, 200));
  else {
    std::ofstream of("s.exp.4.stats");
    std::vector< std::pair<timeval, timeval> > results = runProblemInstance(Problem(1, 3, 1, 10, 1, 0, 200));

    // Output in seconds.
    for(unsigned int i = 0; i<results.size(); i++)
      of << i << ", " << results[i].first.tv_usec / 1000000.0 << std::endl;

    of.close();
  }
}


int main(int argc, char **argv) {
  initTREX();
  NDDL::loadSchema();

  bool generateProblem = (argc == 1);
  //experiment1(generateProblem);
  //experiment2(generateProblem);
  //experiment3(generateProblem);
  experiment4(generateProblem);

  if(generateProblem)
    makeJamFile();

  return 0;
}
