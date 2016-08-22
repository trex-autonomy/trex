/* -*- C++ -*-
 * $Id$
 */
/** @file "ObserverReactor.hh"
 * @brief Provide declarations for the ObserverReactor
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef H_ObserverReactor
#define H_ObserverReactor

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

# include <set>

# include "TeleoReactor.hh"

namespace TREX {
  
  class ObserverReactor: public TeleoReactor {
  public:
    ObserverReactor(LabelStr const &agentName, TiXmlElement const &configData);
    
    ~ObserverReactor();

    void queryTimelineModes(std::list<LabelStr> &externals, 
			    std::list<LabelStr> &internals);

    void handleInit(TICK initialTick, std::map<double, ServerId> const &serversByTimeline,
		    ObserverId const &observer);

    void notify(Observation const &obs);
    bool handleRequest(const TokenId& goal);
    void handleRecall(const TokenId& goal);

  protected:
    bool hasWork() { return false; }
    void resume() {}
    void archive() {}

    void handleTickStart() {}
    void handleTickEnd() {}

    static void getTimelines(std::map<LabelStr, ServerId> &resuls, 
			     TiXmlElement const &configData);
    static TiXmlElement const &externalConfig(TiXmlElement const &sourceConfig);

  private:
    std::map<LabelStr, ServerId> m_externals;

    std::string nameString() const; 

  }; // TREX::ObserverReactor

} // TREX

#endif // H_ObserverReactor
