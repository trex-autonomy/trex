#ifndef H_GreedyOpenConditionManager
#define H_GreedyOpenConditionManager

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
#include "Token.hh"
#include "TokenVariable.hh"

/**
 * @brief The goal manager.
 */

using namespace EUROPA::SOLVERS;

namespace TREX {

  /**
   * @brief Same as parent class but we over-ride the comparator to use the earlier of the 2
   */
  class GreedyOpenConditionManager: public OpenConditionManager {
  public:

    /**
     * @brief Uses standard constructor
     */
    GreedyOpenConditionManager(const TiXmlElement& configData) : OpenConditionManager(configData){}

  protected:
    /**
     * @brief Comparator first prefers according to start time, and then by key
     */
    bool betterThan(const EntityId& a, const EntityId& b, LabelStr& explanation){
      if(b.isNoId()){
        explanation = "b.isNoId";
	return true;
      }

      TokenId tokenA(a);
      TokenId tokenB(b);

      explanation = "earlier";
      return tokenA->start()->lastDomain().getLowerBound() < tokenB->start()->lastDomain().getLowerBound();
    }
  };
}

#endif
