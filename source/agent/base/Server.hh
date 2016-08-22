#ifndef H_Server
#define H_Server

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

namespace TREX {

  /**
   * @brief Abstract interface for handling goals. 
   */
  class Server {
  public:

    /**
     * @brief Commands the server to handle a goal. This method should apply all the constraints attached to the
     * given token that it can reasonably support.
     * @param goal A token from the client database which is to be accomplished.
     * @return true if the request can be received, false if it cannot be received and should be resent
     */
    virtual bool request(const TokenId& goal) = 0;

    /**
     * @brief Commands the server to discard a goal.
     * @param goal A token from the client database which is to be recalled.
     */
    virtual void recall(const TokenId& goal) = 0;


    /**
     * @brief Retrieve the latency it takes to respond. This is used in planning when to dispatch goals. It is a lower bound on dispatch window.
     */
    virtual TICK getLatency() const = 0;

    /**
     * @brief Retrieve the look-ahead window. This is used in deciding how far ahead to commit goals. It is an upper bound on the dispatch window.
     */
    virtual TICK getLookAhead() const = 0;

    virtual ~Server(){}
  };
}

#endif
