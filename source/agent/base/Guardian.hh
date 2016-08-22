/* -*- C++ -*-
 * $Id$
 */
/** @file "Guardian.hh"
 * @brief defintion of Guardian class
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef _GUARDIAN_HH
#define _GUARDIAN_HH

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

  /** @brief Critical section facility.
   *
   * This class is used to ensure the locking/unlocking of a
   * mutex like class in a given context. The locking is done
   * during construction and the unlocking during destruction
   * so we ensure that a locked instance will be unlocked when
   * exiting current context even if an exception is thrown.
   *
   * @param Mtx The type of the managedd mutex like class.
   *
   * @author Frederic Py <fpy@mbari.org>
   */
  template<class Mtx>
  class Guardian {
  public:
    /** @brief Constructor.
     *
     * @param guarded A mutex like instance.
     *
     * Lock @e guarded
     *
     * @post @e guarded is locked 
     */
    Guardian(Mtx &guarded)
      :m_guarded(guarded) {
      m_guarded.lock();
    }

    /** @brief Destructor.
     *
     * Unlock the guarded mutex like instance.
     */
    ~Guardian() {
      m_guarded.unlock();
    }  
  private:
    /** brief Guarded mutex like instance. */
    Mtx &m_guarded;

    // Following functions have no code in purpose
    Guardian(Guardian const &);
    void operator= (Guardian const &);
  }; // TREX::Guardian

} // TREX

#endif // _GUARDIAN_HH
