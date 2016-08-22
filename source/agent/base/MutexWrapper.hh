/* -*- C++ -*-
 * $Id$
 */
/** @file "MutexWrapper.hh"
 * @brief Definition of the Mutex class
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef _MUTEXWRAPPER_HH
#define _MUTEXWRAPPER_HH

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

#include <pthread.h>

#include "ErrnoExcept.hh"

namespace TREX {

  /** @brief simple mutex implementation.
   *
   * This class provide a simple implementation for mutex.
   *
   * @author Frederic Py <fpy@mbari.org>
   */
  class Mutex {
  public:
    /** @brief Constructor.
     *
     * Create a new mutex instance.
     *
     * @post The mutex is unlocked.
     *
     * @throw ErrnoExcept error during mutex resource creation.
     */
    Mutex();
    /** @brief Destructor.
     *
     * @throw ErrnoExcept error dur
     */
    ~Mutex();
    
    /** @brief mutex lock
     *
     * Lock the current instance. If it is already locked
     * it wait until we can lock it.
     *
     * @post The mutex is locked.
     *
     * @throw ErrnoExcept Error during operation.
     * 
     * @sa Mutex::trylock()
     * @sa Mutex::unlock()
     */
    void lock();
    /** @brief Non blocking mutex lock.
     *
     * Tries to lock the mutex. If the mutex is already locked this method just return immediatly
     * without trying to lock it anymore.
     *
     * @retval true if the mutex was successfully locked
     * @retval false if the mutex is already locked elsewhere.
     *
     * @throw ErrnoExcept porblemn during the operation.
     *
     * @sa Mutex::lock()
     * @sa Mutex::unlock()
     * @sa Mutex::islocked()
     */
    bool trylock();
    
    /** @brief Mutex unlock.
     *
     * This method unlock the mutex.
     *
     * @post mutex is unlocked
     *
     * @throw ErrnoExcept problemn during operation.
     *
     * @sa Mutex::lock()
     * @sa Mutex::trylock()
     */
    void unlock();
    
    /** @brief Check if mutex ius locked
     *
     * This method test if this instance is locked.
     *
     * @retval true The mutex is locked
     * @retval false else
     *
     * @throw ErrnoExcept problem during operation.
     *
     * @sa Mutex::trylock()
     * @sa Mutex::lock()
     * @sa Mutex::unlock()
     */
    bool isLocked() const;
    
  private:
    /** @brief mutex id */
    mutable pthread_mutex_t m_mutexId;

    /** @brief Non blocking lock check.
     *
     * This method is internally used by isLocked() and trylock() methods to
     * make a non blocking lock attempt.
     *
     * @param test Is it just a test. If this argument is true then the function
     * will unlock the mutex just ater a succesfull lock operation.
     * @param from An identifier to know the "real" origin of a possible  exception.
     *
     * @retval true The lock was successfull
     * @retval false The mutex is already locked
     *
     * @throw ErrnoExcept problem during the operation.
     *
     * @sa Mutex::trylock()
     * @sa Mutex::isLocked()
     */
    bool checkLock(bool test, std::string const &from) const;

    // Following functions are not implemented in purpose
    Mutex(Mutex const &);
    void operator= (Mutex const &);
  }; // TREX::Mutex

} // TREX

#endif // _MUTEX_HH
