/* -*- C++ -*-
 * $Id$
 */
/** @file "SharedVar.hh"
 * @brief Definition of SharedVar template class
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef _SHAREDVAR_HH
#define _SHAREDVAR_HH

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

#include "Thread.hh"
#include "Guardian.hh"

namespace TREX {

  /** @brief Bad access exception
   * @relates SharedVar
   *
   * This exception is thrown by SharedVar when one tries
   * to access a SharedVar it ihas not locked.
   *
   * @author Frederic Py <fpy@mbari.org>
   *
   * @note The management of this error is more a coding issue.
   * It may be fine to be able to disable it when we are sure
   * that this condition is respected. (?)
   */
  class AccessExcept
    :public ThreadExcept {
  public:
    /** @brief Constructor.
     *
     * @param message Error message.
     */
    AccessExcept(std::string const &message) throw()
      :ThreadExcept(message) {}
    /** @brief Destructor. */
    ~AccessExcept() throw() {}
    
  }; // AccessExcept

  /** @brief Shared variable
   *
   * This class implements a simple utility to manage a varaible which is manipulated
   * by multiple threads. 
   *
   * @param Ty type of the subjacent variable
   *
   * @pre Ty must support copy construction
   * @pre Ty must be default constructible
   *
   * @author Frederic Py <fpy@mbari.org>
   */
  template<class Ty>
  class SharedVar {
  public:
    /** @brief Constructor.
     *
     * @param val A value
     *
     * Create a new instance fixing its value to @e val.
     *
     */
    SharedVar(Ty const &val=Ty())
      :m_owner(0), m_var(val) {}
    /** @brief Destructor */
    ~SharedVar() {}
    
    /** @brief Locking method.
     *
     * This function locks the mutex attached to this variable.
     *
     * @post The varaiable is owned by current thread.
     *
     * @throw ErrnoExcept An error occured during operation
     *
     * @sa SharedVar::unlock()
     */
    void lock() const {
      m_mtx.lock();
      m_owner = Thread::current();
    }
    /** @brief Unlocking method.
     *
     * Unlock the mutex attached to this varaiable.
     *
     * @pre The varaiable is owned by current thread.
     * @post The varaiable is not owned by current thread anymore.
     *
     * @throw AccessExcept Varaiable not owned by current thread
     * @throw ErrnoExcept error during operation.
     *
     * @sa SharedVar::lock()
     * @sa SharedVar::ownIt()
     */
    void unlock() const {
      if( !ownIt() )
	throw AccessExcept("Try to unlock a shared var I don't own.");
      m_owner = 0;
      m_mtx.unlock();
    }

    /** @brief Owning test
     * 
     * Test is this varaiable is owned by current thread. A variable owned by a
     * thread means that it was locked by this one.
     *
     * @retval true if it is owned by this thread
     * @retval false else
     *
     * @sa SharedVar::lock()
     */
    bool ownIt() const {
      return m_mtx.isLocked() && Thread::current()==m_owner;
    }

    /** @brief Copy operator.
     *
     * @param value A value 
     * 
     * This method assign a new @e value to the shared variable.
     *
     * @return @e value
     *
     * @note This function may need to lock/unlock the instance if it is
     * not already owned by current thread. As a consequence it may block
     * current thread during the lock call.
     *
     * @throw ErrnoExcept error during operation.
     */
    Ty const &operator=(Ty const &value) {
      if( !ownIt() ) {
	Guardian< SharedVar<Ty> > guard(*this);
	// CS
	m_var = value;
	// End CS
      } else
	m_var = value;
      return value;
    }
    
    /** @{
     * @brief Dereferencing operator.
     *
     * Give access to variable
     *
     * @pre Variable is owned by current thread. 
     *
     * @return A reference to the variable
     *
     * @throw AccessExcept Variable is not owned by current thread.
     * @throw ErrnoExcept A error occured while checking for owner.
     *
     * @sa SharedVar::lock()
     * @sa SharedVar::operator->()
     */
    Ty &operator* () {
      if( !ownIt() )
	throw AccessExcept("Cannot access a shared var which I don't own.");
      return m_var;
    }

    Ty const &operator* () const {
      if( !ownIt() )
	throw AccessExcept("Cannot access a shared var which I don't own.");
      return m_var;
    }
    /** @} */
    
    /** @{
     * @brief Acces operator.
     * 
     * Give access to variable and more specifically to its attributes.
     *
     * @pre Variable is owned by current thread. 
     *
     * @return A reference to the variable
     *
     * @throw AccessExcept Variable is not owned by current thread.
     * @throw ErrnoExcept A error occured while checking for owner.
     *
     * @sa SharedVar::lock()
     * @sa SharedVar::operator* ()
     */
    Ty *operator->() {
      return &operator* ();
    }

    Ty const *operator->() const {
      return &operator* ();
    }
    /** @} */

  private:
    /** @brief Mutex to lock/unlock varaiable */ 
    mutable Mutex     m_mtx;
    /** @brief owner of the variable.
     *
     * This asttribute is used in conjunction with m_mtx to determine
     * what thread owns the variable.
     */
    mutable pthread_t m_owner; 

    /** @brief Variable value */
    Ty m_var;
    
    // Following functions have no code in purpose
    void operator= (SharedVar const &);

  }; // SharedVar<>

} // TREX

#endif // _SHAREDVAR_HH
