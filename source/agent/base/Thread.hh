/* -*- C++ -*-
 * $Id$
 */
/** @file "Thread.hh"
 * @brief Defintion of the Thread abstract class 
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef _THREAD_HH
#define _THREAD_HH

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

#include "MutexWrapper.hh"

namespace TREX {

  class ThreadImpl;

  /** @brief Thread related exception.
   *
   * This exception may be throwned by Thread when a problem occurs.
   *
   * @author Frederic Py <fpy@mbari.org>
   */
  class ThreadExcept : public std::runtime_error {
  public:
    /** @brief Constructor.
     *
     * @param message Error message.
     */
    ThreadExcept(std::string const &message) throw()
      :std::runtime_error(message) {}
    
    /** @brief destructor. */
    virtual ~ThreadExcept() throw() {}
    
  }; // TREX::ThreadExcept

  /** @brief Thread management class.
   *
   * This class offers a jabva like interface to pthread library.
   *
   * @author Frederic Py <fpy@mbari.org>
   */
  class Thread {
  public:
    /** @brief Thread scope.
     *
     * This type is used to set/know the scope of a Thread.
     */
    enum Scope {
      systemScope = PTHREAD_SCOPE_SYSTEM, //!< System scope
      processScope = PTHREAD_SCOPE_PROCESS //!< Process scope 
    }; // Thread::Scope

    /** @brief Thread detach state.
     *
     * This type is used to set/know the detach state of a Thread.
     */
    enum DetachState {
      joinable = PTHREAD_CREATE_JOINABLE, //!< Thread can be joined
      detached = PTHREAD_CREATE_DETACHED //!< Thread is detached
    }; // Thread::DetachState

    /** @brief Constructor.
     *
     * Create a new thread.
     *
     * @note the thread is not started during construction.
     *
     * @throw ErrnoExcept error during initialization.
     */
    Thread();
    /** @brief Destructor.
     *
     * Destory current Thread.
     *
     * @throw ErrnoExcept Error during thread destruction.
     *
     * @note if the Thread was running it will be stopped using Thread::stop()
     *
     * @sa Thread::stop()
     */
    virtual ~Thread();

    /** @brief Set thread scope value.
     *
     * @param val A scope value
     *
     * Change the scope attribute value to this thread.
     *
     * @pre The Thread is not runnning yet.
     *
     * @throw ErrnoExcept Error while trying to change thread scope.
     */
    void setScope(Scope val);
    /** @brief Set thread detach state.
     *
     * @param val A detach state value
     *
     * Change the detach state attribute value to this thread.
     *
     * @pre The Thread is not runnning yet.
     *
     * @throw ErrnoExcept Error while trying to change thread detach state.
     */
    void setDetachState(DetachState val);
    /** @brief Set thread stack size.
     *
     * @param size A stack size
     *
     * Change the stack size attribute value to this thread.
     *
     * @pre The Thread is not runnning yet.
     *
     * @throw ErrnoExcept Error while trying to change thread stacjk size.
     */
    void setStackSize(size_t size);

    /** @brief Get thread scope.
     *
     * @return current instance thread scope.
     *
     * @throw ErrnoExcept Error while trying to get thread scope
     */
    Scope getScope() const;
    /** @brief Get thread detach state.
     *
     * @return current instance detach state.
     *
     * @throw ErrnoExcept Error while trying to get thread detach state.
     */
    DetachState getDetachState() const;
    /** @brief Get thread stack  size.
     *
     * @return current instance satck size.
     *
     * @throw ErrnoExcept Error while trying to get thread stack size.
     */
    size_t getStackSize() const;

    /** @brief Starts thread.
     *
     * This method launch the thread on the system.
     *
     * @pre The threasd was not started yet
     * @post The thread is running
     *
     * @throw ThreadExcept Thread was already started.
     * @throw ErrnoExcept Error while trying to satrt the thread
     */
    void start();
    /** @brief Stops the thread.
     *
     * This method stops current thread execution.
     *
     * @pre Thread is running.
     *
     * @warning This method is a very rough way to end a running thread. It is preferable to use some
     * kind of boolean flag to tell to the Thread to terminate properly.
     */
    void stop();
    /** @brief Wait for thread end.
     *
     * This method is used to join a running thread to the current thread.
     *
     * @pre Thread detach state is Thread::joinable
     * @pref Thread is running
     *
     * @return Thre returned value by the Thread.
     *
     * @throw ErrnoExcept A problem occured during operation.
     */
    void *join();

    /** @brief Active thread id
     *
     * @return The id for the thread which called this method.
     */
    static pthread_t current(); 

    /** @brief exit function.
     *
     * @param ret A return value.
     *
     * Terminate the caller thread.
     */
    static void exit(void *ret =NULL);
    /** @brief Thread yielding.
     *
     * Indicates to the scheduler that the caller can yield to let other threads execute themselves.
     */
    static void yield(); 

  protected:
    /** @brief Thread main code
     *
     * This method will be executed in the newly satrted thread.
     */
    virtual void *run() =0;

  private:
    /** @brief active thread real implementation.
     *
     * This attribute points to a ThreadImpl instance when the Thread
     * is currently running. This is where sits most of the real
     * implementation of Thread.
     */
    ThreadImpl *m_active;
    /** @brief Thread attributes. */
    pthread_attr_t m_attr;
    
    friend class ThreadImpl; 
  }; // TREX::Thread

} // TREX

#endif // _THREAD_HH
