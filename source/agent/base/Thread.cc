
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

/* -*- C++ -*-
 * $Id$
 */
/** @file "Thread.cc"
 *
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#include <sched.h>

#include "Thread.hh"

namespace TREX {

  /** @brief Thread implementation.
   *
   * This class provide the low level implementation of
   * Thread that can be connnected to pthread lib.
   */
  class ThreadImpl {
  public:
    /** @brief Constructor */
    ThreadImpl(Thread *t);
    /** @brief Destructor */ 
    ~ThreadImpl();

    /** @brief Thread start */
    void start();
    /** @brief Wait for thread end */
    void *join();

    /** @brief Thread main code runner.
     *
     * This method executes the Thread main code.
     *
     * @sa Thread::run() 
     */
    void *run();
  private:

    Thread *m_thread;
    pthread_t m_id;
  }; // TREX::ThreadImpl

} // TREX

extern "C" {

  /** @brief Thread interface to pthread
   *
   * This method is the interface passed as argument to pthread_create
   * and is used as a C interface to the C++ implementation.
   */
  void *thread_launch(void *obj) {
    TREX::ThreadImpl *impl = (TREX::ThreadImpl *)obj;
    return impl->run();
  }

} // extern "C"

using namespace TREX;

/*
 * class Thread
 */ 
// Statics :

pthread_t Thread::current() {
  return pthread_self();
}

void Thread::exit(void *ret) {
  ::pthread_exit(ret);
}

void Thread::yield() {
  ::sched_yield();
}

// Structors :

Thread::Thread() 
  :m_active(NULL) {
  if( 0!=::pthread_attr_init(&m_attr) )
    throw ErrnoExcept("Thread::Thread");
}

Thread::~Thread() {
  stop();
  if( 0!=::pthread_attr_destroy(&m_attr) )
    throw ErrnoExcept("Thread::~Thread");
}

// Modifiers :

void Thread::setScope(Thread::Scope val) {
  if( 0!=::pthread_attr_setscope(&m_attr, val) )
    throw ErrnoExcept("Thread::setScope");
}

void Thread::setDetachState(Thread::DetachState val) {
  if( 0!=::pthread_attr_setdetachstate(&m_attr, val) )
    throw ErrnoExcept("Thread::setDetachState");
}

void Thread::setStackSize(size_t size) {
  if( 0!=::pthread_attr_setstacksize(&m_attr, size) )
    throw ErrnoExcept("Thread::setStackSize");
}

// Manipulators :

void Thread::start() {
  if( NULL!=m_active )
    throw ThreadExcept("Thread::start : Cannot satrt a thread whil active."); 
  m_active = new ThreadImpl(this);
  m_active->start();
}

void Thread::stop() {
  if( NULL!=m_active ) {
    delete m_active;
  }
}

void *Thread::join() {
  if( NULL==m_active )
    throw ThreadExcept("Thread::join : This thread is not active."); 
  return m_active->join();
}

// Observers :

Thread::Scope Thread::getScope() const {
  int res;
  ::pthread_attr_getscope(const_cast<pthread_attr_t *>(&m_attr), &res);
  return (Scope)res;
}

Thread::DetachState Thread::getDetachState() const {
  int res;
  ::pthread_attr_getdetachstate(const_cast<pthread_attr_t *>(&m_attr), &res);
  return (DetachState)res;
}

size_t Thread::getStackSize() const {
  size_t res;
  ::pthread_attr_getstacksize(const_cast<pthread_attr_t *>(&m_attr), &res);
  return res;
}

/*
 * class ThreadImpl
 */ 
// Structors :
ThreadImpl::ThreadImpl(Thread *t)
  :m_thread(t), m_id(0) {}

ThreadImpl::~ThreadImpl() {
  m_thread->m_active = NULL;
  if( 0!=m_id )
    pthread_cancel(m_id);
}

// Manipulators :

void ThreadImpl::start() {
  int ret = ::pthread_create(&m_id, &(m_thread->m_attr), thread_launch, this);
  if( 0!=ret )
    throw ErrnoExcept("Thread::start");
}

void *ThreadImpl::run() {
  void *ret = m_thread->run();
  m_id = 0;
  delete this;
  return ret;
}

void *ThreadImpl::join() {
  void *res = NULL;
  Thread *that = m_thread;
  int ret;
  
  ret = pthread_join(m_id, &res);
  switch( ret ) {
  case 0:
    if( that->m_active==this ) {
      m_id = 0;
      delete this;
    }
    return res;
  case ESRCH:
    if( that->m_active==this ) {
      m_id = 0;
      delete this;
    }
  case EINVAL:
  default:
    throw ErrnoExcept("Thread::join");
  }
}
