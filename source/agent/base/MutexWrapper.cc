
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
/** @file "Mutex.cc"
 *
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#include "MutexWrapper.hh"

using namespace TREX;

/*
 * class Mutex
 */
// Structors :

Mutex::Mutex() {
  int ret;

  ret = pthread_mutex_init(&m_mutexId, NULL);
  if( 0!=ret )
    throw ErrnoExcept("Mutex::Mutex");
}

Mutex::~Mutex() {
  int ret;

  ret = pthread_mutex_destroy(&m_mutexId);
  if( 0!=ret )
    throw ErrnoExcept("Mutex::~Mutex");
}

// Manipulators :

void Mutex::lock() {
  int ret;

  ret = pthread_mutex_lock(&m_mutexId);
  if( 0!=ret ) 
    throw ErrnoExcept("Mutex::lock");
}

void Mutex::unlock() {
  int ret;

  ret = pthread_mutex_unlock(&m_mutexId);
  if( 0!=ret ) 
    throw ErrnoExcept("Mutex::unlock");
}

bool Mutex::checkLock(bool test, std::string const &from) const {
  int ret;
  
  ret = pthread_mutex_trylock(&m_mutexId);
  switch(ret) {
  case EBUSY:
    return false;
  case 0:
    if( test )
      ret = pthread_mutex_unlock(&m_mutexId);
    if( ret==0 )
      return true;
  default:
    throw ErrnoExcept(from);
  }
}

bool Mutex::trylock() {
  return checkLock(false, "Mutex::trylock");
}

// Observers :

bool Mutex::isLocked() const {
  return !checkLock(true, "Mutex::isLocked");
}
