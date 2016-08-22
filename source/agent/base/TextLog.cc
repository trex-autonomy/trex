
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
/** @file "TextLog.cc"
 *
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#include "Guardian.hh"

#include "TextLog.hh"

using namespace TREX;
/*
 * class TextLog
 */ 

// structors :

TextLog::TextLog() {}

TextLog::TextLog(std::string const &name)
  :m_log(name.c_str()) {}

TextLog::~TextLog() {}

// Manipulators :

void TextLog::open(std::string const &name) {
  Guardian<Mutex> guard(m_lock);
  
  m_log.open(name.c_str());
}

void TextLog::write(std::string const &text) {
  Guardian<Mutex> guard(m_lock);
  
  m_log<<text<<std::flush;
}

/* 
 * class LogEntry 
 */ 

// structors :

LogEntry::LogEntry(TextLog &owner)
  :m_owner(owner) {}

LogEntry::LogEntry(LogEntry const &other) 
  :m_owner(other.m_owner), m_buff(other.m_buff.str()) {
  other.m_buff.str(""); // cleaning old temporary variable
}

LogEntry::~LogEntry() {
  std::string to_log = m_buff.str();
  if( !to_log.empty() )
    m_owner.write(to_log);
}
