
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
/** @file "TickLogger.cc"
 *
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#include "LogManager.hh"

using namespace TREX;

/*
 * class TickLogger
 */ 
// structors :

TickLogger::TickLogger(std::string const &fileName)
  :m_baseName(fileName), m_file(LogManager::instance().file_name(fileName).c_str()),
   m_inHeader(true) {}

TickLogger::~TickLogger() {
  std::map<std::string, AbstractField *>::iterator i = m_fields.begin(), 
    endi=m_fields.end();
  for( ; endi!=i; ++i )
    delete i->second;
}

// Manipulators :

TickLogger::AbstractField &TickLogger::getField(std::string const &name) {
  return *(m_fields.find(name)->second);
}

void TickLogger::printHeader() {
  m_inHeader = false;
  order_type::const_iterator i=m_print_order.begin(), endi = m_print_order.end();
  size_t count = 2;
  
  m_file<<"TICK[1]";
  for(; endi!=i; ++i, ++count)
    m_file<<'\t'<<(*i)->first<<'['<<count<<']';
  m_file<<std::endl;
}

// Observers :

void TickLogger::handleNewTick(TICK current) {
  order_type::const_iterator i=m_print_order.begin(), endi = m_print_order.end();
  
  m_file<<current;
  for( ; endi!=i; ++i) {
    m_file.put('\t');
    (*i)->second->print(m_file);
  }
  m_file<<std::endl;
}

bool TickLogger::exist(std::string const &name) const {
  return m_fields.find(name)!=m_fields.end();
}

TickLogger::AbstractField const &TickLogger::getField(std::string const &name) const {
  return *(m_fields.find(name)->second);
}
