
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
/** @file "LogManager.cc"
 *
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <cerrno>

#include "Debug.hh"

#include "TREXDefs.hh"
#include "LogManager.hh"

#include "MutexWrapper.hh"
#include "Guardian.hh"

using namespace TREX;
/*
 * class LogManager
 */ 

// statics :

std::auto_ptr<LogManager> LogManager::s_instance(0);

std::string LogManager::short_name(std::string const &file_name) {
  size_t slash = file_name.find_last_of('/');

  if( slash!=std::string::npos ) 
    return std::string(file_name, slash+1);
  return file_name;
}

LogManager &LogManager::instance() {
  static Mutex sl_mutex;

  if( 0==s_instance.get() ) {
    Guardian<Mutex> guard(sl_mutex);
    if( 0==s_instance.get() ) // double check in case another process did create it
      s_instance.reset(new LogManager);
  }
  return *s_instance;
}

// structors :

LogManager::LogManager() {
  char *base_dir = getenv("TREX_LOG_DIR");
  char dated_dir[17];

  dated_dir[16] = '\0';

  time_t cur_time;
  
  time(&cur_time);
  strftime(dated_dir, 16, "/%Y.%j.", gmtime(&cur_time));
  
  // First I check if latest exists
  std::string latest = (base_dir != NULL ? base_dir : ".");

  // Create if necessary
  debugMsg("LogManager", "Setting up log directory in:" << latest);
  static const mode_t  LOG_MODE = 0777; //S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP | S_IWOTH;
  mkdir(latest.c_str(), LOG_MODE);

  std::string cur_basis = latest;
  size_t len = latest.length()+30;
  char *buf = new char[len+1];

  memset(buf, 0, len+1);
  latest = latest+"/"+LATEST_DIR;
  cur_basis += dated_dir;
  
  int ret = readlink(latest.c_str(), buf, len-1);
  size_t last = 0;

  if( ret>0 ) {
    // The "latest" exist and point to something lets check it
    size_t b_len = cur_basis.length();
    if( cur_basis.compare(0, b_len, buf, b_len)==0 ) {
      std::istringstream iss(std::string(buf, b_len));
      
      iss>>last;
    }

    debugMsg("LogManager", "Unlinking latest directory");
    unlink(latest.c_str());
  }

  delete[] buf;

  for(unsigned i=1; i<MAX_LOG_ATTEMPT; ++i) {
    std::ostringstream oss;

    oss<<cur_basis<<(last+i);
    if(mkdir(oss.str().c_str(), LOG_MODE) != 0)
      continue;

    m_path = oss.str();
    // Create the "latest" symbolic link. As it is not  "critical" I am not checking it. 
    // Normally the unlink as already been done 
    //       unlink(latest.c_str());
    symlink(m_path.c_str(), latest.c_str());

    m_syslog.open(file_name(TREX_LOG_FILE).c_str());
    m_debug.open(file_name(TREX_DBG_FILE).c_str());

    DebugMessage::setStream(m_debug);

    debugMsg("LogManager", " logging directory is \""<<m_path<<'\"');
    return;
  }

  checkError(ALWAYS_FAIL, "LogManager: too many log dirs (MAX_LOG_ATTEMPT).");
}

LogManager::~LogManager() {
  std::map<std::string, TickLogger *>::iterator i = m_logs.begin(), 
    endi = m_logs.end();
  for( ;endi!=i ; ++i )
    delete i->second;
}

// Manipulators:

TickLogger *LogManager::getTickLog(std::string const &baseName) {
  std::pair<std::string, TickLogger *> to_ins(baseName, NULL);
  std::pair<std::map<std::string, TickLogger *>::iterator, bool> ret = m_logs.insert(to_ins);
  
  if( ret.second ) {
    debugMsg("LogManager", " creating numeric data log \""<<baseName<<'\"');
    ret.first->second = new TickLogger(baseName);
  }
  return ret.first->second;
}

// Observers :

void LogManager::handleInit() const {
  std::map<std::string, TickLogger *>::const_iterator i=m_logs.begin(), 
    endi = m_logs.end();
  for( ; endi!=i; ++i)
    i->second->printHeader();
}

void LogManager::handleNewTick(TICK current) const {
  std::map<std::string, TickLogger *>::const_iterator i=m_logs.begin(), 
    endi = m_logs.end();
  for( ; endi!=i; ++i)
    i->second->handleNewTick(current);
}

std::string LogManager::file_name(std::string const &short_name) const {
  return m_path+"/"+short_name;
}

std::string LogManager::reactor_dir_path(
    std::string const &agent_name,
    std::string const &reactor_name,
    std::string local_dir_path) const
{
  // Get reactor base directory (logs/latest/agent_name.reactor_name)
  std::string abs_path = m_path;
  
  local_dir_path = compose(agent_name,reactor_name).toString() +"/"+ local_dir_path;
  
  // This will create each local directory between the base log path and the target path
  // Find the index of the first delimiter in the path
  size_t dindex = local_dir_path.find_first_of("/");
  while( dindex != std::string::npos ) {
    // Get the next index
    dindex = local_dir_path.find_first_of("/");
    // Concatenate the local root entry on the end of the absolute path
    abs_path = abs_path + "/" + local_dir_path.substr(0,dindex);
    // Pop the root entry off of the path
    local_dir_path = local_dir_path.substr(dindex+1);
    // Try to create the directory
    int ret = mkdir(abs_path.c_str(), 0777);
    // Fail if mkdir returns an error other then EEXIST
    if( ret != 0 ) {
      checkError(EEXIST==errno, "LogManager: "<<strerror(errno));
    }
  }

  return abs_path;
}

std::string LogManager::reactor_file_path(
    std::string const &agent_name,
    std::string const &reactor_name,
    std::string local_file_path) const
{
  // Get the filename
  std::string filename = "";
  size_t last_delim = local_file_path.find_last_of("/");
  if(last_delim+1 < local_file_path.size()) {
    filename = local_file_path.substr(last_delim+1);
  }

  if(last_delim == std::string::npos) {
    local_file_path = "";
  }

  // Create the directories that this filename resides in
  std::string absolute_dir_path = LogManager::instance().reactor_dir_path(
      agent_name,
      reactor_name,
      local_file_path.substr(0,last_delim));

  return  absolute_dir_path + "/" + filename;
}

std::string const &LogManager::use(std::string const &fileName) {
  std::ifstream src(fileName.c_str(), std::ios::binary);

  if( !!src ) {
    // Create a sub directory for safety :
    int ret = mkdir(LogManager::instance().file_name("cfg").c_str(), 0777);
    if( 0!=ret )
      checkError(EEXIST==errno, "LogManager: "<<strerror(errno));
    std::string 
      destName = LogManager::instance().file_name("cfg/"+short_name(fileName));
    std::ofstream dest(destName.c_str(), std::ios::binary);
    dest<<src.rdbuf();
  } else 
    std::cerr<<"Unable to find \""<<fileName<<'\"'<<std::endl;
  return fileName;
}
