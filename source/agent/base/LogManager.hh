/* -*- C++ -*-
 * $Id$
 */
/** @file "LogManager.hh"
 * @brief Define a log managing utility
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef _LOGMANAGER_HH
#define _LOGMANAGER_HH

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

# include <memory>
# include <fstream>

# include "EuropaXML.hh"

# include "TextLog.hh"
# include "TickLogger.hh"

# define TREX_LOG_FILE "TREX.log" 
# define TREX_DBG_FILE "Debug.log"

# define LOG_DIR_ENV "TREX_LOG_DIR"
# define LATEST_DIR "latest"
# define MAX_LOG_ATTEMPT 1024

namespace TREX {

  /** @brief A simple log manager.
   *
   * This class implements a simple singleton based log manager for TREX.
   * Its role is to create a new directory where the log files will
   * be stored. When the singleton is created it tries to create
   * $TREX_LOG_DIR/%Y.%j.%n where :
   * @li %Y is the year
   * @li %j is the day of the year
   * @li %n is a number between 0 and MAX_LOG_ATTEMPT
   *
   * This directory is then provided to clients to create and/or
   * manipulate log files for this session.
   *
   * @warn This class is not thread safe !!!
   */
  class LogManager {
  public:
    /** @brief singleton access.
     *
     * This method manages the creation of the singleton.
     *
     * @return The singleton instance.
     *
     * @note This method offers :
     * @li Thread safety in the sense that we ensure
     * that the singleton creation is mutex protected.
     * @li Lifetime guarantees : the singleton is managed by an std::auto_ptr
     * ensuring its destruction at the send of the program.
     */
    static LogManager &instance();

    /** @brief Log path.
     *
     * @return The path created for logging in this session.
     */
    std::string const &get_log_path() const {
      return m_path;
    }
    /** @brief Real log filename
     *
     * @param short_name A short (local) filename
     *
     * @return The real filename for the @e short_name log file.
     */
    std::string file_name(std::string const &short_name) const;

    std::string reactor_dir_path(
	std::string const &agent_name,
	std::string const &reactor_name,
	std::string local_dir_path) const;
    /** @brief Generate real log filename for reactor-specific files
     *
     * If a reactor-specific log is to be generated, this will create a path
     * to the file in logs/latest/agent_name.reactor_name/base_name.ext
     *
     * @param agent_name The name of the agent
     * @param reactor_name The name of the reactor
     * @param file_rel_path The path to the file inside of the reactor dir
     *
     * @return The real filename for the @e short_name log file.
     */
    std::string reactor_file_path(
	std::string const &agent_name,
	std::string const &reactor_name,
	std::string local_file_path) const;

    static std::string const &use(std::string const &fileName);
    static TiXmlElement *initXml(std::string const &fileName) {
      return EUROPA::initXml(use(fileName).c_str());
    }

    /** @brief System log entry point.
     *
     * @return a reference to the syslog file.
     *
     * @note The syslog file is a TextLog ensuring thread safety.  
     */
    TextLog &syslog() {
      return m_syslog;
    }

    /** @brief Get one TickLogger access.
     *
     * @param baseName dhort name of the file
     * 
     * This method is used to have an access to one particular numerical data logging file.
     * If the file was not created yet it is created on the fly. This function would be mostly
     * used to add a new entry to the TickLogger file.
     *
     * @pre The TickLogger file @e baseName is opened.
     *
     * @return A pointer to the file.
     *
     * @bug For now the code of this function and TickLogger implementations are not thread safe.
     *
     * @sa TickLogger
     */
    TickLogger *getTickLog(std::string const &baseName);

    /** @brief Handle agent initialisation.
     *
     * This method is used to do some preparation when Agent is initialized.
     * In particular it prints the header of TickLogger files.
     */
    void handleInit() const;
    /** @brief Handle new tick start.
     *
     * @param current Value of the new tick.
     *
     * This method is called by Agent to indicate to the LogManager that a new tick has started.
     * This is mainly a frontend to TickLogger::handleNewTick(TICK) that allows to put a new line
     * of data on the TickLogger files
     */
    void handleNewTick(TICK current) const;
        
  private:
    /** @brief log path
     */
    std::string m_path;
    
    /** @brief Constructor.
     */
    LogManager();
    /** @brief Destructor.
     */
    ~LogManager();


    /** @brief Managed TickLogger
     *
     * This attribute is used to stroe and maintain all the TickLogger managed by this class.
     */
    std::map<std::string, TickLogger *> m_logs;
    /** @brief Debug message log file.
     *
     * This file is the redirection stream for all the debug messages produced by a TREX agent.
     */
    std::ofstream m_debug;
    /** @brief System log.
     *
     * This attribute manages a ThreadSafe text log to put TREX system log messages.
     */
    TextLog m_syslog;

    friend class std::auto_ptr<LogManager>;
    
    /** @brief Singleton
     */
    static std::auto_ptr<LogManager> s_instance;

    static std::string short_name(std::string const &file_name);

    // Following methods are not implemented in purpose.
    void operator= (LogManager const &);
    LogManager(LogManager const &);
  }; // LogManager

} // TREX 

/** @brief System logginf macro.
 *
 * This macro is an helper to put a system log message.
 *
 * @param id An identifer which will be put at the start of the entry.
 *
 * @sa TREX::LogManager::syslog()
 */
# define TREXLog() TREX::LogManager::instance().syslog()

#endif // _LOGMANAGER_HH
