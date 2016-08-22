/* -*- C++ -*-
 * $Id$
 */
/** @file "TextLog.hh"
 * @brief 
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef _TEXTLOG_HH
#define _TEXTLOG_HH

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

#include <fstream>
#include <memory>
#include <sstream>

#include "MutexWrapper.hh"

namespace TREX {

  class LogManager;
  
  class LogEntry;

  /** @brief Thread safe text logging class.
   *
   * This class implements a simple logging system that ensures that the
   * text displayed will not be splitted by other thread access.
   *
   * @author Frederic Py <fpy@mbari.org>
   *
   * @todo This class has to be more strongly linked to LogManager to be sure that
   * the created file will be on the log directory. 
   */
  class TextLog {
  public:
    /** @brief Default constructor.
     *
     * Create a new instance without aopening a new file.
     */
    TextLog();
    /** @brief Constructor.
     *
     * @param file A file name.
     * 
     * Create a new instance connected to a file.
     */
    explicit TextLog(std::string const &file);
    /** @brief Destructor.
     */
    ~TextLog();

    /** @brief Starts a new entry.
     *
     * @param val A value to print.
     *
     * This method is used to create a new entry transprently. It creates the new entry and writes
     * the value of @e val in here. The entry will be effectively wirtten in the log file
     * at its destruction.
     *
     * @return The newly created LogEntry
     *
     * @sa LogEntry
     */
    template<typename Ty>
    LogEntry operator<<(Ty const &val);

    /** @brief Open the log file.
     *
     * @param file file name
     */
    void open(std::string const &file);

  private:
    /** @brief stream mutex
     *
     * This mutex is used by TextLog::write to ensure that one text is written at a time.
     */
    Mutex m_lock;
    /** @brief Log file
     *
     * This is the file that will be updated by TextLog::write.
     */
    std::ofstream m_log;

    /** @brief Physical log writing
     *
     * This method is called by LogEntry destructor to write a new entry physically
     * on the log file. It is the only critical section connected to this class.
     *
     * @param text The text to wirite in the log file.
     */ 
    void write(std::string const &text);

    friend class LogEntry;
  }; // TextLog

  /** @brief TextLog proxy for producting a new entry.
   *
   * This class is a proxy that allows to write into the TextLog in an asynchronous way.
   * Each time one want to write a new entry in a TextLog file, a new instance of LogEntry
   * is created and the text is stored (using operator<<) into this class. At its
   * destruction, the text content is then sent to the TextLog class that created it ensuring
   * some atomicity of the critical section.
   *
   * @note You can see that this class has no public constructors. Indeed, as the output of a
   * LogEntry is sent to the file at its destruction, this class is designed in such way that
   * nobody is able to create an instance that will last long. The only class allowed to create
   * a new LogEntry is :
   * @li TextLog to be able to create a new entry connected to this class.
   *
   * @warning This class is not thread safe. It is just a proxy that allows TextLog to be thread safe.
   * Normally the design of the class is well suited to avoid instances shared between multiple threads
   * but if you find a way to circumvent it, the behavior of TextLog will not be guaranteed anymore.      
   *
   * @author Frederic Py <fpy@mbari.org>
   */
  class LogEntry {
  public:
    /** @brief Destructor.
     *
     * Will write the content of the entry if any.
     *
     * @sa TextLog::write
     */
    ~LogEntry();

    /** @{ 
     * @brief Output operator
     */
    template<typename Ty>
    LogEntry &operator<<(Ty const &val);

    // Specialization needed for using std::endl, std::flush and so on
    LogEntry &operator<<(std::ostream& (*f)(std::ostream&));
    /** @} */

  private:
    /** @brief Copy constructor.
     *
     * @param other Another instance.
     *
     * This constructor creates a copy of @e other and transfer the content of @e other
     * to this new one.
     *
     * This ownership transfer is necessary to avoid that a given content is multiply written.
     */
    LogEntry(LogEntry const &other);
    /** @brief Constructor.
     *
     * @param owner A TextLog
     *
     * Create a new LogEntry that will write its final content to @e owner
     */
    LogEntry(TextLog &owner);
    
    /** @brief Destination log */
    TextLog &m_owner;
    /** @brief Proxy stream.
     *
     * This is the stream used as proxy. All output operations to the LogEntry will
     * be buffered using this stream which will give the final text to write
     * at destruction.
     */
    mutable std::ostringstream m_buff;
    
    // These function are not implemented in purpose
    LogEntry();
    void operator= (LogEntry const &other);

    friend class TextLog;
  };

  // Templates

  /*
   * class TextLog
   */ 
  template<typename Ty>
  LogEntry TextLog::operator<<(Ty const &val) {
    LogEntry entry(*this);
    entry<<val;
    return entry;
  }

  /*
   * class LogEntry
   */ 
  template<typename Ty>
  LogEntry &LogEntry::operator<<(Ty const &val) {
    m_buff<<val;
    return *this;
  }
  

  inline LogEntry &LogEntry::operator<<(std::ostream& (*f)(std::ostream&)) {
    m_buff<<f;
    return *this;
  }
  
} // TREX

#endif // _TEXTLOG_HH
