

/* -*- C++ -*-
 */
/** @file "ErrnoExcept.hh"
 *
 * @brief ErrnoExcept definition.
 *
 * @author Frederic Py <fpy@mbari.org>
 */

#ifndef H_ErrnoExcept
#define H_ErrnoExcept

/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2007, MBARI.
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

#include <cerrno>

#include <stdexcept>

namespace TREX {

  /** @brief errno based exception.
   *
   * This exception may be used to manage exceptional
   * events that modifies errno value.
   *
   * @author Frederic Py <fpy@mbari.org>
   */
  class ErrnoExcept : public std::runtime_error {
  public:
    /** @brief Constructor
     *
     * @param from origin of the error.
     *
     * Create a new instance. The message attached to this instance
     * will have the form : "from: @\"errno message@\"".
     */
    explicit ErrnoExcept(std::string const &from) throw()
      :std::runtime_error(build_message(from)), _errno(errno) {}
    ErrnoExcept(std::string const &from, std::string const &what) throw()
      :std::runtime_error(build_message(from, what)), _errno(0) {}
    /** @brief Destructor */
    virtual ~ErrnoExcept() throw() {}

    /** @brief Get errno value.
     *
     * @return the value of @c errno when the exception was created.
     *
     * This method may be usefull when one want to check if the error
     * was critical or not.
     */
    int get_errno() const {
      return _errno;
    }
    
  private:
    /** @brief value of @c errno when exception was created. */ 
    int _errno;

    static std::string build_message(std::string const &from, 
				     std::string const &what) throw();

    /** @brief Build mnessage content.
     *
     * This method build the message content taking the string
     * message attached to current errno.
     *
     * @return The message built containing error infromation
     * grabbed with @c errno value 
     */
    static std::string build_message(std::string const &from) throw();
  }; // ErrnoExcept

} // TREX

#endif // H_ErrnoExcept
