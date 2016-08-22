/* -*- C++ -*-
 */
/** @file "TimeUtils.hh"
 *
 * @brief C++ utilities for @c timeval
 *
 * This file defines some C++ operators to manipulate @c timeval
 * structure in a mor intuitive way.
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef H_TimeUtils
#define H_TimeUtils

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

#include <sys/time.h>

#include <cmath>

#include <iostream>
#include <iomanip>

#define TV_USEC_F 1000000l

namespace TREX {

  /** @brief "Less than" operator
   *
   * @param t1 a date
   * @param t2 a date
   *
   * @retval true if @e t1 is before @e t2
   * @retval false else.
   */
  inline bool operator< (timeval const &t1, timeval const &t2) {
    return t1.tv_sec < t2.tv_sec ||
      (t1.tv_sec == t2.tv_sec && t1.tv_usec < t2.tv_usec);
  }

  /** @brief "Equal to" operator
   *
   * @param t1 a date
   * @param t2 a date
   *
   * @retval true if @e t1 has the same value as @e t2
   * @retval false else.
   */
  inline bool operator==(timeval const &t1, timeval const &t2) {
    return (t1.tv_sec == t2.tv_sec && t1.tv_usec == t2.tv_usec);
  }

  /** @brief "Not equal to" operator
   *
   * @param t1 a date
   * @param t2 a date
   *
   * @retval true if @e t1 is different from @e t2
   * @retval false else.
   */
  inline bool operator!=(timeval const &t1, timeval const &t2) {
    return !operator==(t1, t2);
  }

  /** @brief "Greater than" operator
   *
   * @param t1 a date
   * @param t2 a date
   *
   * @retval true if @e t1 is after @e t2
   * @retval false else.
   */
  inline bool operator> (timeval const &t1, timeval const &t2) {
    return t2<t1;
  }

  /** @brief "Less or equal to" operator
   *
   * @param t1 a date
   * @param t2 a date
   *
   * @retval true if @e t1 is before or equal to @e t2
   * @retval false else.
   */
  inline bool operator<=(timeval const &t1, timeval const &t2) {
    return !operator> (t1, t2);
  }

  /** @brief "Greater or equal to" operator
   *
   * @param t1 a date
   * @param t2 a date
   *
   * @retval true if @e t1 is after or equal to @e t2
   * @retval false else.
   */
  inline bool operator>=(timeval const &t1, timeval const &t2) {
    return !operator< (t1, t2);
  }

  /** @brief Addition 
   *
   * @param t1 a date
   * @param t2 a date
   *
   * @return The sum of @e t1 and @e t2
   */
  inline timeval operator+ (timeval const &t1, timeval const &t2) {
    timeval result;

    result.tv_sec = t1.tv_sec + t2.tv_sec;
    if ( (result.tv_usec = t1.tv_usec + t2.tv_usec)>=TV_USEC_F ) {
      ++result.tv_sec;
      result.tv_usec -= TV_USEC_F;
    }
    return result;
  }

  /** @brief Conversion to double
   *
   * @param t a date
   *
   * Convert @e t to a @c double value.
   *
   * @warning Ther may be a prescsion loss during conversion.
   *
   * @return A double value corresponding to @e t
   */
  inline double to_double(timeval const &t) {
    double result = t.tv_usec;
  
    result /= TV_USEC_F;
    result += t.tv_sec;
    return result;
  }

  /** @brief Increment operator
   *
   * @param t1 a date
   * @param t2 a date
   *
   * This operator increments @t1 by the value of @e t2.
   *
   * @return @e t1 after the operation
   */
  inline timeval &operator+=(timeval &t1, timeval const &t2) {
    t1.tv_sec += t2.tv_sec;
    if ( (t1.tv_usec += t2.tv_usec) >= TV_USEC_F ) {
      ++t1.tv_sec;
      t1.tv_usec -= TV_USEC_F;
    }
    return t1;
  }

  /** @brief Substract
   *
   * @param t1 a date
   * @param t2 a date
   *
   * @return the difference between @e t1 and @e t2
   */
  inline timeval operator- (timeval const &t1, timeval const &t2) {
    timeval result;
    result.tv_sec = t1.tv_sec - t2.tv_sec;
    if ( (result.tv_usec = t1.tv_usec - t2.tv_usec) < 0 ) {
      --result.tv_sec;
      result.tv_usec += TV_USEC_F;
    }
    return result;
  }

  /** @brief Divide 
   *
   * @param t a date
   * @param n The divider 
   *
   * @return The division of @e t by @e n
   */
  inline timeval operator/ (timeval const &t, long n) {
    timeval result;

    result.tv_sec = t.tv_sec / n;
    result.tv_usec = ((t.tv_sec % n) * TV_USEC_F + t.tv_usec) / n;

    return result;
  }

  /** @brief Multiply 
   *
   * @param t a date
   * @param n The factor 
   *
   * @return The mutlplication of @e t by @e n
   */
  inline timeval operator* (timeval const &t, long n) {
    timeval result;

    result.tv_usec = t.tv_usec * n;
    result.tv_sec = t.tv_sec * n + (result.tv_usec) / TV_USEC_F;
    result.tv_usec = result.tv_usec % TV_USEC_F;

    return result;    
  }

  /** @brief Decrement operator
   *
   * @param t1 a date
   * @param t2 a date
   *
   * This operator decrement the value of @e t1 by @e t2.
   *
   * @return @e t1 after the operation 
   */
  inline timeval &operator-=(timeval &t1, timeval const &t2) {
    t1.tv_sec -= t2.tv_sec;
    if ( (t1.tv_usec -= t2.tv_usec) < 0 ) {
      --t1.tv_sec;
      t1.tv_usec += TV_USEC_F;
    }
    return t1;
  }

  /** @brief Print operator
   *
   * @param os An output stream
   * @param t a date
   *
   * Thsi method writes the value of @e t in @e os. The value
   * format is @e seconds.microseconds
   *
   * @return @e os after the operation
   */
  inline std::ostream &operator<<(std::ostream &os, timeval const& t) {
    return os<<t.tv_sec<<"."<<std::setw(6)<<std::setfill('0')<<t.tv_usec;
  }

} // TREX

#endif // H_TimeUtils 
