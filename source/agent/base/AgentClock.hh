#ifndef H_AgentClock
#define H_AgentClock

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

#include "TREXDefs.hh"
#include "TeleoReactor.hh"
#include "RStat.hh"
#include <sys/time.h>
#include "Mutex.hh"

/**
 * @brief Declaration of clock interface and implementation sub-classes
 */


namespace TREX {

  /**
   * @brief Abstract class to plug in different clocks
   */
  class Clock {
  public:
    virtual ~Clock(){}

    void doStart();

    /**
     * @brief Used by Agent to get the value of the next tick for synchronization
     */
    virtual TICK getNextTick() = 0;

    /**
     * @brief Helper method to provide a high-resolution sleep method
     * @see sleep(sleepDuration)
     */
    virtual void sleep() const;

    /**
     * @brief Utility to implement high-resolution sleep
     * @param sleepDuration The sleep duration in seconds. Accurate up to nanoseconds.
     */
    static void sleep(double sleepDuration);

    bool debugStats() const {
      return m_processStats;
    }
    
    /** @brief Global process statistics.
     *
     * @return the stats collected for the whole duration of the process.
     */
    RStat const &totalStat() const {
      return m_cur;
    }
    /** @brief Last tick process statistics.
     *
     * @return the stats of the process during the lasst tick.
     */
    RStat const &lastTickStat() const {
      return m_diff;
    }

    /**
     * @brief Accessor for seconds per tick
     */
    double getSecondsPerTick() const {
      return m_secondsPerTick;
    }

  protected:
    virtual double getSleepDelay() const {
      return 0;
    }
    /**
     * @brief Constructor
     * @param secondsPerTick The period of a single tick
     */
    Clock(double secondsPerTick, bool stats = true) : 
      m_secondsPerTick(secondsPerTick),
      m_processStats(stats)
    { }

    /**
     * @brief Called to start the clock counting
     */
    virtual void start(){}

    /** @brief Advance tick and update the stats.
     *
     * @param tick The main tick for clock
     *
     * Increment @e tick and update the process statisitcs for the last tick.
     *
     * @note This function has to be used only for the main clock tick.
     */
    void advanceTick(TICK &tick); 
    
    double m_secondsPerTick;

  private:
    bool m_processStats;
    RStat m_diff, m_cur;
  };

  /**
   * @brief Simple clock for stepping the code on the main thread
   */
  class PseudoClock: public Clock {
  public:
    PseudoClock(double secondsPerTick, unsigned int stepsPerTick, bool stats = true);

    /**
     * @brief Simple clock for stepping the code on the main thread
     */
    TICK getNextTick();
    virtual double getSleepDelay() const;

  private:
    /**
     * @brief Enforces a stepsPerTick > 0, writes to log if <0, and sets to some non-zero value
     */
    static TICK selectStep(unsigned int stepsPerTick);

    TICK m_tick;
    TICK m_internalTicks;
    const TICK m_stepsPerTick;
  };

  /**
   * @brief A clock that monitors time on a separate thread and generates updates to the tick.
   */
  class RealTimeClock: public Clock {
  public:
    RealTimeClock(double secondsPerTick, bool stats = true);

    /**
     * @brief Will idle till this is called.
     */
    void start();

    /**
     * @brief Retrieve the tick
     */
    TICK getNextTick();

  protected:
    double getSleepDelay() const;

  private:
    static void getDate(timeval &val);
    void setNextTickDate(unsigned factor=1);
    double timeLeft() const;

    bool m_started;
    TICK m_tick;
    timeval m_tvSecondsPerTick;
    timeval m_nextTickDate;
    mutable Mutex m_lock;
  };
    
}

#endif
