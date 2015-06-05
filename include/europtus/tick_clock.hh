/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Frederic Py
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
#ifndef H_europtus_tick_clock
# define H_europtus_tick_clock

# include <boost/chrono.hpp>


namespace europtus {
  
  /** @brief A tick based clock
   *
   * This class is mostly an helper to convert a chono clock at 
   * a given frequency. ch   *
   * It takes a chrono clock and gives the time points based on 
   * an epoch defined by the user (either at construction or later)
   * at a period defined by a ratio.
   *
   * It also provide several methods that allow to convert dates and/or 
   * durations from and to the base_clock.
   *
   * It is mostly used internally hence explaining its relatively 
   * bare-bone API
   */
  template <typename Period, class Clock=boost::chrono::system_clock>
  class tick_clock {
  public:
    typedef Clock                           base_clock;
    typedef typename base_clock::duration   base_duration;
    typedef typename base_clock::time_point base_time_point;
    
    typedef typename base_clock::rep rep;
    typedef Period                   period;
    
    typedef boost::chrono::duration<rep, period>            duration;
    typedef boost::chrono::time_point<tick_clock, duration> time_point;
    
    static bool const is_steady = base_clock::is_steady;
    
    tick_clock():m_epoch(base_clock::now()) {}
    explicit tick_clock(base_time_point const &date)
    :m_epoch(date) {}
    template <typename Period2>
    tick_clock(tick_clock<Period2, Clock> const &other)
    :m_epoch(other.epoch()) {}
    ~tick_clock() {}
    
    time_point now() const {
      base_duration ignored;
      return from_base(base_clock::now(), ignored);
    }
    
    base_time_point to_base(time_point const &tick) const {
      return epoch()+boost::chrono::duration_cast<base_duration>(tick.time_since_epoch());
    }
    time_point      from_base(base_time_point const &date,
                              base_duration &delta) const {
      delta = date-epoch();
      duration my_delta = boost::chrono::duration_cast<duration>(delta);
      delta -= boost::chrono::duration_cast<base_duration>(my_delta);
      
      return time_point(my_delta);
    }
    base_duration left(time_point const &tick) const {
      duration t_dur = tick.time_since_epoch();
      base_time_point real_target = epoch()+boost::chrono::duration_cast<base_duration>(t_dur);
      return real_target-base_clock::now();
    }
    
    base_time_point const &epoch() const {
      return m_epoch;
    }
    void reset(base_time_point const &date) {
      m_epoch = date;
    }
    void reset() {
      reset(base_clock::now());
    }
    
    base_duration to_next(time_point &date, duration const &tick) const {
      base_duration ret(base_clock::now()-epoch());
      duration extra(boost::chrono::duration_cast<duration>(ret));
      time_point cur(extra);
      
      extra%=tick;
      cur -= extra;
      if( cur>date ) {
        ret -= date.time_since_epoch();
        ret -= tick;
        date = cur;
        return ret;
      } else
        return base_duration::zero();
    }
    
    std::ostream &print(std::ostream &out, time_point const &tick) const {
      return out<<to_base(tick);
    }
    
  private:
    base_time_point m_epoch;
    
  }; // europtus::tick_clock<Period,Clock>
  
} // europtus


#endif // H_europtus_tick_clock