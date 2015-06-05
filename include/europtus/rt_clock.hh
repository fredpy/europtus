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
#ifndef H_europtus_rt_clock
# define H_europtus_rt_clock

# include "clock.hh"
# include "tick_clock.hh"
# include "time_utils.hh"

namespace europtus {
  
  template<class Period, class Clock=boost::chrono::high_resolution_clock>
  class rt_clock :public clock {
    typedef tick_clock<Period,Clock> base_clock;
    
  public:
    using clock::duration_type;
    using clock::date_type;
    using clock::tick_type;
    
    typedef typename base_clock::duration clock_rate;
    typedef typename base_clock::rep      rep;
    
    explicit rt_clock(rep const &p):m_period(p) {
      restrict_final(cvt_base::max_chrono().count()/m_period.count());
    }
    explicit rt_clock(clock_rate const &p):m_period(p) {
      restrict_final(cvt_base::max_chrono().count()/m_period.count());
    }
    ~rt_clock() {}
    
    duration_type tick_duration() const;

    
  private:
    clock_rate const                m_period;
    base_clock                      m_clock;
    
    typedef details::date_cvt<clock_rate> cvt_date;
    typedef typename cvt_date::convert    cvt_base;
    cvt_date m_cvt;
    typename base_clock::time_point m_tick;
    
    date_type do_start();
    void do_sleep(boost::system::error_code &ec) const;
    tick_type get_tick();
    
    tick_type to_tick_impl(date_type const &date) const;
    date_type to_date_impl(tick_type const &tick) const;

    // not implemented
    rt_clock();
  }; // europtus::rt_clock
  
} // europtus

# include "bits/rt_clock.tcc"

#endif // H_europtus_rt_clock
