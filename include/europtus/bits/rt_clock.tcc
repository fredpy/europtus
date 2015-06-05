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
#ifdef H_europtus_rt_clock 
# ifndef T_europtus_bits_rt_clock
#  define T_europtus_bits_rt_clock

namespace europtus {
  
  /*
   * class europtus::rt_clock<>
   */
  template<class P, class C>
  typename rt_clock<P,C>::duration_type rt_clock<P,C>::tick_duration() const {
    return boost::chrono::duration_cast<duration_type>(m_period);
  }

  template<class P, class C>
  typename rt_clock<P,C>::tick_type
  rt_clock<P,C>::to_tick_impl(typename rt_clock<P,C>::date_type const &date) const {
    return m_cvt.to_chrono(date).count()/m_period.count();
  }

  template<class P, class C>
  typename rt_clock<P,C>::date_type
  rt_clock<P,C>::to_date_impl(typename rt_clock<P,C>::tick_type const &tick) const {
    tick_type max_tick = m_cvt.chrono_max().count(),
      val=max_tick/m_period.count();
    
    // ensure that the date is within base_clock boundaries
    if( val>tick )
      val = tick;
    
    clock_rate p_date(val*m_period.count());
    return m_cvt.to_posix(p_date);
  }

  template<class P, class C>
  typename rt_clock<P,C>::date_type rt_clock<P,C>::do_start() {
    m_clock.reset();
    date_type n = rt_now();
    m_cvt.epoch(n);
    restrict_final(m_cvt.chrono_max().count()/m_period.count());
    return n;
  }
  
  template<class P, class C>
  void rt_clock<P,C>::do_sleep(boost::system::error_code &ec) const {
    typename base_clock::time_point target = m_tick+m_period;
    typedef boost::chrono::nanoseconds ns;
    ns remain = boost::chrono::duration_cast<ns>(m_clock.left(target));
    sleep(remain, ec);
  }

  template<class P, class C>
  typename rt_clock<P,C>::tick_type rt_clock<P,C>::get_tick() {
    m_clock.to_next(m_tick, m_period);
    return m_tick.time_since_epoch().count()/m_period.count();
  }


  
} // europtus

# endif // T_europtus_bits_rt_clock
#endif // H_europtus_rt_clock