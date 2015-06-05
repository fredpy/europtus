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
#ifndef H_europtus_clock
# define H_europtus_clock 

# include <boost/chrono/chrono.hpp>
# include <boost/date_time/posix_time/posix_time_io.hpp>
# include <boost/system/system_error.hpp>
# include <boost/signals2/signal.hpp>
# include <boost/thread/recursive_mutex.hpp>

namespace europtus {
  
  class clock {
  public:
    typedef unsigned long long         tick_type;
    typedef boost::posix_time::ptime   date_type;
    typedef boost::chrono::nanoseconds duration_type;
    
    typedef boost::signals2::signal<void (clock &,
                                          tick_type)> tick_sig;
    
    enum state {
      clock_started = 1,
      clock_completed
    };
    
    typedef boost::signals2::signal<void (clock &,
                                          state)> clock_sig;
    
    virtual ~clock();
    
    bool      started() const;
    bool      completed() const;
    bool      active() const {
      return started() && !completed();
    }
    date_type      epoch() const;
    date_type      end() const;
    tick_type zero() const {
      return 0;
    }
    tick_type final() const;
    virtual duration_type tick_duration() const=0;
    
    tick_type to_tick(date_type const &date) const;
    date_type to_date(tick_type const &tick) const;

    
    void      start();
    tick_type tick();
    
    bool restrict_end(date_type const &date);
    bool restrict_final(tick_type tick);
    
    
    void sleep(boost::system::error_code &ec) const;
    void sleep() const {
      boost::system::error_code ec;
      sleep(ec);
      if( ec )
        throw boost::system::system_error(ec);
    }
    
    void on_clock(clock_sig::slot_type const &cb);
    
    
    tick_sig &on_tick() {
      return m_tick_ev;
    }
    tick_sig &on_final_update() {
      return m_final_ev;
    }

    date_type rt_now() const;
    
  protected:
    clock();
    
    virtual tick_type to_tick_impl(date_type const &date) const =0;
    virtual date_type to_date_impl(tick_type const &tick) const =0;

    virtual date_type do_start() =0;
    virtual void do_sleep(boost::system::error_code &ec) const =0;
    virtual tick_type get_tick() =0;
    
    void sleep(boost::chrono::nanoseconds &delay,
               boost::system::error_code &ec) const;
  
  private:
    typedef boost::recursive_mutex  mutex_type;
    typedef mutex_type::scoped_lock mtx_lock;
    
    mutable mutex_type m_lock;
    
    bool      m_started, m_completed;
    date_type m_epoch;
    tick_type m_cur, m_final;
    
    clock_sig m_clock_ev;
    tick_sig m_tick_ev, m_final_ev;
    
    bool check_completed() {
      if( m_cur>=m_final )
        m_completed = true;
      return m_completed;
    }
    
    
  };
  
} // europtus

#endif // H_europtus_clock