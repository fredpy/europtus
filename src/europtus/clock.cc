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
#include "europtus/clock.hh"
#include "europtus/time_utils.hh"

using namespace europtus;
namespace sy=boost::system;
namespace ch=boost::chrono;
namespace pt=boost::posix_time;

/*
 * class europtus::clock
 */

// structors

clock::clock():m_started(false), m_completed(false) {
  m_final = std::numeric_limits<tick_type>::max();
}

clock::~clock() {}

// observers

bool clock::started() const {
  mtx_lock lock(m_lock);
  return m_started;
}
bool clock::completed() const {
  mtx_lock lock(m_lock);
  return m_completed;
}

clock::date_type  clock::epoch() const {
  mtx_lock lock(m_lock);
  if( m_started )
    return m_epoch;
  return date_type(boost::posix_time::not_a_date_time);
}

clock::date_type  clock::end() const {
  mtx_lock lock(m_lock);
  if( !m_started )
    return date_type(boost::posix_time::not_a_date_time);
  return to_date_impl(m_final);
}

clock::tick_type clock::final() const {
  mtx_lock lock(m_lock);
  return m_final;
}

clock::tick_type clock::to_tick(clock::date_type const &date) const {
  mtx_lock lock(m_lock);
  if( m_started )
    return to_tick_impl(date);
  return zero();
}

clock::date_type clock::to_date(clock::tick_type const &tick) const {
  mtx_lock lock(m_lock);
  if( m_started )
    return to_date_impl(tick);
  return date_type(boost::posix_time::not_a_date_time);
}


// modifiers

bool clock::restrict_end(clock::date_type const &date) {
  mtx_lock lock(m_lock);
  
  if( m_started )
    return restrict_final(to_tick_impl(date));
  return false;
}

bool clock::restrict_final(clock::tick_type tick) {
  bool updated = false;
  bool done = false;
  {
    mtx_lock lock(m_lock);
    if( m_final>tick ) {
      m_final = tick;
      done = check_completed();
      updated = true;
    }
  }
  if( updated ) {
    m_final_ev(*this, tick);
    if( done )
      m_clock_ev(*this, clock_completed);
  }
  return updated;
}

// manipulators

void clock::on_clock(clock_sig::slot_type const &cb) {
  mtx_lock lock(m_lock);
  m_clock_ev.connect(cb);
  if( m_started && !m_completed )
    cb(*this,clock_started);
}

void clock::start() {
  date_type from, to;
  tick_type now;
  bool done = false;
  {
    mtx_lock lock(m_lock);
    if( m_started && !m_completed )
      return;
    m_started = true;
    m_completed = false;
    m_cur = now = 0;
    m_epoch = do_start();
    done = check_completed();
    from = m_epoch;
    to = to_date_impl(m_final);
  }
  m_clock_ev(*this, clock_started);
  m_tick_ev(*this, now);
  //std::cout<<"Started clock from "<<from<<" to "<<to<<std::endl;
  if( done )
    m_clock_ev(*this, clock_completed);
}

void clock::sleep(sy::error_code &ec) const {
  if( active() ) {
    ec.clear();
    do_sleep(ec);
  } else {
    ec = sy::errc::make_error_code(sy::errc::no_message);
  }
}

clock::tick_type clock::tick() {
  tick_type cur;
  bool done = false;
  {
    mtx_lock lock(m_lock);
    if( m_completed )
      return m_cur;
    else if( m_started ) {
      cur = get_tick();
      if( cur==m_cur )
        return cur;
      m_cur = cur;
      done = check_completed();
    } else
      return zero();
  }
  m_tick_ev(*this, cur);
  if( done )
    m_clock_ev(*this, clock_completed);
  return cur;
}

clock::tick_type clock::current() const {
  mtx_lock lock(m_lock);
  return m_cur;
}


void clock::sleep(ch::nanoseconds &delay, sy::error_code &ec) const {
  if( delay>=ch::nanoseconds::zero() ) {
    timespec tv;
    tv.tv_sec = delay.count()/ch::nanoseconds::period::den;
    tv.tv_nsec = delay.count()%ch::nanoseconds::period::den;
  
    while( tv.tv_sec>0 || tv.tv_nsec>0 ) {
      if( 0==nanosleep(&tv, &tv) ) {
        ec.clear();
        break;
      } else {
        ec = sy::error_code(errno, sy::system_category());
        if( ec!=sy::errc::interrupted )
          break;
      }
    }
  }
}

clock::date_type clock::rt_now() const {
  return pt::microsec_clock::universal_time();
}






