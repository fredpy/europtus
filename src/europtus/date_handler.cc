/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Frederic Py.
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
#include "europtus/date_handler.hh"
#include <trex/utils/TimeUtils.hh>

using namespace europtus;
namespace tr=TREX::transaction;
namespace tu=TREX::utils;

/*
 * class europtus::date_handler
 */

// structors

date_handler::date_handler(tu::Symbol const &tag, clock const &clk)
:base_type(tag), m_clock(clk) {
  base_type::notify();
}


date_handler::~date_handler() {
}


// manipulators

date_handler::base_type::result_type date_handler::produce
(date_handler::base_type::argument_type arg) const {
  opt_date min = tu::parse_attr<opt_date>(arg.second, "min"),
  max = tu::parse_attr<opt_date>(arg.second, "max");
  tr::IntegerDomain::bound lo(tr::IntegerDomain::minus_inf),
  hi(tr::IntegerDomain::plus_inf);
  
  if( min )
    lo = m_clock.to_tick(*min);
  if( max )
    hi = m_clock.to_tick(*max);
  return MAKE_SHARED<tr::IntegerDomain>(lo, hi);
}

/*
 * class europtus::duration_handler
 */

// structors

duration_handler::duration_handler(tu::Symbol const &tag, clock const &clk)
:base_type(tag), m_clock(clk) {
  base_type::notify();
}


duration_handler::~duration_handler() {
}


// manipulators

duration_handler::base_type::result_type duration_handler::produce
(duration_handler::base_type::argument_type arg) const {
  opt_dur min = tu::parse_attr<opt_dur>(arg.second, "min"),
  max = tu::parse_attr<opt_dur>(arg.second, "max");
  tr::IntegerDomain::bound lo(tr::IntegerDomain::minus_inf),
  hi(tr::IntegerDomain::plus_inf);
  
  CHRONO::duration<double> ratio = m_clock.tick_duration();
  typedef tu::chrono_posix_convert< CHRONO::duration<double> > cvt;
  
  if( min ) {
    CHRONO::duration<double> min_s(cvt::to_chrono(*min));
    double val = min_s.count()/ratio.count();
    lo = static_cast<tick_type>(val);
  }
  if( max ) {
    CHRONO::duration<double> max_s(cvt::to_chrono(*max));
    double val = max_s.count()/ratio.count();
    hi = static_cast<tick_type>(val);
  }
  
  return MAKE_SHARED<tr::IntegerDomain>(lo, hi);
}
