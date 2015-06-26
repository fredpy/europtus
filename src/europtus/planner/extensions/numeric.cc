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
#include "europtus/planner/extensions/numeric.hh"

#include <PLASMA/Domains.hh>

using namespace europtus::planner;
namespace eu=EUROPA;

/*
 * class europtus::planner::sqrt_cstr
 */

eu::IntervalDomain const sqrt_cstr::s_pos(0.0, std::numeric_limits<eu::edouble>::infinity());
eu::edouble const sqrt_cstr::s_epsilon(1e-4);


sqrt_cstr::sqrt_cstr(eu::LabelStr const &name,
                     eu::LabelStr const &propagatorName,
                     eu::ConstraintEngineId const &cstr,
                     std::vector<eu::ConstrainedVariableId> const &vars)
:eu::Constraint(name, propagatorName, cstr, vars),
m_square(getCurrentDomain(vars[1])),
m_root(getCurrentDomain(vars[0])) {
}


sqrt_cstr::~sqrt_cstr() {
  
}

void sqrt_cstr::handleExecute() {
  if( m_square.isOpen() && m_root.isOpen() ) {
    m_square.intersect(s_pos);
    m_root.intersect(s_pos);
    return;
  } else if( m_square.isSingleton() ) {
    eu::edouble x2 = m_square.getSingletonValue(), x;
    
    if( x2<0.0 ) {
      m_square.intersect(s_pos);
      return;
    }
    x = std::sqrt(x2);
    if( x2>=std::numeric_limits<eu::edouble>::infinity() )
      m_root.intersect(x, std::numeric_limits<eu::edouble>::infinity());
    else
      m_root.intersect(x,x);
    return;
  } else if( m_root.isSingleton() ) {
    eu::edouble x = m_root.getSingletonValue(), x2;
    
    if( x<0.0 ) {
      m_root.intersect(s_pos);
      return;
    }
    x2 = x*x;
    if( x2>=std::numeric_limits<eu::edouble>::infinity() )
      m_square.intersect(x, std::numeric_limits<eu::edouble>::infinity());
    else
      m_square.intersect(x2,x2);
    return;
  }
  
  eu::edouble x_min, x_max, x2_min, x2_max;
  
  for (bool done=false; !done; ) {
    done = true;
    
    m_square.getBounds(x2_min, x2_max);
    m_root.getBounds(x_min, x_max);
    
    if( x_min<=0.0 )
      x_min = 0.0;
    if( x2_min<=0.0 )
      x2_min = 0.0;
    
    eu::edouble max_x, min_x;

    // upper_bound for x
    if( x2_max>=std::numeric_limits<eu::edouble>::infinity() )
      max_x = x2_max;
    else
      max_x = std::sqrt(x2_max);
    
    if( x_max-s_epsilon > max_x )
      x_max = max_x;
    
    // lower_bound for x
    min_x = std::sqrt(x2_min);
    if( x_min+s_epsilon < min_x )
      x_min = min_x;
    
    // apply new domain
    if( m_root.intersect(x_min, x_max) && m_root.isEmpty() )
      return; // failed
    
    eu::edouble max_x2, min_x2;
    
    // upper_bound for x2
    max_x2 = x_max*x_max;
    if( x2_max-s_epsilon > max_x2 )
      x2_max = max_x2;
    
    // lower_bound for x2
    min_x2 = x_min*x_min;
    if( x2_min+s_epsilon < min_x2 )
      x2_min = min_x2;
    
    // apply new domain
    if( m_square.intersect(x2_min, x2_max) ) {
      if( m_square.isEmpty() )
        return; // failure
      done = false; // need to check if this update affects m_root
    }
  }
}
