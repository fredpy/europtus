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
#include "europtus/planner/extensions/ceil_constraint.hh"
#include <PLASMA/Domains.hh>

using namespace europtus::planner;
namespace eu=EUROPA;

/*
 * class europtus::planner::ceil_constraint
 */

// structors

ceil_constraint::ceil_constraint(eu::LabelStr const &name,
                                 eu::LabelStr const &propagatorName,
                                 eu::ConstraintEngineId const &cstr,
                                 std::vector<eu::ConstrainedVariableId> const &vars)
:eu::Constraint(name, propagatorName, cstr, vars),
  m_ceil(getCurrentDomain(vars[0])),
  m_val(getCurrentDomain(vars[1])) {
  checkError(vars.size()==2, "Exactly 2 parameters required.");
}

ceil_constraint::~ceil_constraint() {}

// manipulators

void ceil_constraint::handleExecute() {
  eu::edouble c_lb, c_ub, v_lb, v_ub;
  eu::edouble const almost_one = 1.0;
  
  // restrict m_ceil based on m_val
  m_val.getBounds(v_lb, v_ub);
  
  if( v_lb<=std::numeric_limits<eu::edouble>::minus_infinity() )
    c_lb = std::numeric_limits<eu::eint>::minus_infinity();
  else
    c_lb = std::ceil(v_lb);
  
  if( v_ub>=std::numeric_limits<eu::edouble>::infinity() )
    c_ub = std::numeric_limits<eu::eint>::infinity();
  else
    c_ub = std::ceil(v_ub);
  
  m_ceil.intersect(c_lb, c_ub);
  
  // constrain m_val based on m_ceil
  m_ceil.getBounds(c_lb, c_ub);
  
  if( c_lb<=std::numeric_limits<eu::eint>::minus_infinity() )
    v_lb = std::numeric_limits<eu::edouble>::minus_infinity();
  else if( (c_lb-almost_one)>v_lb )
    v_lb = c_lb-almost_one;
  

  
  if( c_ub>=std::numeric_limits<eu::eint>::infinity() )
    v_ub = std::numeric_limits<eu::edouble>::infinity();
  else if( v_ub>c_ub )
    v_ub = c_ub;

  m_val.intersect(v_lb, v_ub);
}
