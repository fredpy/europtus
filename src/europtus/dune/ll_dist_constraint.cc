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
#include "europtus/dune/ll_dist_constraint.hh"
#include <PLASMA/Domains.hh>

#include <DUNE/Coordinates/WGS84.hpp>

using namespace europtus::dune;
namespace eu=EUROPA;

namespace
{
  
  bool
  intersect(eu::Domain& dom, eu::edouble lb, eu::edouble ub,
            double precision_error)
  {
    if (lb > dom.getUpperBound()
        && lb < std::numeric_limits<EUROPA::edouble>::infinity())
      lb -= precision_error;
    
    if (ub < dom.getLowerBound()
        && std::numeric_limits<EUROPA::edouble>::minus_infinity()<ub)
      ub += precision_error;
    
    return dom.intersect(lb, ub);
  }
}

using DUNE::Coordinates::WGS84;


/*
 * class europtus::dune::ll_dist_constraint
 */

// structors

ll_dist_constraint::ll_dist_constraint(eu::LabelStr const &name,
                                       eu::LabelStr const &propagatorName,
                                       eu::ConstraintEngineId const &cstr,
                                       std::vector<eu::ConstrainedVariableId> const &vars)
:eu::Constraint(name, propagatorName, cstr, vars),
  m_dist(getCurrentDomain(m_variables[DIST])),
  m_lat1(getCurrentDomain(m_variables[LAT1])),
  m_lon1(getCurrentDomain(m_variables[LON1])),
  m_lat2(getCurrentDomain(m_variables[LAT2])),
m_lon2(getCurrentDomain(m_variables[LON2])) {
  
}

ll_dist_constraint::~ll_dist_constraint() {}

// manipulators

void ll_dist_constraint::handleExecute() {
  double lat1, lon1, lat2, lon2;
  
  //  if (m_dist.isSingleton())
  //  {
  //    // While generally incomplete this approach avoid to
  //    // compute the same distance over and over again ... and
  //    // it should work as long as nobody constraints dist
  //    // otherwise
  //    return;
  //  }
  
  if (m_lat1.isSingleton())
    lat1 = cast_basis(m_lat1.getSingletonValue());
  else
    return;
  if (m_lon1.isSingleton())
    lon1 = cast_basis(m_lon1.getSingletonValue());
  else
    return;
  
  if (m_lat2.isSingleton())
    lat2 = cast_basis(m_lat2.getSingletonValue());
  else
    return;
  if (m_lon2.isSingleton())
    lon2 = cast_basis(m_lon2.getSingletonValue());
  else
    return;
  
  EUROPA::edouble dist = WGS84::distance(lat1, lon1, 0, lat2, lon2, 0);
  intersect(m_dist, dist, dist, 0.5);
}

