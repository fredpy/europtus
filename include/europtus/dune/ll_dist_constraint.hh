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
#ifndef H_europtus_dune_ll_dist_constraint
# define H_europtus_dune_ll_dist_constraint

# include "europtus/planner/bits/europa_cfg.hh"
# include <PLASMA/Constraint.hh>


namespace europtus {
  namespace dune {
    
    class ll_dist_constraint:public EUROPA::Constraint {
    public:
      ll_dist_constraint(EUROPA::LabelStr const &name,
                      EUROPA::LabelStr const &propagatorName,
                      EUROPA::ConstraintEngineId const &cstr,
                      std::vector<EUROPA::ConstrainedVariableId> const &vars);
      ~ll_dist_constraint();
      
    private:
      void handleExecute();
      
      EUROPA::Domain &m_dist;
      EUROPA::Domain &m_lat1;
      EUROPA::Domain &m_lon1;
      EUROPA::Domain &m_lat2;
      EUROPA::Domain &m_lon2;
      
      enum indexes {
        DIST = 0,
        LAT1 = 1,
        LON1 = 2,
        LAT2 = 3,
        LON2 = 4,
        NARGS = 5
      };
    }; // europtus::dune::ll_dist_constraint
    
  } // europtus::dune
} // europtus

#endif // H_europtus_dune_ll_dist_constraint