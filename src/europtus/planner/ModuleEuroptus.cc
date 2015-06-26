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
#include "europtus/planner/ModuleEuroptus.hh"

#include "private/assembly_impl.hh"
#include <europtus/planner/extensions/ceil_constraint.hh>
#include <europtus/planner/extensions/deg_to_rad.hh>
#include <europtus/planner/extensions/do_not_match.hh>
#include <europtus/planner/extensions/numeric.hh>
#include <europtus/planner/extensions/trigonometry.hh>
#include <trex/lsts/EuropaExtensions.hh>

#include <PLASMA/CFunctions.hh>
#include <PLASMA/UnboundVariableDecisionPoint.hh>
#include <PLASMA/ValueSource.hh>


using namespace EUROPA;
namespace eu_s=EUROPA::SOLVERS;

namespace TREX {
  namespace LSTS {
    namespace  {
      DECLARE_FUNCTION_TYPE(LatLonDist, ll_dist, "ll_distance", FloatDT, 4);
    }
  }
}

using namespace europtus::planner;

namespace {
  DECLARE_FUNCTION_TYPE(ceil_constraint, ceil, "ceilf",
                        EUROPA::IntDT, 1);
  DECLARE_FUNCTION_TYPE(deg_to_rad, to_rad, "deg_to_rad_f",
                        EUROPA::FloatDT, 1);
  DECLARE_FUNCTION_TYPE(sine_cstr, sin, "sinf",
                        EUROPA::FloatDT, 1);
  DECLARE_FUNCTION_TYPE(cosine_cstr, cos, "cosf",
                        EUROPA::FloatDT, 1);
  DECLARE_FUNCTION_TYPE(sqrt_cstr, sqrt, "sqrtf",
                        EUROPA::FloatDT, 1);
  
  
  class TowardZero: public eu_s::UnboundVariableDecisionPoint {
  public:
    TowardZero(EUROPA::DbClientId const &client,
               EUROPA::ConstrainedVariableId const &flawed_var,
               EUROPA::TiXmlElement const &config,
               EUROPA::LabelStr const &explanation = "unknown")
    :eu_s::UnboundVariableDecisionPoint(client, flawed_var, config, explanation),
    m_choiceIndex(0) {}
    
    bool hasNext() const {
      return m_choiceIndex < m_choices->getCount();
    }
    EUROPA::edouble getNext() {
      EUROPA::edouble ret = choice(m_choiceIndex++);
      debugMsg("trex:to_zero", "next choice: "<<m_flawedVariable->toString()
               <<" <- "<<ret<<" ("<<m_choiceIndex<<")");
      return ret;
    }
    
  private:
    EUROPA::edouble choice(unsigned int idx) {
      EUROPA::Domain const &dom = m_flawedVariable->lastDomain();
      EUROPA::edouble zero(0.0);
      
      if( dom.isInterval() ) {
        EUROPA::edouble lo = dom.getLowerBound(),
        hi = dom.getUpperBound(),
        steps = dom.minDelta();
        if( lo<=zero ) {
          if( hi<=zero )
            idx = m_choices->getCount() - (idx+1);
          else {
            EUROPA::Domain::size_type
            lo_cut = EUROPA::cast_int((zero-lo)/steps),
            hi_cut = EUROPA::cast_int(hi/steps),
            half_idx = (idx+1)/2;
            if( half_idx>lo_cut ) {
              // exhausted all the choices below zero
              idx -= lo_cut;
              return steps*idx;
            } else if( half_idx>hi_cut ) {
              // exhausted all the choices above zero
              idx -= hi_cut;
              return zero-(steps*idx);
            } else if( idx&1 )
              return steps*half_idx;
            else
              return zero-(steps*half_idx);
          }
        }
      }
      
      return m_choices->getValue(idx);
    }
    unsigned int m_choiceIndex;
  };

}

namespace lsts=TREX::LSTS;




/*
 * class europtus::planner::ModuleEuroptus
 */
ModuleEuroptus::ModuleEuroptus(assembly::pimpl *ref)
  :Module("Europtus"),m_assembly(ref) {}

ModuleEuroptus::~ModuleEuroptus() {
}

void ModuleEuroptus::initialize() {
}

void ModuleEuroptus::uninitialize() {
  m_assembly = NULL;
}

void ModuleEuroptus::initialize(EngineId engine) {
 ConstraintEngine* ce = (ConstraintEngine*)engine->getComponent("ConstraintEngine");
  CESchema* ceSchema = (CESchema*)engine->getComponent("CESchema");
  
  REGISTER_CONSTRAINT(ceSchema, lsts::LatLonDist,
                      "ll_distance", "Default");
  ce->getCESchema()->registerCFunction((new lsts::LatLonDistFunction())->getId());

  REGISTER_CONSTRAINT(ceSchema, lsts::LatLonDisplace,
                      "ll_displace", "Default");

  REGISTER_CONSTRAINT(ceSchema,
                      ceil_constraint,
                      "ceilf", "Default");
  ce->getCESchema()->registerCFunction((new ceil_constraintFunction())->getId());
  REGISTER_CONSTRAINT(ceSchema,
                      deg_to_rad,
                      "deg_to_rad_f", "Default");
  ce->getCESchema()->registerCFunction((new deg_to_radFunction())->getId());

  REGISTER_CONSTRAINT(ceSchema,
                      cosine_cstr,
                      "cosf", "Default");
  ce->getCESchema()->registerCFunction((new cosine_cstrFunction())->getId());

  REGISTER_CONSTRAINT(ceSchema,
                      sine_cstr,
                      "sinf", "Default");
  ce->getCESchema()->registerCFunction((new sine_cstrFunction())->getId());

  REGISTER_CONSTRAINT(ceSchema,
                      sqrt_cstr,
                      "sqrtf", "Default");
  ce->getCESchema()->registerCFunction((new sqrt_cstrFunction())->getId());
  
  eu_s::ComponentFactoryMgr *cfm =  (eu_s::ComponentFactoryMgr *)engine->getComponent("ComponentFactoryMgr");
  
  REGISTER_FLAW_FILTER(cfm, do_not_match, doNotMatch);
  REGISTER_FLAW_HANDLER(cfm, TowardZero, toZero);
}

void ModuleEuroptus::uninitialize(EngineId engine) {
}
