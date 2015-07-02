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
#include <europtus/planner/extensions/dispatchable.hh>
#include <europtus/planner/extensions/decision_point.hh>
#include <europtus/planner/extensions/earliest_first.hh>
#include <europtus/planner/extensions/do_not_match.hh>
#include <europtus/planner/extensions/fact_filters.hh>
#include <europtus/planner/extensions/numeric.hh>
#include <europtus/planner/extensions/trigonometry.hh>
#include <trex/lsts/EuropaExtensions.hh>

#include <PLASMA/CFunctions.hh>
#include <PLASMA/UnboundVariableDecisionPoint.hh>
#include <PLASMA/ValueSource.hh>
#include <PLASMA/Token.hh>


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
namespace eu=EUROPA;

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
      return m_choiceIndex<m_my_choices.size();
    }
    EUROPA::edouble getNext() {
      eu::edouble ret = m_my_choices[m_choiceIndex++];
      debugMsg("trex:to_zero", "next choice: "<<m_flawedVariable->toString()
               <<" <- "<<ret<<" ("<<m_choiceIndex<<" of "<<m_my_choices.size()<<")");
      return ret;
    }
    
  private:
    std::vector<eu::edouble> m_my_choices;
    
    void handleInitialize() {
      eu::edouble zero(0.0);
      eu::Domain const &dom = m_flawedVariable->lastDomain();

      if( dom.isMember(zero) )
        m_my_choices.push_back(zero);
      eu::edouble lo, hi;
      dom.getBounds(lo, hi);
      
      if( dom.isSingleton() ) {
        m_my_choices.push_back(lo);
      } if( dom.areBoundsFinite() ) {
        if( lo>zero ) {
          m_my_choices.push_back(lo);
          m_my_choices.push_back(hi);
        } else if( hi<zero ) {
          m_my_choices.push_back(hi);
          m_my_choices.push_back(lo);
        } else {
          eu::edouble nlo = -lo;
          
          if( nlo<hi ) {
            if( lo!=zero )
              m_my_choices.push_back(lo);
            m_my_choices.push_back(hi);
          } else {
            if( hi!=zero )
              m_my_choices.push_back(hi);
            m_my_choices.push_back(lo);
          }
        }
      } else if( lo > std::numeric_limits<eu::edouble>::minus_infinity() )
        m_my_choices.push_back(lo);
      else if( hi < std::numeric_limits<eu::edouble>::infinity() )
        m_my_choices.push_back(hi);
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
  
  REGISTER_CONSTRAINT(ceSchema, dispatchable,
                      "dispatchable", "europtus");

  
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
  
  REGISTER_FLAW_FILTER(cfm, fact_filter, FactFilter);
  REGISTER_FLAW_FILTER(cfm, do_not_match, doNotMatch);
  REGISTER_FLAW_HANDLER(cfm, TowardZero, toZero);
  REGISTER_FLAW_HANDLER(cfm, threat_decision_pt, EuroptusThreatHandler);
  REGISTER_FLAW_MANAGER(cfm, earliest_first, EarliestFirst);

}

void ModuleEuroptus::uninitialize(EngineId engine) {
}
