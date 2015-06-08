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
#include "assembly_impl.hh"
#include "europtus/planner/ModuleEuroptus.hh"

#include <PLASMA/ModuleConstraintEngine.hh>
#include <PLASMA/ModulePlanDatabase.hh>
#include <PLASMA/ModuleRulesEngine.hh>
#include <PLASMA/ModuleTemporalNetwork.hh>
#include <PLASMA/ModuleSolvers.hh>
#include <PLASMA/ModuleNddl.hh>
#include <PLASMA/NddlInterpreter.hh>


using namespace europtus::planner;

namespace eu=EUROPA;
namespace eu_s=eu::SOLVERS;
namespace ch=boost::chrono;


using europtus::clock;

/*
 * class europtus::planner::assembly::pimpl
 */

// statics

boost::filesystem::path const assembly::pimpl::s_europa(EUROPA_HOME"/include");

// structor

assembly::pimpl::pimpl(clock &c):m_clock(c) {
  // Populate europa with desired modules
  addModule((new eu::ModuleConstraintEngine())->getId());
  addModule((new eu::ModuleConstraintLibrary())->getId());
  addModule((new eu::ModulePlanDatabase())->getId());
  addModule((new eu::ModuleRulesEngine())->getId());
  addModule((new eu::ModuleTemporalNetwork())->getId());
  addModule((new eu::ModuleSolvers())->getId());
  addModule((new eu::ModuleNddl())->getId());
  
  // and also outr own extensions
  // addModule((new ModuleEuroptus(*this))->getId());
  
  // complete the intiialization
  doStart();
  
  m_schema = ((eu::Schema *)getComponent("Schema"))->getId();
  m_plan   = ((eu::PlanDatabase *)getComponent("PlanDatabase"))->getId();
  m_cstr   = ((eu::ConstraintEngine *)getComponent("ConstraintEngine"))->getId();
  
  m_cstr->setAutoPropagation(false);
  eu::DomainComparator::setComparator((eu::Schema *)m_schema);
}


assembly::pimpl::~pimpl() {
  doShutdown();
}

// manipulators

eu::ConstrainedVariableId
assembly::pimpl::restict_global(char const *name,
                                char const *type,
                                eu::Domain const &base) {
  eu::ConstrainedVariableId ret;
  eu::DbClientId cli = m_plan->getClient();
  
  if( m_plan->isGlobalVariable(name) ) {
    ret = m_plan->getGlobalVariable(name);
    ret->restrictBaseDomain(base);
  } else {
    ret = cli->createVariable(type, base, name, false);
  }
  return ret;
}

bool assembly::pimpl::nddl(std::string path, std::string file) {
  // First inject the nddl search path
  
  std::cout<<"nddl.includePath = "<<path<<std::endl;
  getLanguageInterpreter("nddl")->getEngine()->getConfig()->setProperty("nddl.includePath", path);
  
  try {
    std::string ret = executeScript("nddl", file, true);
    
    if( !ret.empty() )
      throw exception("Errors while parsing \""+file+"\":\n"+ret);
  } catch(eu::PSLanguageExceptionList &le) {
    std::ostringstream err;
    
    err<<"Errors while parsing \""<<file<<"\":\n"<<le;
    throw exception(err.str());
  } catch(Error const &e) {
    std::ostringstream err;
    
    err<<"Error while parsing \""<<file<<"\":\n"<<e;
    throw exception(err.str());
  }
  return m_cstr->constraintConsistent();
}

void assembly::pimpl::init_clock() {
  
  double
    secs = ch::duration_cast< ch::duration<double> >(m_clock.tick_duration()).count();
  restict_global("TICK_DURATION", eu::FloatDT::NAME().c_str(),
                 eu::IntervalDomain(secs));
  m_cur = restict_global("CUR_DATE", eu::IntDT::NAME().c_str(),
                         eu::IntervalIntDomain());
  m_last = restict_global("FINAL_TICK", eu::IntDT::NAME().c_str(),
                          eu::IntervalIntDomain());

  m_clock.restrict_final(eu::cast_basis(std::numeric_limits<eu::eint>::max()));
}

void assembly::pimpl::final_updated(clock::tick_type final) {
  if( final<=eu::cast_basis(std::numeric_limits<eu::eint>::max()) ) {
    m_last->restrictBaseDomain(eu::IntervalIntDomain(0, final));
    std::cout<<m_last->toLongString()<<std::endl;
  }
}

void assembly::pimpl::tick_updated(clock::tick_type cur) {
  if( cur<=eu::cast_basis(std::numeric_limits<eu::eint>::max()) ) {
    eu::IntervalIntDomain
    future(eu::eint::basis_type(cur),
           std::numeric_limits<eu::eint>::infinity());
    m_cur->restrictBaseDomain(future);
    std::cout<<m_cur->toLongString()<<std::endl;
  }
}