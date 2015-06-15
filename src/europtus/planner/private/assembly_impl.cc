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

/*
 * class europtus::planner::assembly::pimpl::token_proxy
 */

// structors

assembly::pimpl::token_proxy::token_proxy(assembly::pimpl &me)
:eu::PlanDatabaseListener(me.m_plan), m_self(me) {}

assembly::pimpl::token_proxy::~token_proxy() {}


// callbacks

void assembly::pimpl::token_proxy::notifyAdded(const eu::TokenId& token) {
  eu::TokenId master = token->master();
  std::cerr<<"ADDED: "<<token->getPredicateName().toString()<<std::endl;
  
  // Lets cheat the system that way
//  if( master.isNoId() ) {
//    std::cerr<<"inc ref("<<token->getPredicateName().toString()<<")"<<std::endl;
//    token->incRefCount();
//  }
  
}

void assembly::pimpl::token_proxy::notifyRemoved(const eu::TokenId& token) {
  std::cerr<<"REMOVED: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifyActivated(const eu::TokenId& token) {
  std::cerr<<"ACTIVE: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifyDeactivated(const eu::TokenId& token) {
  std::cerr<<"INACTIVE: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifyMerged(const eu::TokenId& token) {
  std::cerr<<"MERGE: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifySplit(const eu::TokenId& token) {
  std::cerr<<"SPLIT: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifyRejected(const eu::TokenId& token) {
  std::cerr<<"REJECT: "<<token->getPredicateName().toString()<<std::endl;
}

void assembly::pimpl::token_proxy::notifyReinstated(const eu::TokenId& token) {
  std::cerr<<"REINST: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifyCommitted(const eu::TokenId& token) {
  std::cerr<<"COMMIT: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifyTerminated(const eu::TokenId& token) {
  std::cerr<<"TERM: "<<token->getPredicateName().toString()<<std::endl;
  
}



namespace ch=boost::chrono;
using europtus::clock;


/*
 * class europtus::planner::assembly::pimpl
 */

// statics

boost::filesystem::path const assembly::pimpl::s_europa(EUROPA_HOME"/include");

// structor

assembly::pimpl::pimpl(clock &c)
  :m_clock(c),m_planning(false),m_pending(false) {
   
  // Populate europa with desired modules
  addModule((new eu::ModuleConstraintEngine())->getId());
  addModule((new eu::ModuleConstraintLibrary())->getId());
  addModule((new eu::ModulePlanDatabase())->getId());
  addModule((new eu::ModuleRulesEngine())->getId());
  addModule((new eu::ModuleTemporalNetwork())->getId());
  addModule((new eu::ModuleSolvers())->getId());
  addModule((new eu::ModuleNddl())->getId());
  
  // and also outr own extensions
  // m_europtus = (new ModuleEuroptus(this))->getId();
  // addModule(m_europtus);
  
  // complete the intiialization
  doStart();
  
  m_schema = ((eu::Schema *)getComponent("Schema"))->getId();
  m_cstr   = ((eu::ConstraintEngine *)getComponent("ConstraintEngine"))->getId();
  m_plan   = ((eu::PlanDatabase *)getComponent("PlanDatabase"))->getId();
  
  m_cstr->setAutoPropagation(false);
  eu::DomainComparator::setComparator((eu::Schema *)m_schema);
  
  m_proxy.reset(new token_proxy(*this));
}


assembly::pimpl::~pimpl() {
  // make sure that my module is removed before this class is destroyed
  m_proxy.reset();
  removeModule(m_europtus);
  doShutdown();
}

// manipulators

void assembly::pimpl::send_step() {
  boost::weak_ptr<pimpl> me(shared_from_this());
  details::europa_protect::strand().send(boost::bind(&pimpl::async_exec,
                                                     me,
                                                     &pimpl::check_planning),
                                         assembly::plan_p);
}


void assembly::pimpl::check_planning() {
//  std::cerr<<"Check planning"<<std::endl;
  bool prop_needed = true;
//  std::cerr<<"check_planning: "<<boost::this_thread::get_id()<<std::endl;
  
//  std::cout<<"xCstr: pending="<<m_cstr->pending()
//  <<", consistent="<<m_cstr->constraintConsistent()
//  <<", inconsistent="<<m_cstr->provenInconsistent()
//  <<", can_do="<<m_cstr->canContinuePropagation()<<std::endl;
  
  if( m_cstr->provenInconsistent() )
    prop_needed = m_cstr->canContinuePropagation();
  
  if( prop_needed && m_cstr->pending() ) {
    bool ret = m_cstr->propagate();
    if( !ret )
      std::cerr<<"Constraint inconsistency."<<std::endl;
  }
  if( m_solver.isId() ) {
    if( m_cstr->provenInconsistent() || m_cstr->pending() ||
       !( m_solver->noMoreFlaws() && m_solver->getOpenDecisions().empty() ) ) {
      if( !m_planning ) {
//        std::cerr<<"Start planning"<<std::endl;
        m_planning = true;
      }
      
      if( !m_pending ) {
        boost::weak_ptr<pimpl> me(shared_from_this());

        m_pending = true;
        prot::strand().send(boost::bind(&pimpl::async_exec,
                                        me,
                                        &pimpl::do_step),
                            assembly::plan_p);
      }
    } else if( m_planning ) {
      m_planning = false;
     std::cerr<<"End planning:\n"<<m_plan->toString()<<std::endl;
    }
  } else {
    std::cerr<<"No solver yet."<<std::endl;
  }
}

void assembly::pimpl::do_step() {
//  std::cerr<<"do_step: "<<boost::this_thread::get_id()<<std::endl;
  m_pending = false;
//  std::cout<<"cstr: pending="<<m_cstr->pending()
//  <<", consistent="<<m_cstr->constraintConsistent()
//  <<", inconsistent="<<m_cstr->provenInconsistent()
//  <<", can_do="<<m_cstr->canContinuePropagation()<<std::endl;
  if( m_planning ) {
    if( m_solver->isExhausted() ) {
      std::cerr<<"Solver is exhausted (depth="<<m_solver->getDepth()
      <<", steps="<<m_solver->getStepCount()<<")"<<std::endl;
      m_planning = false;
      // m_solver->reset();
    } else {
      if( m_cstr->provenInconsistent() ) {
        std::cerr<<"backjump"<<std::endl;
        m_solver->backjump(1);
        std::cerr<<"~backjump"<<std::endl;
        send_step();
      } else if( m_solver->getOpenDecisions().empty() &&
                !m_cstr->pending() ) {
        std::cerr<<"No more flaws"<<std::endl;
        m_planning = false;
        std::cerr<<"Plan:\n"<<m_plan->toString()
        <<"\n==================================================="<<std::endl;
      } else {
        std::cerr<<m_solver->printOpenDecisions()<<std::endl;
        if( m_solver->getDepth()>0 )
          std::cerr<<"last: "<<m_solver->getLastExecutedDecision()<<std::endl;
        
        m_solver->step();
//        std::cerr<<"step: count="<<m_solver->getStepCount()
//                 <<", depth="<<m_solver->getDepth()<<std::endl;
        send_step();
      }
    }
  } else
    std::cerr<<"do_step while not planning"<<std::endl;
}

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

void assembly::pimpl::set_log(std::ostream &log) {
  DebugMessage::setStream(log);
}



void assembly::pimpl::debug_cfg(std::string file) {
  std::cerr<<"debug_cfg"<<std::endl;
  Error::doThrowExceptions();
  std::ifstream cfg(file.c_str());
  DebugMessage::readConfigFile(cfg);
  std::cerr<<"~debug_cfg"<<std::endl;
}


void assembly::pimpl::cfg_solver(std::string file) {
  std::cerr<<"cfg_solver"<<std::endl;
  boost::scoped_ptr<eu::TiXmlElement> xml_cfg(eu::initXml(file.c_str()));
  m_solver = (new eu_s::Solver(m_plan, *xml_cfg))->getId();
  std::cerr<<"~cfg_solver"<<std::endl;
}


bool assembly::pimpl::nddl(std::string path, std::string file) {
  // First inject the nddl search path
  std::cerr<<"nddl"<<std::endl;
  
  std::cout<<"  nddl.includePath = "<<path<<std::endl;
  getLanguageInterpreter("nddl")->getEngine()->getConfig()->setProperty("nddl.includePath", path);
  
  try {
    std::string ret = executeScript("nddl", file, true);
    
    if( !ret.empty() )
      throw exception("Errors while parsing \""+file+"\":\n"+ret);
    if( !m_plan->isClosed() ) {
      std::cerr<<"Closing plan db"<<std::endl;
      m_plan->close();
    }
    if( m_cstr->pending() )
      m_cstr->propagate();
    
    send_step();
    
    
    std::cerr<<"~nddl"<<std::endl;
    return m_cstr->constraintConsistent();
  } catch(eu::PSLanguageExceptionList &le) {
    std::ostringstream err;
    
    err<<"Errors while parsing \""<<file<<"\":\n"<<le;
    throw exception(err.str());
  } catch(Error const &e) {
    std::ostringstream err;
    
    err<<"Error while parsing \""<<file<<"\":\n"<<e;
    throw exception(err.str());
  }
}

void assembly::pimpl::init_clock() {
  
  double
    secs = ch::duration_cast< ch::duration<double> >(m_clock.tick_duration()).count();
  eu::ConstrainedVariableId dur = restict_global("TICK_DURATION",
                                                 eu::FloatDT::NAME().c_str(),
                                                 eu::IntervalDomain(secs));
  std::cout<<dur->toLongString()<<std::endl;
  m_cur = restict_global("CUR_DATE", eu::IntDT::NAME().c_str(),
                         eu::IntervalIntDomain());
  m_last = restict_global("FINAL_TICK", eu::IntDT::NAME().c_str(),
                          eu::IntervalIntDomain());

  m_clock.restrict_final(eu::cast_basis(std::numeric_limits<eu::eint>::max()));
  send_step();
}

void assembly::pimpl::final_updated(clock::tick_type final) {
  if( final<=eu::cast_basis(std::numeric_limits<eu::eint>::max()) ) {
    m_last->restrictBaseDomain(eu::IntervalIntDomain(0, final));
    std::cout<<m_last->toLongString()
      <<" - base:"<<m_last->baseDomain().toString()<<std::endl;
    send_step();
  }
}

void assembly::pimpl::tick_updated(clock::tick_type cur) {
  if( cur<=eu::cast_basis(std::numeric_limits<eu::eint>::max()) ) {
    eu::IntervalIntDomain
    future(eu::eint::basis_type(cur),
           std::numeric_limits<eu::eint>::infinity());
    m_cur->restrictBaseDomain(future);
    std::cout<<m_cur->toLongString()<<" - base: "<<m_cur->baseDomain().toString()<<std::endl;
    send_step();
  }
}
