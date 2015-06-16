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
#include <PLASMA/TokenVariable.hh>

#include <trex/europa/bits/europa_convert.hh>


using namespace europtus::planner;

namespace tr=TREX::transaction;
namespace tu=TREX::utils;
namespace te=TREX::europa;
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
//  eu::TokenId master = token->master();
//  std::cerr<<"ADDED: "<<token->getPredicateName().toString()<<std::endl;
  
  // Lets cheat the system that way
//  if( master.isNoId() ) {
//    std::cerr<<"inc ref("<<token->getPredicateName().toString()<<")"<<std::endl;
//    token->incRefCount();
//  }
  
}

void assembly::pimpl::token_proxy::notifyRemoved(const eu::TokenId& token) {
//  std::cerr<<"REMOVED: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifyActivated(const eu::TokenId& token) {
//  std::cerr<<"ACTIVE: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifyDeactivated(const eu::TokenId& token) {
//  std::cerr<<"INACTIVE: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifyMerged(const eu::TokenId& token) {
//  std::cerr<<"MERGE: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifySplit(const eu::TokenId& token) {
//  std::cerr<<"SPLIT: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifyRejected(const eu::TokenId& token) {
//  std::cerr<<"REJECT: "<<token->getPredicateName().toString()<<std::endl;
}

void assembly::pimpl::token_proxy::notifyReinstated(const eu::TokenId& token) {
//  std::cerr<<"REINST: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifyCommitted(const eu::TokenId& token) {
//  std::cerr<<"COMMIT: "<<token->getPredicateName().toString()<<std::endl;
  
}

void assembly::pimpl::token_proxy::notifyTerminated(const eu::TokenId& token) {
//  std::cerr<<"TERM: "<<token->getPredicateName().toString()<<std::endl;
  
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
  
  // and also our own extensions
  m_europtus = (new ModuleEuroptus(this))->getId();
  addModule(m_europtus);
  
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

// observers

bool assembly::pimpl::have_predicate(eu::ObjectId const &object,
                                     std::string &pred) const {
  eu::LabelStr o_type = object->getType();
  if( !schema()->isPredicate(pred.c_str()) ) {
    std::string long_name = o_type.toString()+"."+pred;
    if( schema()->isPredicate(long_name) ) {
      pred = long_name;
    } else
      return false;
  }
  return schema()->canBeAssigned(o_type, pred.c_str());
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
  bool prop_needed = true;
  
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
      // TODO need to track if the plannign changed anything here
    }
  } else {
    std::cerr<<"No solver yet."<<std::endl;
  }
}

void assembly::pimpl::do_step() {
  m_pending = false;
  if( m_planning ) {
    if( m_solver->isExhausted() ) {
      std::cerr<<"Solver is exhausted (depth="<<m_solver->getDepth()
      <<", steps="<<m_solver->getStepCount()<<")"<<std::endl;
      m_planning = false;
      // TODO I need to handle this one way ... do not know how yet
      // m_solver->reset();
    } else {
      if( m_cstr->provenInconsistent() ) {
        std::cerr<<"backjump"<<std::endl;
        m_solver->backjump(1);
        std::cerr<<"~backjump"<<std::endl;
        send_step();
      } else if( m_solver->getOpenDecisions().empty() &&
                !m_cstr->pending() ) {
        m_planning = false;
        // TODO need to ctrack that the planning did anything here
      } else {
        std::cerr<<m_solver->printOpenDecisions()<<std::endl;
        if( m_solver->getDepth()>0 )
          std::cerr<<"last: "<<m_solver->getLastExecutedDecision()<<std::endl;
        
        m_solver->step();
        send_step();
      }
    }
  } else
    std::cerr<<"do_step while not planning"<<std::endl;
}




eu::TokenId assembly::pimpl::new_token(std::string const &object,
                                       std::string pred,
                                       bool is_fact) {
  eu::ObjectId obj = m_plan->getObject(object);
  if( obj.isNoId() )
    throw exception("Undefined object \""+object+"\"");
  if( !have_predicate(obj, pred) )
    throw exception("Object \""+object+"\" do not have predicate \""+pred+"\"");
  
  eu::DbClientId cli = m_plan->getClient();
  eu::TokenId tok = cli->createToken(pred.c_str(), NULL, !is_fact, is_fact);
  
  tok->getObject()->specify(obj->getKey());
  return tok;
}


void assembly::pimpl::add_obs(tr::goal_id g) {
  try {
    eu::TokenId obs = new_token(g->object().str(),
                                g->predicate().str(),
                                true);
    if( obs.isId() ) {
      // TODO populate all the attributes
      std::list<tu::Symbol> attrs;
      g->listAttributes(attrs, true);
      for(std::list<tu::Symbol>::const_iterator i=attrs.begin();
          attrs.end()!=i; ++i) {
        eu::ConstrainedVariableId param = obs->getVariable(i->str());
        if( param.isId() ) {
          tr::Variable var = (*g)[*i];
          try {
            te::details::europa_restrict(param, var.domain());
          } catch(tr::DomainExcept const &de) {
            std::cerr<<"WARNING: "<<g->object()<<"."<<g->predicate()
            <<" failed to constraint to "<<var<<std::endl;
          }
        } else
          std::cerr<<"WARNING: "<<g->object()<<"."<<g->predicate()
          <<" do not have attribute "<<(*i)<<std::endl;
      }
    }
  } catch(exception const &e) {
    std::cerr<<"exception while adding observation:\n"
     <<"  - "<<*g<<'\n'
     <<"  - "<<e.what()<<std::endl;
  }
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
  Error::doThrowExceptions();
  std::ifstream cfg(file.c_str());
  DebugMessage::readConfigFile(cfg);
}


void assembly::pimpl::cfg_solver(std::string file) {
  boost::scoped_ptr<eu::TiXmlElement> xml_cfg(eu::initXml(file.c_str()));
  m_solver = (new eu_s::Solver(m_plan, *xml_cfg))->getId();
}


bool assembly::pimpl::nddl(std::string path, std::string file) {
  // Inject my path to europa
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
    std::cout<<"Updated final tick to "<<final
      <<" ("<<m_clock.to_date(final)<<')'<<std::endl;
    send_step();
  }
}

void assembly::pimpl::tick_updated(clock::tick_type cur) {
  if( cur<=eu::cast_basis(std::numeric_limits<eu::eint>::max()) ) {
    eu::IntervalIntDomain
    future(eu::eint::basis_type(cur),
           std::numeric_limits<eu::eint>::infinity());
    m_cur->restrictBaseDomain(future);
    std::cout<<"Updated tick to "<<cur<<" ("
    <<m_clock.to_date(cur)<<')'<<std::endl;
    send_step();
  }
}
