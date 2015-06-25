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
namespace tlog=tu::log;
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
}

void assembly::pimpl::token_proxy::notifyRemoved(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifyActivated(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifyDeactivated(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifyMerged(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifySplit(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifyRejected(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifyReinstated(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifyCommitted(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifyTerminated(const eu::TokenId& token) {
}



namespace ch=boost::chrono;
using europtus::clock;


/*
 * class europtus::planner::assembly::pimpl
 */

// statics

boost::filesystem::path const assembly::pimpl::s_europa(EUROPA_HOME"/include");

// structor

assembly::pimpl::pimpl(clock &c, tlog::text_log &log)
  :m_lost(0), m_clock(c),m_planning(false),m_pending(false),
  m_log(log) {
   
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


bool assembly::pimpl::is_fact(eu::TokenId const &tok,
                             bool or_merged_to) const {
  if( tok.isId() ) {
    bool fact = tok->isFact();
    
    if( !fact && or_merged_to ) {
      if( tok->isActive() ) {
        eu::TokenSet merged = tok->getMergedTokens();
        
        for(eu::TokenSet::const_iterator m=merged.begin();
            merged.end()!=m; ++m) {
          if( (*m)->isFact() )
            return true;
        }
      } else if( tok->isMerged() ) {
        eu::TokenId real = tok->getActiveToken();
        return is_fact(real);
      }
    }
    return fact;
  }
  return false;
}


// manipulators

tlog::stream assembly::pimpl::log(tlog::id_type const &what) const {
  if( m_clock.started() ) {
    m_last_log = m_clock.current();
    return m_log.msg(*m_last_log, tlog::null, what);
  } else
    return m_log.msg(tlog::null, what);
}



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
      log()<<"Constraint inconsistency.";
  }
  if( m_solver.isId() ) {
    if( m_cstr->provenInconsistent() || m_cstr->pending() ||
       !( m_solver->noMoreFlaws() && m_solver->getOpenDecisions().empty() ) ) {
      if( !m_planning ) {
        m_planning = true;
        m_steps = m_solver->getStepCount()+m_lost;
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
      end_plan();
    }
  } else {
    log(tlog::error)<<"No solver yet.";
  }
}

void assembly::pimpl::do_step() {
  m_pending = false;
  if( m_planning ) {
    if( m_solver->isExhausted() ) {
      log(tlog::error)<<"Solver is exhausted (depth="<<m_solver->getDepth()
      <<", steps="<<(m_solver->getStepCount()+m_lost)<<")";
      m_planning = false;
      // TODO I need to handle this one way ... do not know how yet
      // m_solver->reset();
    } else {
      if( m_cstr->provenInconsistent() ) {
        size_t steps = m_solver->getStepCount(), bsteps;
        m_solver->backjump(1);
        bsteps = m_solver->getStepCount();
        if( steps>=bsteps )
          m_lost += steps-bsteps;
        send_step();
      } else if( m_solver->getOpenDecisions().empty() &&
                !m_cstr->pending() ) {
        end_plan();
      } else {
        log()<<m_solver->printOpenDecisions()<<std::endl;
        if( m_solver->getDepth()>0 )
          log()<<"last decision: "<<m_solver->getLastExecutedDecision();
        
        m_solver->step();
        send_step();
      }
    }
  } else
    log(tlog::warn)<<"do_step while not planning";
}


void assembly::pimpl::end_plan() {
  size_t steps = m_solver->getStepCount()+m_lost;
  
  if( steps!=m_steps ) {
    log()<<"Planning completed after "
      <<(steps-m_steps)<<" steps:\n"
    <<"  - steps="<<steps<<"\n"
    <<"  - depth="<<m_solver->getDepth()<<"\n\n"
    <<"==============================================================\n"
    <<m_plan->toString()
    <<"\n=============================================================="
    <<std::endl;
    m_steps = steps;
  }
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
  eu::TokenId tok = cli->createToken(pred.c_str(), NULL,
                                     !is_fact, // a non fact is rejectable
                                     is_fact);
  
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
            log(tlog::warn)<<g->object()<<"."<<g->predicate()
            <<" failed to constraint to "<<var<<std::endl;
          }
        } else
          log(tlog::warn)<<g->object()<<"."<<g->predicate()
          <<" do not have attribute "<<(*i)<<std::endl;
      }
    }
  } catch(exception const &e) {
    log(tlog::error)<<"exception while adding observation:\n"
     <<"  - "<<*g<<'\n'
     <<"  - "<<e.what()<<std::endl;
  }
}

void assembly::pimpl::add_goal(tr::goal_id g) {
  // First restrict the goal to be in the scope of planning
  tr::IntegerDomain future;
  clock::tick_type cur = m_clock.tick(), last = m_clock.final();
  
  try {
    tr::IntegerDomain window(cur+1, last);

    g->restrictStart(window);
    
  } catch(tr::EmptyDomain const &e) {
    log(tlog::warn)<<"Ignore goal "<<(*g)<<":\n"
      <<" - it cannot start between "<<(cur+1)
      <<" ("<<m_clock.to_date(cur+1)<<") and "<<last
      <<" ("<<m_clock.to_date(last)<<")"<<std::endl;
    return;
  }
  
  
  try {
    eu::TokenId req = new_token(g->object().str(),
                                g->predicate().str(),
                                false); // add the token as a non fact
                                        // non facts are considered as
                                        // rejectable goals in this code
    if( req.isId() ) {
      // TODO populate all the attributes
      std::list<tu::Symbol> attrs;
      g->listAttributes(attrs, true);
      for(std::list<tu::Symbol>::const_iterator i=attrs.begin();
          attrs.end()!=i; ++i) {
        eu::ConstrainedVariableId param = req->getVariable(i->str());
        if( param.isId() ) {
          tr::Variable var = (*g)[*i];
          try {
            te::details::europa_restrict(param, var.domain());
          } catch(tr::DomainExcept const &de) {
            log(tlog::warn)<<g->object()<<"."<<g->predicate()
            <<" failed to constraint to "<<var<<std::endl;
          }
        } else
          log(tlog::warn)<<g->object()<<"."<<g->predicate()
          <<" do not have attribute "<<(*i)<<std::endl;
      }
      log()<<"Posted goal "<<(*g)<<std::endl;
    } else
      log(tlog::warn)<<"Failed to create token "<<g->object()<<"."<<g->predicate()
      <<std::endl;
  } catch(exception const &e) {
    log(tlog::error)<<"exception while adding goal:\n"
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
      log()<<"Closing plan db";
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
  restict_global("TICK_DURATION", eu::FloatDT::NAME().c_str(),
                 eu::IntervalDomain(secs));
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
    log()<<"Updated final tick to "<<final
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
    if( 0==(cur%10) && m_last_log &&  (5+*m_last_log)<cur ) {
        log()<<"Updated tick (still alive: "<<m_clock.to_date(cur)<<")";
    }
    send_step();
  }
}
