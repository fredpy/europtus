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
#include "europtus/planner/propagator.hh"

#include <PLASMA/ModuleConstraintEngine.hh>
#include <PLASMA/ModulePlanDatabase.hh>
#include <PLASMA/ModuleRulesEngine.hh>
#include <PLASMA/ModuleTemporalNetwork.hh>
#include <PLASMA/ModuleSolvers.hh>
#include <PLASMA/ModuleNddl.hh>
#include <PLASMA/NddlInterpreter.hh>
#include <PLASMA/TokenVariable.hh>
#include <PLASMA/Timeline.hh>

#include <trex/europa/bits/europa_convert.hh>


using namespace europtus::planner;

namespace tr=TREX::transaction;
namespace tu=TREX::utils;
namespace tlog=tu::log;
namespace te=TREX::europa;
namespace eu=EUROPA;
namespace eu_s=eu::SOLVERS;

namespace {
  
  std::string const implicit_s("implicit_");
  
  std::ostream &print_variable(std::ostream &out, std::string const &name,
                               eu::Domain const &dom) {
    out<<name;
    if( dom.isSingleton() )
      out<<"="<<dom.toString(dom.getSingletonValue());
    else if( dom.areBoundsFinite() )
      out<<"="<<dom.toString();
    return out;
  }
  
  std::ostream &print_token(std::ostream &out, eu::Token const &tok) {
    out<<'['<<tok.getKey()<<"] "<<tok.getObject()->toString()<<"."
      <<tok.getUnqualifiedPredicateName().toString()<<"(";
    
    typedef std::vector<eu::ConstrainedVariableId> var_set;
    var_set vars = tok.parameters();
    bool first = true;
    
    if( tok.start()->lastDomain().areBoundsFinite() ) {
      print_variable(out, "start", tok.start()->lastDomain());
      first = false;
    }
    if( tok.end()->lastDomain().areBoundsFinite() ) {
      if( first )
        first = false;
      else
        out<<", ";
      print_variable(out, "end", tok.end()->lastDomain());
    }
    
    for(var_set::const_iterator i=vars.begin(); vars.end()!=i; ++i) {
      std::string name = (*i)->getName().toString();
      if( name.compare(0, implicit_s.size(), implicit_s)!=0 ) {
        if( first )
          first = false;
        else
          out<<", ";
        print_variable(out, name, (*i)->lastDomain());
      }
    }
    return out<<")";
  }
  
  tu::Symbol s_justify("just");
  tu::Symbol s_unjustify("unjust");
  
}

/*
 * class europtus::planner::assembly::pimpl::token_proxy
 */

// structors

assembly::pimpl::token_proxy::token_proxy(assembly::pimpl &me)
:eu::PlanDatabaseListener(me.m_plan), m_self(me) {
}

assembly::pimpl::token_proxy::~token_proxy() {
}


// callbacks

void assembly::pimpl::token_proxy::notifyAdded(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifyRemoved(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifyActivated(const eu::TokenId& token) {
  if( token->isFact() )
    m_self.justify(token, token);
  else {
    if( m_self.is_condition(token) ) {
      eu::TokenId master = token->master();
      if( m_self.is_action(master) && m_self.justified(master) )
        m_self.justify(token, master);
    } else if( m_self.is_action(token) ) {
      m_self.m_guarded.insert(token);
    }
  }
}

void assembly::pimpl::token_proxy::notifyDeactivated(const eu::TokenId& token) {
  m_self.unjustify(token);
  if( m_self.is_action(token) )
    m_self.m_guarded.erase(token);
}

void assembly::pimpl::token_proxy::notifyMerged(const eu::TokenId& token) {
  eu::TokenId me = token->getActiveToken();

  if( token->isFact() ) {
    m_self.justify(token, token);
  } else {
    if( m_self.is_condition(token) ) {
      eu::TokenId master = token->master();
      if( m_self.is_action(master) && m_self.justified(master) ) {
        m_self.justify(token, master);
      }
    }
  }
}

void assembly::pimpl::token_proxy::notifySplit(const eu::TokenId& token) {
  m_self.unjustify(token);
}

void assembly::pimpl::token_proxy::notifyRejected(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifyReinstated(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifyCommitted(const eu::TokenId& token) {
}

void assembly::pimpl::token_proxy::notifyTerminated(const eu::TokenId& token) {
}


/*
 * class europtus::planner::assembly::pimpl::propagator
 */

propagator::propagator(assembly::pimpl &me,
                       eu::LabelStr const &name,
                       eu::ConstraintEngineId const &cstr)
:eu::DefaultPropagator(name, cstr), m_self(me) {}


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
  
  m_propagator = (new propagator(*this, eu::LabelStr("europtus"), m_cstr))->getId();
    
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


bool assembly::pimpl::justified(eu::TokenId tok) const {
  if( tok->isFact() )
    return true;
  eu::TokenId me = tok;
  if( tok->isMerged() )
    me = tok->getActiveToken();

  token_map::left_const_iterator from, to;
  boost::tie(from, to) = m_justified.left.equal_range(me);
  return from!=to;
}

bool assembly::pimpl::is_action(eu::TokenId const &tok) const {
  return tok.isId() && tok->hasAttributes(eu::PSTokenType::ACTION);
}

bool assembly::pimpl::is_predicate(eu::TokenId const &tok) const {
  return tok.isId() && tok->hasAttributes(eu::PSTokenType::PREDICATE);
}

bool assembly::pimpl::is_condition(eu::TokenId  const &tok) const {
  return tok.isId() && tok->hasAttributes(eu::PSTokenType::CONDITION);
}

bool assembly::pimpl::is_effect(eu::TokenId const &tok) const {
  return tok.isId() && tok->hasAttributes(eu::PSTokenType::EFFECT);
}

void assembly::pimpl::effect_for(eu::TokenId const &tok,
                                 eu::TokenSet &actions,
                                 bool recurse) const {
  if( is_effect(tok) )
    actions.insert(tok->master());
  if( tok->isMerged() && recurse )
    effect_for(tok->getActiveToken(), actions, false);
  else if( tok->isActive() ) {
    eu::TokenSet const &merged = tok->getMergedTokens();
    for(eu::TokenSet::const_iterator m=merged.begin(); merged.end()!=m; ++m)
      effect_for(*m, actions, false);
  }
}


// manipulators

void assembly::pimpl::justify(eu::TokenId tok, eu::TokenId just) {
  
  if( !tok->isInactive() ) {
    token_map::left_const_iterator from, to, i;
    boost::tie(from, to) = m_justified.left.equal_range(tok);
  
    if( from!=to ) {
      // it was justifed but was it self justified ?
      for(i=from; i!=to; ++i)
        if( i->second==just )
          return;
    }
    m_justified.insert(token_map::relation(tok, just));
    if( just==tok ) {
      print_token(log(s_justify), *tok);
    } else {
      print_token(print_token(log(s_justify), *tok)<<"\n <- ", *just);
    }
  
    if( tok->isMerged() )
      justify(tok->getActiveToken(), tok);
    else if( tok->isActive() ) {
//      if( is_action(tok) ) {
//        // if an action is justified I can assume that all of its conditions are (??)
      // the answer is either no or I need to change my model
//        eu::TokenSet const &sl = tok->slaves();
//        for(eu::TokenSet::const_iterator i=sl.begin(); sl.end()!=i; ++i) {
//          eu::TokenId s = *i;
//          if( is_condition(s) )
//            justify(s, tok);
//        }
//      }
      // an effect justify its action (??)
      eu::TokenSet actions;
      effect_for(tok, actions);
      for(eu::TokenSet::const_iterator a=actions.begin(); actions.end()!=a; ++a)
        justify(*a, tok);
    }
  }
}

void assembly::pimpl::unjustify(eu::TokenId tok) {
  token_map::right_iterator from, to, i;
  boost::tie(from, to) = m_justified.right.equal_range(tok);
  
  if( from!=to ) {
    eu::TokenSet to_check;
    for(i=from; i!=to; ++i) {
      if( i->second!=tok )
        to_check.insert(i->second);
    }
    print_token(log(s_unjustify)<<"(self) ", *tok);
    m_justified.right.erase(from, to);
    for (eu::TokenSet::const_iterator t=to_check.begin(); to_check.end()!=t; ++t) {
      if( !justified(*t) )
        unjustify(*t);
    }
  }
}


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
        // log()<<"Resuming planning.";
        m_planning = true;
        m_confirmed = false;
        m_steps = m_solver->getStepCount()+m_lost;
        if( m_clock.started() )
          m_plan_since = m_clock.current();
      } else {
        // log()<<"New plan step.";
        m_confirmed = true;
        if( m_max_delay && m_clock.started() ) {
          clock::tick_type delay = m_clock.current()-m_plan_since;
          if( delay>*m_max_delay ) {
            log(tlog::error)<<"Planning exceeded its timeout ("<<delay<<">"
              <<(*m_max_delay)<<")";
            end_plan();
            exit(5);
          }
        }
      }
      
      if( !m_pending ) {
        boost::weak_ptr<pimpl> me(shared_from_this());

        m_pending = true;
        prot::strand().send(boost::bind(&pimpl::async_exec,
                                        me,
                                        &pimpl::do_step),
                            assembly::plan_p);
      } else {
        log(tlog::error)<<"already have a pending step";
      }
      
    } else if( m_planning ) {
      end_plan();
    }
  } else {
    log(tlog::error)<<"No solver yet.";
  }
}

void assembly::pimpl::do_step() {
  // log()<<"Executing step";
  try {
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
          eu::DbClientId cli = m_plan->getClient();
          
          // Before I need to relax my own mess
          for(token_almanach::iterator i=m_forcefully_injected.begin();
              m_forcefully_injected.end()!=i; ++i) {
            if( !i->second->isInactive() )
              cli->cancel(i->second);
          }
          m_forcefully_injected.clear();
          m_cstr->propagate();
          if( m_cstr->provenInconsistent() ) {
            m_solver->backjump(1);
            bsteps = m_solver->getStepCount();
            if( steps>=bsteps )
              m_lost += steps-bsteps;
          }
          send_step();
        } else if( m_solver->getOpenDecisions().empty() &&
                  !m_cstr->pending() ) {
          end_plan();
        } else {
          std::multimap<eu_s::Priority, std::string>
          decisions = m_solver->getOpenDecisions();
          
          std::ostringstream oss;
          
          oss<<"Decisions left: "<<decisions.size();
          if( !decisions.empty() ) {
            std::multimap<eu_s::Priority, std::string>::const_iterator
            d = decisions.begin();
            oss<<"\n   - Next decision: <"<<d->first<<", "<<d->second<<">";
          }
          
          if( m_solver->getDepth()>0 )
            oss<<"\n   - Last decision: "<<m_solver->getLastExecutedDecision();
          log()<<oss.str();
          
          log()<<"Step start";
          m_solver->step();
          log()<<"Step end";
          send_step();
        }
      }
    } else
      log(tlog::warn)<<"do_step while not planning";
  } catch(std::exception const &e) {
    log(tlog::error)<<"exception during step: "<<e.what();
    debugMsg("europtus", e.what());
    exit(1);
  } catch(...) {
    log(tlog::error)<<"unknown exception during step";
    exit(1);
  }
}


void assembly::pimpl::end_plan() {
  size_t steps = m_solver->getStepCount()+m_lost;
  
  if( steps!=m_steps ) {
    log()<<"Planning completed after "
      <<(steps-m_steps)<<" steps:\n"
    <<"  - steps="<<steps<<"\n"
    <<"  - depth="<<m_solver->getDepth()<<"\n"
    <<"==============================================================\n"
    <<m_plan->toString()
    <<"\n=============================================================="
    <<std::endl;
    m_steps = steps;
  }
  m_planning = false;
  m_confirmed = false;
}


eu::TokenId assembly::pimpl::new_token(eu::ObjectId const &obj,
                                       std::string pred,
                                       bool is_fact) {
  if( !have_predicate(obj, pred) )
    throw exception("Object \""+obj->getName().toString()
                    +"\" do not have predicate \""+pred+"\"");
  eu::DbClientId cli = m_plan->getClient();
  eu::TokenId tok = cli->createToken(pred.c_str(), NULL,
                                     !is_fact, // a non fact is rejectable
                                     is_fact);
  
  tok->getObject()->specify(obj->getKey());
  return tok;
}


eu::TokenId assembly::pimpl::new_token(std::string const &object,
                                       std::string const &pred,
                                       bool is_fact) {
  eu::ObjectId obj = m_plan->getObject(object);
  if( obj.isNoId() )
    throw exception("Undefined object \""+object+"\"");
  return new_token(obj, pred, is_fact);
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
assembly::pimpl::restrict_global(char const *name,
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

void assembly::pimpl::check_guarded() {
  eu::eint::basis_type e_next = static_cast<eu::eint::basis_type>(m_clock.current()+1);
  
  for(eu::TokenSet::const_iterator i=m_guarded.begin(); m_guarded.end()!=i; ++i) {
    eu::TokenId tok = *i;
    
    print_token(log()<<"checking ", *tok);
    if( tok->start()->lastDomain().isMember(e_next) ) {
      eu::TokenSet const &sl = tok->slaves();
      eu::TokenSet effects;
      bool guarded = false;
      
      for(eu::TokenSet::const_iterator s=sl.begin(); sl.end()!=s; ++s) {
        if( is_condition(*s) ) {
          if( !justified(*s) ) {
            eu::TokenId guard = *s;
            if( guard->isMerged() )
              guard->getActiveToken();
            
            print_token(log()<<"   - guard: ", *guard);
            guarded = true;
            // break;
          }
        } else if( is_effect(*s) )
          effects.insert(*s);
      }
      if( !guarded ) {
        log()<<"   * token is free to start ("<<effects.size()<<" effects)";
        for(eu::TokenSet::const_iterator e=effects.begin(); effects.end()!=e; ++e)
          print_token(log()<<" - ", **e);
      }
        
    } else
      log()<<"   - cannot start at next tick";
  }
}


void assembly::pimpl::init_plan_state() {
  eu::ObjectId obj = m_plan->getObject("europtus");
  if( obj.isNoId() )
    log(tlog::error)<<"Did not find europtus timeline";
  else if( !eu::TimelineId::convertable(obj) )
    log(tlog::error)<<"europtus is not a timeline";
  else {
    eu::eint lo = std::numeric_limits<eu::eint>::minus_infinity(),
    hi = static_cast<eu::eint::basis_type>(m_clock.current())-1;
  
    eu::IntervalIntDomain past(lo, hi);
    
    m_plan_state = obj;
    m_plan_tok = new_token(m_plan_state, "planning", true);
    m_plan_tok->start()->restrictBaseDomain(past);
//    typedef std::list<eu::TokenId> tok_seq;
//    tok_seq const &toks = m_plan_state->getTokenSequence();
//    eu::DbClientId cli = m_plan->getClient();
//    
//    if( toks.empty() ) {
//      cli->constrain(m_plan_state, m_plan_tok, m_plan_tok);
//      m_cstr->propagate();
//    } else
//      log(tlog::warn)<<"did not activate initial planning";
  }
}

void assembly::pimpl::update_state(clock::tick_type date) {
  // TODO revise this code 
  if( m_plan_state.isId() && date>0 ) {
    if( m_plan_tok.isNoId() ) {
      log(tlog::error)<<"No curent plan state token";
      exit(5);
    }
    
    eu::IntervalIntDomain now(date),
    future(static_cast<eu::eint::basis_type>(date+1),
           std::numeric_limits<eu::eint>::infinity());
    eu::TokenId current = m_plan_tok, tok;
    
    if( m_plan_tok->isMerged() )
      current = m_plan_tok->getActiveToken();
    
    if( m_planning && m_confirmed ) {
      if( m_plan_tok->getUnqualifiedPredicateName()!="planning" ) {
        tok = new_token(m_plan_state, "planning", true);
        tok->start()->restrictBaseDomain(now);
        m_plan_tok->end()->restrictBaseDomain(now);
        m_plan_tok = tok;
      }
      m_plan_tok->end()->restrictBaseDomain(future);
      // No need to propagaet as we are in planning
    } else if( !m_planning ) {
      if( m_plan_tok->getUnqualifiedPredicateName()!="execute" ) {
        
        // Look for my successor:
        // Assumptions:
        //   - I assume that planning has the rule meets(execute)
        //   - I "know" that I have no more flaw as m_planning=false
        //   - hence I should have successor to m_plan_tok which is like me
        typedef std::list<eu::TokenId> tok_seq;
        tok_seq const &toks = m_plan_state->getTokenSequence();
        tok_seq::const_iterator i = toks.begin();
        
        for( ; toks.end()!=i && current!=(*i); ++i);
        if( toks.end()==i ) {
          log(tlog::error)<<"Failed to find my previous \"planning\"";
          exit(5);
        } else {
          ++i;
          if( toks.end()==i ) {
            log(tlog::error)<<"My previous planning has no successor";
            exit(5);
          } else if( (*i)->getUnqualifiedPredicateName()!="execute" ) {
            log(tlog::error)<<"The successor to my planning is not \"execute\"";
            exit(5);
          }
          tok = new_token(m_plan_state, "execute", true);
          tok->start()->restrictBaseDomain(now);
          eu::DbClientId cli = m_plan->getClient();
          cli->merge(tok, *i);
          token_almanach::value_type val(m_solver->getDepth(), tok);
          m_forcefully_injected.insert(val);
          m_plan_tok->end()->restrictBaseDomain(now);
          m_plan_tok = tok;
        }
      }
      m_plan_tok->end()->restrictBaseDomain(future);
      m_cstr->propagate();
      // TODO: need to revise this whole thing so it is managed by assmbly with proper priority
      check_guarded();
    }
  }
}

void assembly::pimpl::init_clock() {
  
  double
    secs = ch::duration_cast< ch::duration<double> >(m_clock.tick_duration()).count();
  restrict_global("TICK_DURATION", eu::FloatDT::NAME().c_str(),
                 eu::IntervalDomain(secs));
  m_cur = restrict_global(assembly::s_now.c_str(), eu::IntDT::NAME().c_str(),
                         eu::IntervalIntDomain());
  m_last = restrict_global("FINAL_TICK", eu::IntDT::NAME().c_str(),
                          eu::IntervalIntDomain());
  if( m_planning )
    m_plan_since = m_clock.current();
  init_plan_state();

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
    update_state(cur);
    send_step();
  }
}

void assembly::pimpl::reset_plan_time_out() {
  m_max_delay.reset();
}

void assembly::pimpl::set_plan_time_out(clock::tick_type value) {
  log()<<"Plan timer limitted to "<<value<<" ticks";
  m_max_delay.reset(value);
}

