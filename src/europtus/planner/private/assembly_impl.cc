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
namespace fs=boost::filesystem;


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
  
  
  tu::Symbol s_justify("just");
  tu::Symbol s_unjustify("unjust");
  
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


std::ostream &assembly::pimpl::print_token(std::ostream &out, eu::Token const &tok) {
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


// structor

assembly::pimpl::pimpl(clock &c, tlog::text_log &log)
:m_lost(0), m_clock(c),m_planning(false),m_pending(false),m_log(log) {
  std::string base_name;
  {
    std::ostringstream oss;
    boost::posix_time::ptime cur = boost::posix_time::second_clock::universal_time();
    oss<<cur.date()<<'.';
    base_name = oss.str();
  }
  
  if( fs::exists(base_name+"xml") ) {
    size_t i=1;
    for( ; i<=4096; ++i) {
      std::ostringstream oss;
      oss<<base_name<<i<<".xml";
      if( !fs::exists(oss.str()) ) {
        fs::rename(base_name+"xml", oss.str());
        break;
      }
    }
    if( i>4096 )
      log(tlog::warn)<<"Failed to move previous "<<base_name<<"xml";
  }
  base_name += "xml";
  m_xml.open(base_name.c_str());
  m_xml<<"<Log>"<<std::endl;
}

void assembly::pimpl::initialize() {
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
  
  m_disp.reset(new dispatch_manager(shared_from_this(), m_plan));
}



assembly::pimpl::~pimpl() {
  // make sure that my module is removed before this class is destroyed
  m_disp.reset();
  removeModule(m_europtus);
  doShutdown();
  
  if( m_tick_opened )
    m_xml<<"  </tick>\n";
  m_xml<<"</Log>"<<std::endl;
  m_xml.close();
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
  return m_disp && m_disp->is_fact(tok, or_merged_to);
}


bool assembly::pimpl::is_action(eu::TokenId const &tok) const {
  return m_disp && m_disp->is_action(tok);
}

bool assembly::pimpl::is_predicate(eu::TokenId const &tok) const {
  return tok.isId() && tok->hasAttributes(eu::PSTokenType::PREDICATE);
}

bool assembly::pimpl::is_condition(eu::TokenId  const &tok) const {
  return m_disp && m_disp->is_condition(tok);
}

bool assembly::pimpl::is_effect(eu::TokenId const &tok) const {
  return m_disp && m_disp->is_effect(tok);
}

void assembly::pimpl::effect_for(eu::TokenId const &tok,
                                 eu::TokenSet &actions) const {
  if( m_disp )
    m_disp->effect_for(tok, actions);
}


// manipulators

bool assembly::pimpl::justified(EUROPA::TokenId const &tok) const {
  return m_disp && m_disp->justified(tok);
}

void assembly::pimpl::justify(EUROPA::TokenId const &tok) {
  if( m_disp )
    m_disp->justify(tok);
}

void assembly::pimpl::unjustify(EUROPA::TokenId const &tok) {
  if( m_disp )
    m_disp->unjustify(tok);
}


void assembly::pimpl::schedulled(EUROPA::TokenId tok) {
  m_disp->schedulled(tok);
}

void assembly::pimpl::unschedulled(EUROPA::TokenId tok) {
  m_disp->unschedulled(tok);
}

assembly::request_sig &assembly::pimpl::on_dispatch() {
  return m_request;
}

void assembly::pimpl::dispatch(EUROPA::TokenId const &tok, bool direct) {
  if( tok.isId() ) {
    // convert the europa token into TREX format
    eu::ObjectDomain const &dom = tok->getObject()->lastDomain();
    eu::ObjectId obj = dom.makeObjectList().front();
    tu::Symbol name(obj->getName().toString()),
    pred(tok->getUnqualifiedPredicateName().c_str());
    tr::goal_id req = MAKE_SHARED<tr::Goal>(name, pred);
    
    std::vector<EUROPA::ConstrainedVariableId> const &attrs = tok->parameters();
    // Get start, duration and end
    UNIQ_PTR<tr::DomainBase>
    d_start(te::details::trex_domain(tok->start()->lastDomain())),
    d_duration(te::details::trex_domain(tok->duration()->lastDomain())),
    d_end(te::details::trex_domain(tok->end()->lastDomain()));
    
    req->restrictTime(*dynamic_cast<tr::IntegerDomain *>(d_start.get()),
                          *dynamic_cast<tr::IntegerDomain *>(d_duration.get()),
                          *dynamic_cast<tr::IntegerDomain *>(d_end.get()));
    
    for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator a=attrs.begin();
        attrs.end()!=a; ++a) {
      // Exclude "implicit_var_*"
      if( 0!=(*a)->getName().toString().compare(0, implicit_s.length(), implicit_s) ) {
        UNIQ_PTR<tr::DomainBase> dom(te::details::trex_domain((*a)->lastDomain()));
        tr::Variable attr((*a)->getName().toString(), *dom);
        req->restrictAttribute(attr);
      }
    }
    
    log("REQUEST")<<'['<<req<<"] "<<*req;
    m_request(req);

    // TODO: send the message

    if( direct ) {
      eu::eint::basis_type e_cur = static_cast<eu::eint::basis_type>(m_clock.current());

      eu::ObjectDomain const &dom = tok->getObject()->lastDomain();
      eu::ObjectId obj = dom.makeObjectList().front();
    
      eu::TokenId f = new_token(obj, tok->getPredicateName().c_str(), true);
      eu::IntervalIntDomain t_start = tok->start()->lastDomain();
      
      if( t_start.isMember(e_cur+1) ) {
        t_start.intersect(e_cur+1, e_cur+1);
      } else if( t_start.isMember(e_cur) ) {
        t_start.intersect(e_cur, e_cur);
      } else if( t_start.getLowerBound()>e_cur+1 ) {
        t_start.intersect(t_start.getLowerBound(), t_start.getLowerBound());
      }
      f->start()->restrictBaseDomain(t_start);
      
      std::vector<eu::ConstrainedVariableId> const &attrs = tok->parameters();
      for(std::vector<eu::ConstrainedVariableId>::const_iterator v=attrs.begin();
          attrs.end()!=v; ++v) {
        f->getVariable((*v)->getName())->restrictBaseDomain((*v)->lastDomain());
        if( (*v)->lastDomain().isSingleton() )
          f->getVariable((*v)->getName())->specify((*v)->lastDomain().getSingletonValue());
      }
      eu::DbClientId cli = m_plan->getClient();
      cli->merge(f, tok);
      m_forcefully_injected.insert(std::make_pair(m_solver->getDepth(), f));
    }
  }
}


void assembly::pimpl::add_dispatchable(EUROPA::TokenId tok, bool direct) {
  m_disp->make_dispatchable(tok, direct);
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
          size_t unfilt_size = decisions.size();
          
          if( unfilt_size>1 ) {
            std::set<std::string> cache;
            std::multimap<eu_s::Priority, std::string>::iterator i = decisions.begin();
          
            while( decisions.end()!=i ) {
              if( cache.insert(i->second).second ) {
                ++i;
              } else {
                // this flaw was already there
                decisions.erase(i++);
              }
            }
          }
          
          
          std::ostringstream oss;
          std::multimap<eu_s::Priority, std::string>::const_iterator
          d = decisions.begin();
          
          
          oss<<"Decisions left: "<<decisions.size();
          if( decisions.size()<unfilt_size )
            oss<<"("<<unfilt_size<<")";
          oss<<"\n   - (steps="<<(m_solver->getStepCount()+m_lost)
            <<", depth="<<m_solver->getDepth()<<")";
          
          bool verbose = false;
          
          if( !verbose ) {
            if( !decisions.empty() ) {
              oss<<"\n   - Next decision: <"<<d->first<<", "<<d->second<<">";
            }
          
            if( m_solver->getDepth()>0 )
              oss<<"\n   - Last decision: "<<m_solver->getLastExecutedDecision();
          } else {
            oss<<"\n   - flaw stack {";
            for( ; decisions.end()!=d; ++d)
              oss<<"\n      "<<d->first<<": "<<d->second;
            if( !decisions.empty() )
              oss<<"\n    ";
            oss.put('}');
          
            eu_s::DecisionStack const &stack = m_solver->getDecisionStack();
            size_t count = stack.size();
            if( !stack.empty() ) {
              oss<<"\n   - decisions stack (last 10) {";
          
              size_t c = 0;
              for(eu_s::DecisionStack::const_reverse_iterator i=stack.rbegin();
                  stack.rend()!=i && c<10; ++i, --count, ++c) {
                eu_s::DecisionPointId x = (*i);
                oss<<"\n      "<<count<<": "<<x->toShortString();
              }
              oss<<"\n    }";
            }
          }
          log()<<oss.str();
          
          m_solver->step();
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
    <<"  - (steps="<<steps<<", depth="<<m_solver->getDepth()<<")\n"
    <<"==============================================================\n"
    <<m_plan->toString()
    <<"\n=============================================================="
    <<std::endl;
    m_steps = steps;
  }
  m_planning = false;
  m_confirmed = false;
  send_exec();
}


eu::TokenId assembly::pimpl::new_token(eu::ObjectId const &obj,
                                       std::string pred,
                                       bool is_fact) {
  static size_t count = 0;
  
  if( !have_predicate(obj, pred) )
    throw exception("Object \""+obj->getName().toString()
                    +"\" do not have predicate \""+pred+"\"");
  eu::DbClientId cli = m_plan->getClient();
  
  std::ostringstream name;
  
  if( is_fact )
    name.put('f');
  else
    name.put('r');
  
  size_t last_dot = pred.find_last_of('.');
  
  if( last_dot==std::string::npos )
    last_dot = 0;
  else
    last_dot += 1;
  
  name<<'_'<<obj->getName().toString()<<'.'<<pred.substr(last_dot)<<'_'<<(++count);
  
  
  eu::TokenId tok = cli->createToken(pred.c_str(), name.str().c_str(),
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
    clock::tick_type cur = m_clock.current();
    
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
    bool new_tick = false;
    if( m_tick_opened && cur!=*m_tick_opened ) {
      m_xml<<"  </tick>\n";
      new_tick = true;
    }
    
    if( new_tick || !m_tick_opened ) {
      m_tick_opened = cur;
      m_xml<<"  <tick value=\""<<cur<<"\">\n";
    }
    m_xml<<"    ";
    boost::property_tree::ptree x = g->as_tree(true);
    boost::property_tree::ptree::value_type out("Observation", x.front().second);
    x.clear();
    x.push_front(out);
    tu::write_xml(m_xml, x, false);
    m_xml<<std::endl;
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

void assembly::pimpl::send_exec() {
  boost::weak_ptr<pimpl> me(shared_from_this());
  details::europa_protect::strand().send(boost::bind(&pimpl::async_exec,
                                                     me,
                                                     &pimpl::check_guarded),
                                         assembly::exec_p);
}


void assembly::pimpl::check_guarded() {
  if( m_disp ) {
    eu::eint::basis_type e_cur = static_cast<eu::eint::basis_type>(m_clock.current());
    eu::TokenSet postponed;
    eu::IntervalIntDomain future(e_cur+1, std::numeric_limits<eu::eint>::infinity());
  
    if( m_disp->postponable(e_cur, postponed)>0 ) {
      // Make all the postponable token to start after e_cur
      for(eu::TokenSet::const_iterator p=postponed.begin(); postponed.end()!=p; ++p) {
        (*p)->start()->restrictBaseDomain(future);
        if( !m_cstr->propagate() ) {
          log(tlog::warn)<<"Failed to set start of "<<(*p)->getName().toString()<<'('
            <<(*p)->getKey()<<") to be after "<<e_cur;
          send_step();
          return;
        }
      }
    }
    
    m_disp->do_dispatch(e_cur);
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
      send_exec();
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

