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
#include "private/assembly_impl.hh" 

#include <boost/bind.hpp>
#include <boost/bind/apply.hpp>
#include <boost/tokenizer.hpp>

using namespace europtus::planner;
namespace fs=boost::filesystem;
namespace asio=boost::asio;

using europtus::clock;

namespace {

  typedef boost::tokenizer< boost::char_separator<char> > path_split;

}

/*
 * class europa::planner::assembly
 */

// statics

void assembly::send(boost::weak_ptr<pimpl> who,
                    boost::function<void (pimpl *)> fn,
                    europtus::priority_strand::priority_type p) {
  boost::shared_ptr<pimpl> me(who.lock());
  if( me )
    prot::strand().send(boost::bind(&pimpl::async_exec, who, fn), p);
}


void assembly::send(boost::weak_ptr<pimpl> who,
                    boost::function<void (pimpl *)> fn) {
  boost::shared_ptr<pimpl> me(who.lock());
  if( me )
    prot::strand().send(boost::bind(&pimpl::async_exec, who, fn));
}



void assembly::on_clock(boost::weak_ptr<pimpl> who, clock &c,
                        clock::state evt) {
  boost::shared_ptr<pimpl> me(who.lock());

  if( clock::clock_started==evt && me ) {
    clock::tick_sig::slot_type
      tick(&assembly::on_tick, who, _1, _2),
      end(&assembly::on_final, who, _1, _2);
    
    c.on_tick().connect(tick.track(who));
    c.on_final_update().connect(end.track(who));
    
    send(who, &pimpl::init_clock, 0);
  }
}

void assembly::on_tick(boost::weak_ptr<pimpl> who, clock &c,
                       clock::tick_type t) {
  send(who, boost::bind(&pimpl::tick_updated, _1, t), 2);
}

void assembly::on_final(boost::weak_ptr<pimpl> who, clock &c,
                        clock::tick_type f) {
  send(who, boost::bind(&pimpl::final_updated, _1, f), 2);
}


// structors

assembly::assembly(asio::io_service &io, clock &c)
:m_path_fresh(false) {
  prot::init(io);
  // Initialize the implementaion as a blocking call
  //   not ideal but it should work for now
  
  m_impl = prot::strand().post(boost::bind(&pimpl::create, boost::ref(c)), 0).get();
  
  boost::weak_ptr<pimpl> ref(m_impl);
  clock::clock_sig::slot_type ck(&assembly::on_clock, ref, _1, _2);
  c.on_clock(ck.track(ref));
  add_search_path(pimpl::s_europa);
}



assembly::~assembly() {
  // ensure that destruction is done on the strand
  // right now I can only do it by blocking
  prot::strand().post(boost::bind(&pimpl::release, boost::ref(m_impl)), 0).get();
}

// observers

std::string const &assembly::search_path() const {
  path_mutex::scoped_lock lock(m_mtx);
  
  if( !m_path_fresh ) {
    std::ostringstream oss;
    bool first = true;
    for(path_set::const_iterator i=m_path.begin(); m_path.end()!=i; ++i) {
      if( first )
        first = false;
      else
        oss.put(':');
      oss<<i->string();
    }
    m_europa_path = oss.str();
    m_path_fresh = true;
  }
  return m_europa_path;
}


// modifiers

bool assembly::add_search_path(assembly::path p) {
  if( !p.empty() ) {
    p.make_preferred();
    path::iterator i = p.begin();
    
    if( "~"==i->string() ) {
      // handle case where ~ was not extended
      fs::path home(std::getenv("HOME"));
      
      for(++i; p.end()!=i; ++i)
        home /= *i;
      p = home;
      p.make_preferred();
    }
    
    // Now I can do the proper insertion
    {
      path_mutex::scoped_lock lock(m_mtx);
      
      if( m_path.insert(p).second ) {
        m_path_fresh = false;
        return true;
      }
    }
  }
  return false;
}

size_t assembly::add_search_path(std::string const &str) {
  size_t added=0;
  boost::char_separator<char> sep(":;");
  path_split tok(str, sep);
  
  for(path_split::iterator i=tok.begin(); tok.end()!=i; ++i) {
    path p(*i);
    if( add_search_path(p) )
      ++added;
  }
  return added;
}





//#include "europtus/planner/assembly.hh"
//#include "europtus/planner/ModuleEuroptus.hh"
//
//// headers for all the europa modules we use (or intend to)
//#include <PLASMA/ModuleConstraintEngine.hh>
//#include <PLASMA/ModulePlanDatabase.hh>
//#include <PLASMA/ModuleRulesEngine.hh>
//#include <PLASMA/ModuleTemporalNetwork.hh>
//#include <PLASMA/ModuleSolvers.hh>
//#include <PLASMA/ModuleNddl.hh>
//#include <PLASMA/NddlInterpreter.hh>
//
//#include <PLASMA/TemporalAdvisor.hh>
//#include <PLASMA/Propagator.hh>
//
//
//#include <boost/scoped_ptr.hpp>
//#include <boost/tokenizer.hpp>
//#include <boost/filesystem/operations.hpp>
//
//# include <cstdlib>
//
//#include "europtus/planner/exception.hh"
//
//using namespace europtus::planner;
//namespace eu=EUROPA;
//namespace eu_s=eu::SOLVERS;
//namespace fs=boost::filesystem;
//
//using namespace boost::chrono;
//
//
///*
// * class europtus::planner::assembly
// */
//
//// statics
//
//fs::path const assembly::s_europa(EUROPA_HOME"/include");
//
//
//// structors
//
//assembly::assembly(clock &c):m_clock(c), m_path_fresh(false) {
//  
//  // inject diverse modules needed
//  addModule((new eu::ModuleConstraintEngine())->getId());
//  addModule((new eu::ModuleConstraintLibrary())->getId());
//  addModule((new eu::ModulePlanDatabase())->getId());
//  addModule((new eu::ModuleRulesEngine())->getId());
//  addModule((new eu::ModuleTemporalNetwork())->getId());
//  addModule((new eu::ModuleSolvers())->getId());
//  addModule((new eu::ModuleNddl())->getId());
//  addModule((new ModuleEuroptus(*this))->getId());
//  
//  // complete europa engine initialisation
//  doStart();
//  
//  
//  
//  // gather usefull attributes for plan manipuation
//  m_schema = ((eu::Schema *)getComponent("Schema"))->getId();
//  m_plan = ((eu::PlanDatabase *)getComponent("PlanDatabase"))->getId();
//  m_cstr = ((eu::ConstraintEngine *)getComponent("ConstraintEngine"))->getId();
//  
//  // some extra init on europa side
//  cstr_engine()->setAutoPropagation(false);
//  eu::DomainComparator::setComparator((eu::Schema *)m_schema);
//  
//  // Add the directory where europa core nddl files are
//  add_search_path(s_europa);
//}
//
//assembly::~assembly() {
//  // notify perent class on destruction
//  doShutdown();
//}
//
//// observers
//
//std::string const &assembly::nddl_path() const {
//  if( !m_path_fresh ) {
//    std::ostringstream oss;
//    bool first = true;
//    
//    for(path_set::const_iterator i=m_path.begin();
//        m_path.end()!=i; ++i) {
//      if( first )
//        first = false;
//      else
//        oss<<':';
//      oss<<i->string();
//    }
//    
//    m_europa_path = oss.str();
//    m_path_fresh = true;
//  }
//  return m_europa_path;
//}
//
//bool assembly::locate(fs::path &p) const {
//  if( fs::is_regular_file(p) )
//    return true;
//  else {
//    for(path_set::const_iterator i=m_path.begin();
//        m_path.end()!=i; ++i) {
//      fs::path tmp(*i);
//      tmp /= p;
//      if( fs::is_regular_file(tmp) ) {
//        p = tmp;
//        return true;
//      }
//    }
//  }
//  return false;
//}
//
//
//
//// modifiers
//
//bool assembly::add_search_path(fs::path p) {
//  
//  if( !p.empty() ) {
//    p.make_preferred();
//    fs::path::iterator i = p.begin();
//
//    if( i->string()=="~" ) {
//      fs::path home(std::getenv("HOME"));
//      
//      for(++i; p.end()!=i; ++i)
//        home /= *i;
//      
//      p = home;
//      p.make_preferred();
//    }
//    
//    if( m_path.insert(p).second ) {
//      m_path_fresh = false;
//      return true;
//    }
//  }
//  return false;
//}
//
//namespace {
//  typedef boost::tokenizer< boost::char_separator<char> > my_tokenizer;
//}
//
//size_t assembly::add_search_path(std::string str) {
//  size_t added = 0;
//  boost::char_separator<char> sep(":;");
//  my_tokenizer tok(str, sep);
//  
//  for(my_tokenizer::iterator i=tok.begin(); tok.end()!=i; ++i) {
//    fs::path p(*i);
//    if( add_search_path(p) )
//      ++added;
//  }
//  
//  return added;
//}
//
//
//
//void assembly::configure(std::string const &solver_cfg) {
//  boost::scoped_ptr<eu::TiXmlElement> xml_cfg(eu::initXml(solver_cfg.c_str()));
//
//  m_planner = (new eu_s::Solver(plan_db(), *xml_cfg))->getId();
//  
//}
//
//bool assembly::load_model(fs::path nddl_file) {
//  std::string const &path = nddl_path();
//  std::string ret;
//  
//  nddl_file.make_preferred();
//  
//  getLanguageInterpreter("nddl")->getEngine()->getConfig()->setProperty("nddl.includePath", path);
//  try {
//    if( !locate(nddl_file) )
//      throw exception("Failed to locate file \""+nddl_file.string()+"\"");
//    
//    std::cout<<"Loading "<<nddl_file<<std::endl;
//    ret = executeScript("nddl", nddl_file.string(), true);
//    
//    if( !ret.empty() ) {
//      throw exception("Errors while parsing \""+nddl_file.string()+"\":\n"+ret);
//    }
//    
//  } catch(eu::PSLanguageExceptionList const &le) {
//    std::ostringstream err;
//    err<<"Errors while parsing \""<<nddl_file<<"\":\n"<<le;
//    throw exception(err.str());
//  } catch(Error const &e) {
//    std::ostringstream err;
//    err<<"Error while parsing \""<<nddl_file<<"\":\n"<<e;
//    throw exception(err.str());
//  }
//  return cstr_engine()->constraintConsistent();
//}
//
//void assembly::init_clock_model() {
//  eu::ConstrainedVariableId
//   tck_dur = plan_db()->getGlobalVariable("TICK_DURATION");
//  m_now = plan_db()->getGlobalVariable("CUR_DATE");
//  m_clock.restrict_final(eu::cast_basis(std::numeric_limits<eu::eint>::infinity())-1);
//  if( tck_dur.isId() ) {
//    double secs = duration_cast< duration<double> >(m_clock.tick_duration()).count();
//    
////    std::cout<<"BEFORE: "<<tck_dur->getName().toString()<<"="
////      <<tck_dur->toString()<<std::endl;
//    tck_dur->restrictBaseDomain(eu::IntervalDomain(secs));
////    std::cout<<"AFTER: "<<tck_dur->getName().toString()<<"="
////      <<tck_dur->toString()<<std::endl;
//  } else
//    std::cerr<<"Plan database has no variable named TICK_DURATION"<<std::endl;
//  if( m_now.isId() ) {
//    m_clock.on_tick().connect(boost::bind(&assembly::update_tick,
//                                         this, _1, _2));
//  } else
//    std::cerr<<"Plan database has no variable named CUR_DATE"<<std::endl;
//}
//
//
//void assembly::update_tick(europtus::clock &c, europtus::clock::tick_type t) {
//  europtus::clock::tick_type last = c.final();
//  // Later I will probably need to check also that we are not planning
//  if( c.completed() )
//    t = last;
//  
//  if( m_now.isId() && (&m_clock)==(&c) ) {
//    eu::IntervalIntDomain future(eu::eint(static_cast<eu::eint::basis_type>(t)),
//                                 eu::eint(static_cast<eu::eint::basis_type>(last)));
//    
////    std::cout<<"BEFORE: "<<m_now->getName().toString()<<"="
////      <<m_now->toString()<<std::endl;
//    m_now->restrictBaseDomain(future);
////    std::cout<<"AFTER: "<<m_now->getName().toString()<<"="
////      <<m_now->toString()<<std::endl;
//    if( !cstr_engine()->propagate() )
//      std::cerr<<"Propagation failure on new tick date\n";
////    std::cout<<plan_db()->toString()<<std::endl;
//  }
//}
//
//
