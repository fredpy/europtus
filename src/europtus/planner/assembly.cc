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
#include <boost/filesystem/operations.hpp>
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
                    europa_priority p) {
  boost::shared_ptr<pimpl> me(who.lock());
  if( me )
    prot::strand().send(boost::bind(&pimpl::async_exec, who, fn),
                        static_cast<europtus::priority_strand::priority_type>(p));
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
    
    send(who, &pimpl::init_clock, init_p);
  }
}

void assembly::on_tick(boost::weak_ptr<pimpl> who, clock &c,
                       clock::tick_type t) {
  send(who, boost::bind(&pimpl::tick_updated, _1, t), tick_p);
}

void assembly::on_final(boost::weak_ptr<pimpl> who, clock &c,
                        clock::tick_type f) {
  send(who, boost::bind(&pimpl::final_updated, _1, f), tick_p);
}


// structors

assembly::assembly(asio::io_service &io, clock &c)
:m_path_fresh(false) {
  prot::init(io);
  // Initialize the implementaion as a blocking call
  //   not ideal but it should work for now
  
  m_impl = prot::strand().post(boost::bind(&pimpl::create, boost::ref(c)),
                               init_p).get();
  
  boost::weak_ptr<pimpl> ref(m_impl);
  clock::clock_sig::slot_type ck(&assembly::on_clock, ref, _1, _2);
  c.on_clock(ck.track(ref));
  add_search_path(pimpl::s_europa);
}



assembly::~assembly() {
  // ensure that destruction is done on the strand
  // right now I can only do it by blocking
  prot::strand().post(boost::bind(&pimpl::release, boost::ref(m_impl)),
                      init_p).get();
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

bool assembly::locate(path &p) const {
  if( fs::is_regular_file(p) )
    return true;
  else {
    path_mutex::scoped_lock lock(m_mtx);
    
    for(path_set::const_iterator i=m_path.begin();
        m_path.end()!=i; ++i) {
      path tmp(*i);
      
      tmp /= p;
      if( fs::is_regular_file(tmp) ) {
        p = tmp;
        return true;
      }
    }
    return false;
  }
}

// modifiers

void assembly::load_solver(path cfg_file) {
  cfg_file.make_preferred();
  if( !locate(cfg_file) )
    throw exception("Failed to locate file \""+cfg_file.string()+"\".");
  prot::strand().post(boost::bind(&pimpl::cfg_solver, m_impl,
                                  cfg_file.string()),
                      init_p).get();
}

bool assembly::load_nddl(path nddl_file) {
  nddl_file.make_preferred();
  if( !locate(nddl_file) )
    throw exception("Failed to locate file \""+nddl_file.string()+"\".");

  return prot::strand().post(boost::bind(&pimpl::nddl, m_impl,
                                          search_path(), nddl_file.string()),
                             init_p).get();
}

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
