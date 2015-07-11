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
#include "europtus/log_player.hh"

#include <boost/property_tree/xml_parser.hpp>

using namespace europtus;
namespace bpt=boost::property_tree;
namespace xml=bpt::xml_parser;
namespace tr=TREX::transaction;
namespace tu=TREX::utils;



/*
 * class europtus::log_player
 */

// structors

log_player::log_player() {}

log_player::~log_player() {
  m_logs.clear();
}

// modifiers

void log_player::load(std::string const &file) {
  bpt::ptree log;
  read_xml(file, log, xml::no_comments|xml::trim_whitespace);
  if( log.empty() )
    throw std::runtime_error(file+" is en empty xml");
  if( log.size()!=1 )
    throw std::runtime_error(file+" has multiple xml roots");
  load(log.front());
}

void log_player::load(bpt::ptree::value_type const &xml) {
  bpt::ptree ticks = xml.second;
  bpt::ptree::assoc_iterator i, last;
  
  boost::tie(i, last) = ticks.equal_range("tick");
  
  for(; last!=i; ++i) {
    tick_type cur = tu::parse_attr<tick_type>(*i, "value");
    for(bpt::ptree::iterator j=i->second.begin(); i->second.end()!=j; ++j) {
      if( j->first=="Observation" ) {
        tr::goal_id obs = MAKE_SHARED<tr::Goal>(boost::ref(*j));
        m_logs.insert(transaction_map::value_type(cur, obs));
      }
    }
  }
}


void log_player::tick(clock &clk, clock::tick_type cur) {
  transaction_map::iterator i, last;
  boost::tie(i, last) = m_logs.equal_range(cur);
  for(; last!=i; ++i)
    m_obs(i->second);
}



