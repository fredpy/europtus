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
#include "europtus/planner/extensions/decision_point.hh"
#include "europtus/planner/assembly.hh"

#include <PLASMA/DbClient.hh>
#include <PLASMA/Token.hh>
#include <PLASMA/TokenVariable.hh>

using namespace europtus::planner;
namespace eu=EUROPA;
namespace eu_s=EUROPA::SOLVERS;

threat_decision_pt::threat_decision_pt(eu::DbClientId const &client,
                                       eu::TokenId const &tokenToOrder,
                                       eu::TiXmlElement const &configData,
                                       eu::LabelStr const &explanation)
:eu_s::ThreatDecisionPoint(client, tokenToOrder, configData, explanation) {
  if( !client->isGlobalVariable(assembly::s_now) ) {
    eu::IntervalIntDomain dom;
    m_clock = client->createVariable(eu::IntDT::NAME().c_str(), dom,
                                     assembly::s_now.c_str(), false);
  } else {
    m_clock = client->getGlobalVariable(assembly::s_now);
  }
}

threat_decision_pt::~threat_decision_pt() {
  
}

eu::eint threat_decision_pt::now() const {
  return m_clock->baseDomain().getLowerBound();
}

std::string threat_decision_pt::toString() const {
  if( m_choices.empty() )
    return "THREAT:EMPTY["+m_tokenToOrder->toString()+"]";
  if( m_index>=m_choiceCount )
    return "THREAT:EXHAUSTED["+m_tokenToOrder->toString()+"]";
  return eu_s::ThreatDecisionPoint::toString();
}

std::string threat_decision_pt::toShortString() const {
  if( m_choices.empty() )
    return "THR:EMPTY["+m_tokenToOrder->toString()+"]";
  if( m_index>=m_choiceCount )
    return "THR:EXHAUSTED["+m_tokenToOrder->toString()+"]";
  return eu_s::ThreatDecisionPoint::toShortString();
}


void threat_decision_pt::handleInitialize() {
  eu_s::ThreatDecisionPoint::handleInitialize();
  typedef std::pair<eu::TokenId, eu::TokenId> tok_pair;
  typedef std::pair<eu::ObjectId, tok_pair> obj_assign;
  typedef std::vector<obj_assign> choice;
  
  choice past;
  
  if( m_tokenToOrder->start()->lastDomain().getUpperBound()>=now() ) {
    eu::eint min_dur = m_tokenToOrder->duration()->lastDomain().getLowerBound();
    
    // prune all the choices that would put this token in the past
    choice::iterator i = m_choices.begin();
    while( m_choices.end()!=i ) {
      if( m_tokenToOrder==i->second.first ) {
        if( m_tokenToOrder!=i->second.second ) {
          // m_tokenToOrder is inserted before i->second.second
          eu::eint max_start = i->second.second->start()->lastDomain().getUpperBound();
          if( max_start < std::numeric_limits<eu::eint>::infinity() ) {
            max_start = max_start-min_dur;
            if( max_start<now() ) {
              past.push_back(*i);
              i = m_choices.erase(i);
              continue;
            }
          }
        }
      }
      ++i;
    }
  }
  std::reverse(m_choices.begin(), m_choices.end());
  m_choices.insert(m_choices.end(), past.begin(), past.end());
  m_choiceCount = m_choices.size();
}
