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
#include "europtus/planner/extensions/dispatchable.hh"
#include <PLASMA/RuleInstance.hh>

#include "../private/assembly_impl.hh"

using namespace europtus::planner;
namespace eu=EUROPA;

namespace {

  eu::TokenId getParentToken(eu::ConstrainedVariableId const &var) {
    if(var->parent().isId()){
      if(eu::TokenId::convertable(var->parent()))
        return var->parent();
      
      if(eu::RuleInstanceId::convertable(var->parent())){
        eu::RuleInstanceId r = var->parent();
        return r->getToken();
      }
    }
    
    return eu::TokenId::noId();
  }
  
}

dispatchable::dispatchable(eu::LabelStr const &name,
                           eu::LabelStr const &propagatorName,
                           eu::ConstraintEngineId const &cstr,
                           std::vector<eu::ConstrainedVariableId> const &vars)
:eu::Constraint(name, propagatorName, cstr, vars),
m_token(getParentToken(vars[0])),
m_var(getCurrentDomain(vars[0])),
m_pending(true) {
}

bool dispatchable::connected() const {
  if( m_prop.isNoId() ) {
    eu::PropagatorId prop = getPropagator();
    
    if( !propagator::id::convertable(prop) )
      return false;
    m_prop = prop;
  }
  return true;
}


assembly::pimpl &dispatchable::self() const {
  if( !connected() )
    throw exception("dispatchable() not connected to europtus");
  return m_prop->self();
}


dispatchable::~dispatchable() {
  handleDiscard();
}

void dispatchable::dispatch(bool direct) {
  m_pending = false;
  if( connected() ) {
    // TODO: do something to mark the token in assembly
    self().log(getName().c_str())<<"dispatch("<<m_token->getPredicateName().toString()<<", "<<direct<<")";
    self().add_dispatchable(m_token, direct);
  } else
    debugMsg("europtus", "dispatchable() is not connected to europtus")
}

void dispatchable::handleExecute() {
  if( m_token.isId() && m_token->isActive() && m_pending ) {
    eu::BoolDomain true_dom(true);
  
    typedef std::vector<eu::ConstrainedVariableId> scope;
    
    scope const &vars = getScope();
    for(scope::const_iterator v = vars.begin(); vars.end()!=v; ++v) {
      eu::Domain const &d = getCurrentDomain(*v);
      
      if( !d.isSingleton() ) {
        if( connected() )
          self().log(getName().c_str())<<m_token->getPredicateName().toString()<<"("
          <<m_token->getKey()<<") still guarded by "<<(*v)->getName().toString();
        return;
      }
    }
    // at this point I know that all domains are singleton (including m_var)
    bool direct = (m_var.getSingletonValue()==true_dom.getSingletonValue());
    
    dispatch(direct);
  }
}


void dispatchable::handleDiscard() {
  // TODO: do somethign to clean assembly
  if( !m_pending && connected() ) {
    self().log(getName().c_str())<<"undispatch("<<m_token->getPredicateName().toString()<<")";
    self().remove_dispatchable(m_token);
    m_pending = true;
  }
}
