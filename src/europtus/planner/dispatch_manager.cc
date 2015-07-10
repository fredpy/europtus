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
#include "private/assembly_impl.hh"

#include <PLASMA/TokenVariable.hh>

namespace eu=EUROPA;
namespace tu=TREX::utils;
namespace tlog=tu::log;

namespace europtus {
  namespace planner {
    
    class dispatch_manager::token_proxy :public eu::PlanDatabaseListener {
    public:
      token_proxy(dispatch_manager &me, eu::PlanDatabaseId const &plan);
      ~token_proxy() {}
      
    private:
      void notifyAdded(eu::TokenId const &token);
      void notifyRemoved(eu::TokenId const &token);
      void notifyActivated(eu::TokenId const &token);
      void notifyDeactivated(eu::TokenId const &token);
      void notifyMerged(eu::TokenId const &token);
      void notifySplit(eu::TokenId const &token);
      void notifyRejected(eu::TokenId const &token);
      void notifyReinstated(eu::TokenId const &token);
      void notifyCommitted(eu::TokenId const &token);
      void notifyTerminated(eu::TokenId const &token);
      
      dispatch_manager &m_self;
    };
    
    
  }
}



using namespace europtus::planner;

namespace {
  
  tu::Symbol s_just("JUST");
  tu::Symbol s_disp("DISPATCH");
  
}

/*
 * class europtus::planner::dispatch_manager::token_proxy
 */

// structors

dispatch_manager::token_proxy::token_proxy(dispatch_manager &me, eu::PlanDatabaseId const &plan)
:eu::PlanDatabaseListener(plan), m_self(me) {}

// callbacks

void dispatch_manager::token_proxy::notifyAdded(eu::TokenId const &token) {
}

void dispatch_manager::token_proxy::notifyRemoved(eu::TokenId const &token) {
}

void dispatch_manager::token_proxy::notifyActivated(eu::TokenId const &token) {
  if( token->isFact() )
    m_self.justify(token);
  else {
    if( m_self.is_condition(token) ) {
      eu::TokenId m = token->master();
      
      if( m_self.is_action(m) && m_self.justified(m) ) {
        m_self.justify(token, m);
      }
    } else if( m_self.is_action(token) ) {
      m_self.add_guarded(token);
    }
  }
}

void dispatch_manager::token_proxy::notifyDeactivated(eu::TokenId const &token) {
  m_self.unjustify(token);
  if( m_self.is_action(token) ) {
    m_self.remove_guarded(token);
  }
  m_self.unschedulled(token);
}

void dispatch_manager::token_proxy::notifyMerged(eu::TokenId const &token) {
  if( token->isFact() )
    m_self.justify(token);
  else {
    if( m_self.is_condition(token) ) {
      eu::TokenId m = token->master();
      
      if( m_self.is_action(m) && m_self.justified(m) ) {
        m_self.justify(token, m);
      }
    }
  }
}

void dispatch_manager::token_proxy::notifySplit(eu::TokenId const &token) {
  m_self.unjustify(token);
}

void dispatch_manager::token_proxy::notifyRejected(eu::TokenId const &token) {
}

void dispatch_manager::token_proxy::notifyReinstated(eu::TokenId const &token) {
}

void dispatch_manager::token_proxy::notifyCommitted(eu::TokenId const &token) {
}

void dispatch_manager::token_proxy::notifyTerminated(eu::TokenId const &token) {
}

/*
 * class europtus::planner::dispatch_manager
 */

// structors

dispatch_manager::dispatch_manager(boost::shared_ptr<assembly::pimpl> const &ref,
                                   eu::PlanDatabaseId const &plan)
:m_owner(ref), m_proxy(new token_proxy(*this, plan)) {}

dispatch_manager::~dispatch_manager() {}

// observers

bool dispatch_manager::is_fact(EUROPA::TokenId  const &tok,
                               bool or_merged) const {
  if( tok.isId() ) {
    bool fact = tok->isFact();
    
    if( !fact && or_merged ) {
      if( tok->isActive() ) {
        eu::TokenSet const &merged = tok->getMergedTokens();
        
        for(eu::TokenSet::const_iterator m=merged.begin(); merged.end()!=m; ++m) {
          if( (*m)->isFact() )
            return true;
        }
      } else if ( tok->isMerged() )
        return is_fact(tok->getActiveToken(), or_merged);
    }
    return fact;
  }
  return false;
}


bool dispatch_manager::is_action(eu::TokenId const &tok) const {
  return tok.isId() && tok->hasAttributes(eu::PSTokenType::ACTION);
}

bool dispatch_manager::is_condition(eu::TokenId const &tok) const {
  return tok.isId() && tok->hasAttributes(eu::PSTokenType::CONDITION);
}

bool dispatch_manager::is_effect(eu::TokenId const &tok) const {
  return tok.isId() && tok->hasAttributes(eu::PSTokenType::EFFECT);
}

void dispatch_manager::effect_for(eu::TokenId const &tok, eu::TokenSet &actions,
                                  bool recurse) const {
  if( tok.isId() ) {
    if( is_effect(tok) )
      actions.insert(tok->master());
    if( recurse && tok->isMerged() )
      effect_for(tok->getActiveToken(), actions, false);
    else if( tok->isActive() ) {
      eu::TokenSet const &merged = tok->getMergedTokens();
      
      for(eu::TokenSet::const_iterator m=merged.begin(); merged.end()!=m; ++m)
        effect_for(*m, actions, false);
    }
  }
}



bool dispatch_manager::justified(eu::TokenId const &tok, bool or_fact) const {
  if( tok.isId() ) {
    if( tok->isFact() && or_fact )
      return true;
    else {
      eu::TokenId me = tok;
   
      if( tok->isMerged() )
        me = tok->getActiveToken();
      
      return 0 < m_justified.left.count(me);
    }
  }
  return false;
}


// modifiers

void dispatch_manager::justify(eu::TokenId const &tok, eu::TokenId const &just) {
  boost::shared_ptr<assembly::pimpl> me = m_owner.lock();

  if( me && tok.isId() && just.isId() ) {
    if( !tok->isInactive() ) {
      justification_table::left_const_iterator from, to, i;
      boost::tie(from, to) = m_justified.left.equal_range(tok);
      // check that this one did not exist
      for(i=from; to!=i; ++i)
        if( i->second==just )
          return;
      m_justified.insert(justification_table::value_type(tok, just));
      if( just==tok )
        me->log(s_just)<<"justified(self, "<<tok->getName().toString()<<'('<<tok->getKey()<<"))";
      else
        me->log(s_just)<<"justified("<<tok->getName().toString()<<'('<<tok->getKey()<<") <- "
        <<just->getName().toString()<<'('<<just->getKey()<<"))";
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
}

void dispatch_manager::unjustify(EUROPA::TokenId const &tok, bool silent) {
  boost::shared_ptr<assembly::pimpl> me = m_owner.lock();
  
  // No need to do anything if I have no owner
  if( me && tok.isId() ) {
    bool self = false;
    // remove self-justification
    if( m_justified.erase(justification_table::value_type(tok, tok))>0 ) {
      self = true;
      if( !silent )
        me->log(s_just)<<"unjustify(self, "<<tok->getName().toString()<<'('<<tok->getKey()<<"))";
    }
    // check that it is indeed no longer justified
    if( justified(tok, false) ) {
      if( !silent )
        me->log(tlog::warn)<<"Attempt to unjustify "<<tok->getName().toString()
          <<'('<<tok->getKey()<<") which is still justified by others";
    } else {
      justification_table::right_iterator from, to, i;
      // look the tokens that tok used to justify
      boost::tie(from, to) = m_justified.right.equal_range(tok);
      
      if( !(self || silent) )
        me->log(s_just)<<"unjustify("<<tok->getName().toString()<<'('<<tok->getKey()<<"))";
    
      if( from!=to ) {
        eu::TokenSet to_check;
        for(i=from; i!=to; ++i)
          to_check.insert(i->first);
        
        m_justified.right.erase(from, to);
        
        // propagate the update to token justifed by this
        for(eu::TokenSet::const_iterator t=to_check.begin(); to_check.end()!=t; ++t)
          if( !justified(*t) )
            unjustify(*t);
      }
    }
  }
}

void dispatch_manager::schedulled(EUROPA::TokenId const &tok) {
  boost::shared_ptr<assembly::pimpl> me = m_owner.lock();

  if( me && tok.isId() ) {
    if( 0==m_dispatch.count(tok) ) {
      eu::TokenSet::const_iterator pos;
      bool inserted;
    
      boost::tie(pos, inserted) = m_schedulled.insert(tok);
      if( inserted )
        assembly::pimpl::print_token(me->log(s_disp)<<"schedulled ", *tok);
    }
  }
}


void dispatch_manager::unschedulled(EUROPA::TokenId const &tok) {
  boost::shared_ptr<assembly::pimpl> me = m_owner.lock();

  if( me && tok.isId() ) {
    bool removed = m_schedulled.erase(tok);
    
    if( !removed )
      removed = m_dispatch.erase(tok);

    if( removed ) {
      m_resolved.erase(tok);
      assembly::pimpl::print_token(me->log(s_disp)<<"unschedulle ", *tok);
    }
  }
}

void dispatch_manager::make_dispatchable(eu::TokenId const &tok, bool direct) {
  boost::shared_ptr<assembly::pimpl> me = m_owner.lock();

  if( me && tok.isId() ) {
    dispatch_table::iterator pos;
    bool inserted;
    
    boost::tie(pos, inserted) = m_dispatch.insert(dispatch_table::value_type(tok, direct));
    
    if( !inserted )
      pos->second = direct;
    else {
      if( !m_schedulled.erase(tok) )
        me->log(tlog::warn)<<"Made a token dispatchable before scheduling it";
      assembly::pimpl::print_token(me->log(s_disp)<<"dispatchable ", *tok);
    }
  }
}



void dispatch_manager::add_guarded(eu::TokenId const &tok) {
  m_guarded.insert(tok);
}

void dispatch_manager::remove_guarded(eu::TokenId const &tok) {
  m_guarded.erase(tok);
}


// manipulators

size_t dispatch_manager::postponable(eu::eint date, eu::TokenSet &postpone) {
  boost::shared_ptr<assembly::pimpl> me = m_owner.lock();
  size_t count = 0;
  
  if( me ) {
    eu::IntervalIntDomain const future(date+1, std::numeric_limits<eu::eint>::infinity());
  
    // check all the tokends which are still self-guarded
    for(eu::TokenSet::const_iterator s=m_schedulled.begin(); m_schedulled.end()!=s; ++s) {
      if( is_fact(*s) ) {
        if( m_resolved.insert(*s).second )
          assembly::pimpl::print_token(me->log(s_disp)<<"schedulled->fact ", **s);
      } else if( !(*s)->isInactive() ) {
        eu::TokenSet actions;
        effect_for(*s, actions);
        me->log("GUARD")<<"["<<(*s)->getName().toString()
          <<'('<<(*s)->getKey()<<")]= !singletons";
        
        
        for(eu::TokenSet::const_iterator a=actions.begin(); actions.end()!=a; ++a) {
          // First check if there are further guards
          if( !justified(*a) ) {
            eu::TokenSet const &slaves = (*a)->slaves();
            
            for(eu::TokenSet::const_iterator sl=slaves.begin(); slaves.end()!=sl; ++sl) {
              if( is_condition(*sl) && !justified(*sl) ) {
                me->log("GUARD")<<"guard["<<(*s)->getName().toString()<<'('
                <<(*s)->getKey()<<")] = "<<(*sl)->getName().toString()<<'('
                <<(*sl)->getKey()<<')';
              }
            }
           
          }
          
          
          eu::Domain const &dom =  (*a)->start()->lastDomain();
          
          if( dom.isMember(date) && dom.isMember(date+1) ) {
            if( postpone.insert(*a).second )
              ++count;
          }
        }
        eu::Domain const &dom =  (*s)->start()->lastDomain();
        if( dom.isMember(date) && dom.isMember(date+1) ) {
          if( postpone.insert(*s).second )
            ++count;
        }
      }
    }
  }
  return count;
}


size_t dispatch_manager::do_dispatch(eu::eint date) {
  boost::shared_ptr<assembly::pimpl> me = m_owner.lock();
  size_t count = 0;

  if( me ) {
    for (dispatch_table::const_iterator i=m_dispatch.begin(); m_dispatch.end()!=i; ++i) {
      if( is_fact(i->first) ) {
        if( m_resolved.insert(i->first).second )
          assembly::pimpl::print_token(me->log(s_disp)<<"dispatch->fact ", *(i->first));
      } else {
        if( m_resolved.count(i->first)==0 ) {
          eu::TokenSet actions;
          bool trigger = false;
          eu::TokenId trig_action = eu::TokenId::noId();
        
          effect_for(i->first, actions);
          
          if( actions.empty() )
            trigger = true;
          else {
            for (eu::TokenSet::const_iterator a=actions.begin(); actions.end()!=a; ++a) {
              eu::Domain const &dom = (*a)->start()->lastDomain();
              
              if( dom.getLowerBound()<=date ) {
                bool guarded = false;
              
                if( !justified(*a) ) {
                  eu::TokenSet const &slaves = (*a)->slaves();
                  
                  for(eu::TokenSet::const_iterator s=slaves.begin(); slaves.end()!=s; ++s) {
                    if( is_condition(*s) && !justified(*s) ) {
                      guarded = true;
                      me->log("GUARD")<<"guard["<<i->first->getName().toString()<<'('
                      <<i->first->getKey()<<")] = "<<(*s)->getName().toString()<<'('
                      <<(*s)->getKey()<<')';
                    }
                  }
                }
                if( !guarded ) {
                  // This action is ready to fire
                  trig_action = *a;
                  trigger = true;
                  break;
                }
              } else {
                me->log("GUARD")<<"guard["<<i->first->getName().toString()<<'('
                <<i->first->getKey()<<")] = "<<date<<" < start";
              }
            }
          }
          
          if( trigger ) {
            if( trig_action.isId() )
              me->log(s_disp)<<trig_action->getName().toString()<<'('<<trig_action->getKey()
              <<"): "<<trig_action->start()->toString()<<" -> "
              <<i->first->getName().toString()<<'('<<i->first->getKey()<<") "
              <<i->first->start()->toString();
            else
              me->log(s_disp)<<"nil -> "
              <<i->first->getName().toString()<<'('<<i->first->getKey()<<") "
              <<i->first->start()->toString();
                        
            me->dispatch(i->first, i->second);
            ++count;
            m_resolved.insert(i->first);
          }
        }
      }
    }
  }
  return count;
}


