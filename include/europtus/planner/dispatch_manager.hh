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
#ifndef H_europtus_planner_dispatch_manager
# define H_europtus_planner_dispatch_manager

# include "assembly.hh"
# include "europtus/planner/bits/europa_cfg.hh"

# include <PLASMA/PlanDatabaseDefs.hh>

# include <boost/bimap.hpp>
# include <boost/bimap/multiset_of.hpp>


namespace europtus {
  namespace planner {
    
    class dispatch_manager :boost::noncopyable {
      typedef boost::bimaps::multiset_of<EUROPA::TokenId> token_bag;
      typedef boost::bimap<token_bag, token_bag> justification_table;
      typedef std::map<EUROPA::TokenId, bool>    dispatch_table;
      class token_proxy;

    public:
      ~dispatch_manager();
      
      bool justified(EUROPA::TokenId const &tok, bool or_fact=true) const;
      
      bool is_fact(EUROPA::TokenId  const &tok, bool or_merged=true) const;
      bool is_action(EUROPA::TokenId const &tok) const;
      bool is_condition(EUROPA::TokenId const &tok) const;
      bool is_effect(EUROPA::TokenId const &tok) const;
      void effect_for(EUROPA::TokenId const &tok, EUROPA::TokenSet &actions,
                      bool recurse=true) const;
      
      
    private:
      boost::weak_ptr<assembly::pimpl> m_owner;
      justification_table              m_justified;
      dispatch_table                   m_dispatch;
      EUROPA::TokenSet                 m_guarded, m_schedulled, m_resolved;
      
      dispatch_manager(boost::shared_ptr<assembly::pimpl> const &ref,
                       EUROPA::PlanDatabaseId const &plan);

      void justify(EUROPA::TokenId const &tok, EUROPA::TokenId const &just);
      void justify(EUROPA::TokenId const &tok) {
        justify(tok, tok);
      }
      void unjustify(EUROPA::TokenId const &tok, bool silent=false);
      
      void schedulled(EUROPA::TokenId const &tok);
      void unschedulled(EUROPA::TokenId const &tok);
      
      void make_dispatchable(EUROPA::TokenId const &tok, bool direct);
      
      void add_guarded(EUROPA::TokenId const &tok);
      void remove_guarded(EUROPA::TokenId const &tok);
      
      size_t postponable(EUROPA::eint date, EUROPA::TokenSet &postpone);
      size_t do_dispatch(EUROPA::eint date);
      
      boost::scoped_ptr<token_proxy> m_proxy;
      
      friend class assembly::pimpl;
      friend class token_proxy;
    };
  
  }
}


#endif