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
#ifndef H_europtus_planner_private_assembly_impl
# define H_europtus_planner_private_assembly_impl

# include "europtus/planner/exception.hh"
# include "europtus/planner/propagator.hh"
# include "europtus/planner/dispatch_manager.hh"

# include "europtus/planner/bits/europa_cfg.hh"

# include <PLASMA/RulesEngineDefs.hh>
# include <PLASMA/PlanDatabase.hh>
# include <PLASMA/Module.hh>
# include <PLASMA/Engine.hh>
# include <PLASMA/Solver.hh>
# include <PLASMA/TemporalNetwork.hh>

# include <trex/utils/log/text_log.hh>

# include <boost/enable_shared_from_this.hpp>
# include <boost/make_shared.hpp>

# include <boost/bimap.hpp>
# include <boost/bimap/multiset_of.hpp>


namespace europtus {
  namespace planner {
  
    class assembly::pimpl :public EUROPA::EngineBase, boost::noncopyable,
    public boost::enable_shared_from_this<assembly::pimpl> {
    public:
      static boost::filesystem::path const s_europa;
      
      typedef EUROPA::SchemaId           schema_type;
      typedef EUROPA::PlanDatabaseId     plan_db_type;
      typedef EUROPA::ConstraintEngineId cstr_eng_type;
      typedef EUROPA::SOLVERS::SolverId  solver_type;
      
      static boost::shared_ptr<pimpl> create(clock &c,
                                             TREX::utils::log::text_log &l) {
        boost::shared_ptr<pimpl> ret = boost::make_shared<pimpl>(boost::ref(c), boost::ref(l));
        ret->initialize();
        return ret;
      }
      
      static std::ostream &print_token(std::ostream &out, EUROPA::Token const &tok);

      
      explicit pimpl(clock &c, TREX::utils::log::text_log &l);
      ~pimpl();
      
      
      static void async_exec(boost::weak_ptr<pimpl> who,
                             boost::function<void (pimpl *)> fn) {
        boost::shared_ptr<pimpl> me(who.lock());
        if( me )
          fn(me.get());
      }
      static void release(boost::shared_ptr<pimpl> &me) {
        me.reset();
      }
      
      void init_clock();
      void final_updated(clock::tick_type val);
      void tick_updated(clock::tick_type val);
      
      void set_log(std::ostream &log);
      void debug_cfg(std::string file);
      void cfg_solver(std::string file);
      bool nddl(std::string path, std::string file);
      
      bool is_planning() const;
      
      void add_obs(TREX::transaction::goal_id g);
      void add_goal(TREX::transaction::goal_id g);
      
      schema_type const &schema() const {
        return m_schema;
      }
      
      TREX::utils::log::stream log(TREX::utils::log::id_type const &what) const;
      TREX::utils::log::stream log() const {
        return log(TREX::utils::log::info);
      }
      
      void reset_plan_time_out();
      void set_plan_time_out(clock::tick_type value);
      
      void schedulled(EUROPA::TokenId tok);
      void unschedulled(EUROPA::TokenId tok);
      void add_dispatchable(EUROPA::TokenId tok, bool direct);
      
      bool justified(EUROPA::TokenId const &tok) const;
      void justify(EUROPA::TokenId const &tok);
      void unjustify(EUROPA::TokenId const &tok);
      
      void dispatch(EUROPA::TokenId const &tok, bool direct);
   
      request_sig &on_dispatch();
      
    private:
      typedef std::multimap<size_t, EUROPA::TokenId> token_almanach;
      token_almanach m_forcefully_injected;
      assembly::request_sig  m_request;
      
      void initialize();
      
      bool is_action(EUROPA::TokenId const &tok) const;
      bool is_predicate(EUROPA::TokenId const &tok) const;
      
      
      bool is_condition(EUROPA::TokenId  const &tok) const;
      bool is_effect(EUROPA::TokenId const &tok) const;
      
      void effect_for(EUROPA::TokenId const &tok, EUROPA::TokenSet &actions) const;
          
      void send_exec();
      void check_guarded();
      
      
      EUROPA::ConstrainedVariableId restrict_global(char const *name,
                                                    char const *type,
                                                    EUROPA::Domain const &base);
      void check_planning();
      void send_step();
      void do_step();
      
      size_t m_steps, m_lost;
      
      void end_plan();
      
      bool have_predicate(EUROPA::ObjectId const &object,
                          std::string &pred) const;
      EUROPA::TokenId new_token(EUROPA::ObjectId const &obj,
                                std::string pred,
                                bool is_fact);
      EUROPA::TokenId new_token(std::string const &object,
                                std::string const &pred,
                                bool is_fact);
      /** @brief Test if a token is a fact
       *
       * @param[in] tok A token
       * @param[in] or_merged_to expand to merged tokens
       *
       * Test if @p tok is a fast or, if @p or_merged_to is @c true, 
       * that it is merged to a token that is a fact.
       *
       * This test is useful to see igf the conditions of an executable 
       * action are established and hence allow the token to be 
       * dispatched.
       *
       * @retval true if tok is a fact (or merged to a fact with 
       *         @p or_merged_to @c true
       * @retval false otherwise
       */
      bool is_fact(EUROPA::TokenId const &tok,
                   bool or_merged_to=true) const;
      
      
      EUROPA::ModuleId m_europtus;
      clock &m_clock;
      mutable boost::optional<clock::tick_type> m_last_log;
      clock::tick_type m_plan_since;
      boost::optional<clock::tick_type> m_max_delay;
      
      
      void init_plan_state();
      void update_state(clock::tick_type date);
      
      schema_type   m_schema;
      plan_db_type  m_plan;
      cstr_eng_type m_cstr;
      solver_type   m_solver;
      
      bool m_planning, m_pending, m_confirmed;
      EUROPA::ConstrainedVariableId m_cur, m_last;
      EUROPA::TimelineId            m_plan_state;
      EUROPA::TokenId               m_plan_tok;
      TREX::utils::log::text_log &m_log;
      
      pimpl();
      
      propagator::id                      m_propagator;
      boost::scoped_ptr<dispatch_manager> m_disp;
      
      boost::optional<clock::tick_type> m_tick_opened;
      std::ofstream m_xml;
    }; // europtus::planner::assmebly::pimpl
    
  }
}

#endif // H_europtus_planner_private_assembly_impl