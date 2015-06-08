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

# include "europtus/planner/assembly.hh"
# include "europtus/planner/exception.hh"

# include "europtus/planner/bits/europa_cfg.hh"

# include <PLASMA/RulesEngineDefs.hh>
# include <PLASMA/PlanDatabase.hh>
# include <PLASMA/Module.hh>
# include <PLASMA/Engine.hh>
# include <PLASMA/Solver.hh>
# include <PLASMA/TemporalNetwork.hh>

# include <boost/enable_shared_from_this.hpp>
# include <boost/make_shared.hpp>


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
      
      static boost::shared_ptr<pimpl> create(clock &c) {
        return boost::make_shared<pimpl>(boost::ref(c));
      }
      
      explicit pimpl(clock &c);
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
      
      bool nddl(std::string path, std::string file);
      
    private:
      EUROPA::ConstrainedVariableId restict_global(char const *name,
                                                   char const *type,
                                                   EUROPA::Domain const &base);
      
      clock &m_clock;
      
      schema_type   m_schema;
      plan_db_type  m_plan;
      cstr_eng_type m_cstr;
      
      EUROPA::ConstrainedVariableId m_cur, m_last;
      
      pimpl();
      
    }; // europtus::planner::assmebly::pimpl
    
  }
}

#endif // H_europtus_planner_private_assembly_impl