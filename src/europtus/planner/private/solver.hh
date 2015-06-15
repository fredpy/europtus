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
#ifndef H_europtus_planner_private_solver 
# define H_europtus_planner_private_solver

# include <boost/scoped_ptr.hpp>

# include "europtus/planner/bits/europa_cfg.hh"
# include <PLASMA/PlanDatabaseListener.hh>


namespace europtus {
  namespace planner {
    
    
    class solver {
    public:
      solver(EUROPA::PlanDatabaseId const &db);
      ~solver();
      
      void step();
      void backtrack(size_t n);
      
      size_t step_count() const {
        return m_steps;
      }
      size_t depth() const {
        return m_depth;
      }
      
    private:
      size_t                 m_steps, m_depth;
      EUROPA::PlanDatabaseId m_db;
      
      class ce_listener: public EUROPA::ConstraintEngineListener {
      public:
        ce_listener(solver &me);
        ~ce_listener();
        
      private:
        void notifyRemoved(EUROPA::ConstrainedVariableId const &variable);
        void notifyChanged(EUROPA::ConstrainedVariableId const &variable,
                           EUROPA::DomainListener::ChangeType const &changeType);
        void notifyAdded(EUROPA::ConstraintId const &constraint);
        void notifyRemoved(EUROPA::ConstraintId const &constraint);

        solver &m_self;
      };
      
      
      class db_listener:public EUROPA::PlanDatabaseListener {
      public:
        db_listener(solver &me);
        ~db_listener();
        
      private:
        void notifyAdded(EUROPA::TokenId const &tok);
        void notifyRemoved(EUROPA::TokenId const &tok);
        
        solver &m_self;
      }; // europtus::planner::solver::db_listener
      
      boost::scoped_ptr<db_listener> m_db_listen;
      boost::scoped_ptr<ce_listener> m_ce_listen;
      
      friend class db_listener;
      friend class ce_listener;
    }; // europtus::planner::solver
    
  }
}


#endif // H_europtus_planner_private_solver