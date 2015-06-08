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
#ifndef H_europtus_planner_assembly
# define H_europtus_planner_assembly

# include "europtus/clock.hh"
# include "europa_protect.hh"

# include <boost/filesystem/path.hpp>

# include <set>


namespace europtus {
  namespace planner {
    
    
    class assembly :boost::noncopyable {
      class pimpl;
    public:
      typedef boost::filesystem::path path;
      
      
      assembly(boost::asio::io_service &io, clock &c);
      ~assembly();
      
      bool   add_search_path(path p);
      size_t add_search_path(std::string const &p);
      size_t add_search_path(char const *s) {
        if(NULL!=s)
          return add_search_path(std::string(s));
        return 0;
      }
      std::string const &search_path() const;
      
      bool load_nddl(boost::filesystem::path nddl_file);

      
      
    private:
      bool locate(path &p) const;

      
      static void send(boost::weak_ptr<pimpl> who,
                       boost::function<void (pimpl *)> fn,
                       priority_strand::priority_type p);
      static void send(boost::weak_ptr<pimpl> who,
                       boost::function<void (pimpl *)> fn);
      
      
      static void on_clock(boost::weak_ptr<pimpl> who, clock &c,
                           clock::state evt);
      static void on_tick(boost::weak_ptr<pimpl> who, clock &c,
                          clock::tick_type t);
      static void on_final(boost::weak_ptr<pimpl> who, clock &c,
                           clock::tick_type f);
      
      
      typedef details::europa_protect prot;
      typedef std::set<path>          path_set;
    
      boost::shared_ptr<pimpl>  m_impl;
      
      typedef boost::mutex     path_mutex;
      
      mutable path_mutex       m_mtx;
      mutable bool             m_path_fresh;
      mutable std::string      m_europa_path;

      path_set                 m_path;
    }; // europtus::planner::assembly
    
    
  
//    class assembly: public EUROPA::EngineBase, boost::noncopyable {
//    public:
//      typedef EUROPA::SchemaId           schema_type;
//      typedef EUROPA::PlanDatabaseId     plan_db_type;
//      typedef EUROPA::ConstraintEngineId cstr_eng_type;
//      
//      typedef EUROPA::SOLVERS::SolverId  solver_type;
//      
//      explicit assembly(clock &c);
//      ~assembly();
//
//      bool add_search_path(boost::filesystem::path p);
//      size_t add_search_path(std::string p);
//      size_t add_search_path(char const *s) {
//        if( NULL!=s )
//          return add_search_path(std::string(s));
//        return 0;
//      }
//      std::string const &nddl_path() const;
//      
//      
//      void configure(std::string const &solver_cfg);
//      bool load_model(boost::filesystem::path nddl_file);
//
//      schema_type   const &schema()      const {
//        return m_schema;
//      }
//      plan_db_type  const &plan_db()     const {
//        return m_plan;
//      }
//      cstr_eng_type const &cstr_engine() const {
//        return m_cstr;
//      }
//      solver_type const &planner() const {
//        return m_planner;
//      }
//      
//      clock &clk() {
//        return m_clock;
//      }
//      void init_clock_model();
//      
//    private:
//      clock &m_clock;
//      
//      schema_type   m_schema;
//      plan_db_type  m_plan;
//      cstr_eng_type m_cstr;
//      
//      solver_type   m_planner;
//      
//      typedef std::set<boost::filesystem::path> path_set;
//      
//      path_set    m_path;
//      // cache local atttribute to avoid to reggenrate the nddl
//      // path every single time
//      mutable bool        m_path_fresh;
//      mutable std::string m_europa_path;
//      
//      bool locate(boost::filesystem::path &p) const;
//      
//      static boost::filesystem::path const s_europa;
//      
//      assembly();
//      
//      EUROPA::ConstrainedVariableId m_now;
//      void update_tick(clock &c, clock::tick_type t);
//    }; // europtus::planner::assembly
    
  } // europtus::planner
} // europtus

#endif // H_europtus_planner_assembly