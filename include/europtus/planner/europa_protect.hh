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
#ifndef H_europtus_planner_europa_protect
# define H_europtus_planner_europa_protect

# include <boost/noncopyable.hpp>
# include <boost/scoped_ptr.hpp>
# include <boost/thread/shared_mutex.hpp>

# include "europtus/priority_strand.hh"

namespace europtus {
  namespace planner {
    namespace details {

      /* @brief thread safe protection for europa calls
       *
       * This class stores a gloabl starnd that should be used by any
       * components interacting directly with europa.
       *
       * Europa calls are not thread safe and should be protected 
       * accordingly. This class acts as a singleton that stores 
       * a global strand which should be used for any europa calls. 
       * Indeed a starnd allow to do functions calls while ensuring 
       * that any task executed by this strand will not be concurrent.
       *
       * As this class is a singleton it ensures that all component that
       * use its strand will have its call properly protected from europa 
       * calls (as long as other europa calls are ptrotected the same way)
       */
      class europa_protect :boost::noncopyable {
      public:
        typedef europtus::priority_strand strand_type;
        
        static void init(boost::asio::io_service &io);
        static strand_type &strand();
      
        ~europa_protect();
      private:
        typedef boost::shared_mutex mutex_type;
        
        static mutex_type                        s_mtx;
        static boost::scoped_ptr<europa_protect> s_instance;
        
        
        static void make_instance(boost::asio::io_service &io);
      
        strand_type m_strand;
      
        europa_protect(boost::asio::io_service &io);

        // no implementation
        europa_protect();
      }; // europtus::planner::details::europa_protect
      
    } // europtus::planner::details
  } // europtus::planner
} // europtus

#endif // H_europtus_planner_europa_protect