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
#ifndef H_europtus_priority_strand
# define H_europtus_priority_strand

# include <boost/shared_ptr.hpp>
# include <boost/asio/strand.hpp>
# include <boost/tuple/tuple.hpp>
# include <boost/system/system_error.hpp>


# include "bits/async_result.hh"


namespace europtus {
  
  
  class priority_strand :boost::noncopyable {
    class pimpl;
  public:
    class task {
    public:
      typedef size_t priority;
      
      bool operator< (task const &other) const;
      
    protected:
      task() {}
      task(priority p):m_level(p) {}
      virtual ~task() {}
      
      virtual void execute() =0;
      
    private:
      boost::optional<priority> m_level;
      
      friend class priority_strand;
    };
    
    typedef task::priority priority_type;
    
    explicit priority_strand(boost::asio::io_service &io,
                             bool active = true);
    explicit priority_strand(boost::shared_ptr<boost::asio::strand> const &s,
                             bool active = true);
    ~priority_strand();
    
    boost::asio::strand &strand();
    
    bool is_active() const;
    void start();
    void stop();
    
    template<typename Fn>
    typename details::task_helper<Fn>::future post(Fn f);
    template<typename Fn>
    typename details::task_helper<Fn>::future post(Fn f, priority_type p);
    template<typename Fn>
    void send(Fn f);
    template<typename Fn>
    void send(Fn f, typename details::task_helper<Fn>::handler handle);
    template<typename Fn>
    void send(Fn f, priority_type p);
    template<typename Fn>
    void send(Fn f, typename details::task_helper<Fn>::handler handle,
              priority_type p);
    size_t tasks() const;
    bool empty() const;
    
    void clear();
    
  private:
    boost::shared_ptr<pimpl> m_impl;
    
    void enqueue(task *tsk);
  }; // europtus::priority_strand
  
} // europtus

#  include "bits/priority_strand.tcc"

#endif // H_europtus_priority_strand