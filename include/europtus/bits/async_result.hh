/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py.
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
#ifndef H_europtus_bits_async_result
# define H_europtus_bits_async_result

# include <boost/bind.hpp>
# include <boost/optional.hpp>
# include <boost/thread/future.hpp>
# include <boost/utility/result_of.hpp>

namespace europtus {
  
  namespace details {
    
    template<typename Ret>
    class async_task;
    
  } // europtus::details
  
  
  template<typename Ret>
  class async_result;
  
  template<>
  class async_result<void> {
  public:
    ~async_result() {}
    
    bool is_exception() const {
      return bool(m_error);
    }
    bool operator! () const {
      return is_exception();
    }
    boost::exception_ptr const &exception() const {
      return m_error;
    }
    void get() const {
      if( m_error )
        rethrow_exception(m_error);
    }
    void get(boost::promise<void> &p) const {
      if( m_error )
        p.set_exception(m_error);
      else
        p.set_value();
    }
    
  private:
    boost::exception_ptr m_error;
    
    async_result() {}
    
    void set_exception(boost::exception_ptr const &e) {
      m_error = e;
    }
    void set(boost::function<void ()> const &fn) {
      try {
        fn();
      } catch(...) {
        set_exception(boost::current_exception());
      }
    }
    
    template<typename Ret>
    friend class async_result;
    
    friend class details::async_task<void>;
  }; // europtus::async_result<void>
  
  template<typename Ret>
  class async_result {
  public:
    ~async_result() {}
    
    bool is_exception() const {
      return m_except.is_exception();
    }
    bool operator! () const {
      return is_exception();
    }
    boost::exception_ptr const &exception() const {
      return m_except.exception();
    }
    Ret get() const {
      m_except.get();
      return *m_value;
    }
    void get(boost::promise<Ret> &p) const {
      if( is_exception() )
        p.set_exception(exception());
      else
        p.set_value(*m_value);
    }
    
  private:
    async_result() {}
    
    void set_value(Ret const &v) {
      m_value = v;
    }
    void set_exception(boost::exception_ptr const &e) {
      m_except.set_exception(e);
    }
    void set(boost::function<Ret ()> const &fn) {
      m_except.set(boost::bind(&async_result::set_value, this,
                               boost::bind(fn)));
    }
    
    async_result<void>    m_except;
    boost::optional<Ret> m_value;
    
    friend class details::async_task<Ret>;
  }; // europtus::async_result<>
  
  namespace details {
    
    template<typename Fn>
    struct task_helper {
      typedef typename boost::result_of<Fn()>::type return_type;
      typedef boost::shared_future<return_type>     future;
      
      typedef async_result<return_type>                wrapper;
      typedef boost::function<void (wrapper const &)>  handler;
    }; // europtus::details::task_helper<>
    
  } // europtus::details
  
} // europtus

#endif // H_europtus_bits_async_result