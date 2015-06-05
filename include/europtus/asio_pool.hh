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
#ifndef H_europtus_asio_pool
# define H_europtus_asio_pool

# include <boost/asio.hpp>
# include <boost/noncopyable.hpp>
# include <boost/thread.hpp>
# include <boost/smart_ptr.hpp>

namespace europtus {
  
  class asio_pool :boost::noncopyable {
  public:
    asio_pool();
    explicit asio_pool(size_t n_threads);
    ~asio_pool();
    
    boost::asio::io_service &service() {
      return m_io;
    }
    boost::asio::io_service &operator* () {
      return service();
    }
    boost::asio::io_service *operator->() {
      return &service();
    }
    size_t thread_count() const {
      return m_pool.size();
    }
    size_t thread_count(size_t n, bool hw_override=false);
    
  private:
    void spawn(size_t n_threads);
    void thread_task();

    boost::asio::io_service                          m_io;
    boost::scoped_ptr<boost::asio::io_service::work> m_active;
    boost::thread_group                              m_pool;
  }; // europtus::asio_pool
  
  /** @brief synchronize asynchronous call
   *
   * @param s Executing service
   * @param f A function
   *
   * Executes @p f using the asio service @p s and block until
   * @p f completes.
   *
   * @note This method acts as a critical section between the calling
   * thread and @p s as it will block the calling thread until @p f
   * was completed by @p s.
   *
   * @note If the caller is executed by @p s then the call of @p f will
   * be done immediately within this thread.
   *
   * @return the value returned by @p f
   *
   * @throw an exception produced by @p f if any
   */
  template<class Service, typename Ret>
  Ret strand_run(Service &s, boost::function<Ret ()> const &f) {
    boost::packaged_task<Ret> tsk(f);
    boost::unique_future<Ret> result = tsk.get_future();
    
    s.dispatch(boost::bind(&boost::packaged_task<Ret>::operator(),
                           boost::ref(tsk)));
    return result.get();
  }
  
  template<class Service>
  void strand_run(Service &s, boost::function<void ()> const &f) {
    boost::packaged_task<void> tsk(f);
    boost::unique_future<void> result = tsk.get_future();
    
    s.dispatch(boost::bind(&boost::packaged_task<void>::operator(),
                           boost::ref(tsk)));
    result.get();
  }
  
} // europtus

#endif // H_europtus_asio_pool