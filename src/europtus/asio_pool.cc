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
#include "europtus/asio_pool.hh"

using namespace europtus;
namespace asio = boost::asio;

/*
 * class europtus::asio_pool
 */

// structors

asio_pool::asio_pool() {
  m_active.reset(new asio::io_service::work(m_io));
}

asio_pool::asio_pool(size_t n_threads) {
  m_active.reset(new asio::io_service::work(m_io));
  if( n_threads>0 )
    thread_count(n_threads);
}

asio_pool::~asio_pool() {
  m_active.reset();
  m_pool.join_all();
}

// modifiers

size_t asio_pool::thread_count(size_t n, bool hw_override) {
  if( !hw_override ) {
    size_t max_n = 2*boost::thread::hardware_concurrency();
    if( n>max_n )
      n = max_n;
  }
  size_t cur = thread_count();
  if( cur<n ) {
    spawn(n-cur);
    return thread_count();
  }
  return cur;
}

void asio_pool::spawn(size_t n_threads) {
  for(size_t i=0; i<n_threads; ++i)
    m_pool.create_thread(boost::bind(&asio_pool::thread_task, this));
}

// manipulators

void asio_pool::thread_task() {
  do {
    try {
      m_io.run();
    } catch(...) {
      // silently ignore exceptions for now
      // it should not happen anyway as long as the programmer
      // did properly handle his exceptions
    }
  } while( !m_io.stopped() );
}


