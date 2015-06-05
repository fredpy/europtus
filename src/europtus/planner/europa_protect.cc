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
#include "europtus/planner/europa_protect.hh"
#include "europtus/planner/exception.hh"

#include <boost/thread/once.hpp>

namespace {
  boost::once_flag o_flag = BOOST_ONCE_INIT;
}


using namespace europtus::planner::details;
namespace asio=boost::asio;

/*
 * class europtus::planner::details::europa_protect
 */

// statics

europa_protect::mutex_type  europa_protect::s_mtx;
boost::scoped_ptr<europa_protect> europa_protect::s_instance;

void europa_protect::make_instance(asio::io_service &io) {
  boost::upgrade_lock<mutex_type> lock(s_mtx);
  if( !s_instance ) {
    boost::upgrade_to_unique_lock<mutex_type> write(lock);
    s_instance.reset(new europa_protect(io));
    std::cout<<"created europa protection"<<std::endl;
  }
}

void europa_protect::init(asio::io_service &io) {
  // ensure that make_instance is called once and only once 
  boost::call_once(o_flag, boost::bind(europa_protect::make_instance,
                                       boost::ref(io)));
}

asio::strand &europa_protect::strand() {
  boost::shared_lock<mutex_type> read(s_mtx);
  if( s_instance )
    return s_instance->m_strand;
  throw exception("Europa protection not initialized");
}

// structors

europa_protect::europa_protect(asio::io_service &io):m_strand(io) {}

europa_protect::~europa_protect() {
  std::cout<<"destroyed europa protection"<<std::endl;
}




