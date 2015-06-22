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
#include "europtus/dune/imc_client.hh"

#include <boost/signals2/shared_connection_block.hpp>

using namespace europtus::dune;
namespace tlog=TREX::utils::log;

namespace asio=boost::asio;
namespace bs=boost::system;
namespace sig2=boost::signals2;
namespace imc=DUNE::IMC;


/*
 * class europtus::dune::imc_client
 */

// structors

imc_client::imc_client(tlog::text_log &out)
:m_log(out) {}

imc_client::~imc_client() {
  stop_imc();
}

// observers

bool imc_client::active() const {
  return m_conn.connected();
}


// manipulators

tlog::stream imc_client::log(tlog::id_type const &what) const {
  return m_log.msg("imc", what);
}



void imc_client::start_imc(int id, int port, clock &clk) {
  if( active() )
    throw exception("imc already connected");
  m_adapter.setTrexId(id);
  m_adapter.bindAsynchronous(port);
  m_conn = clk.on_tick().connect_extended(boost::bind(&imc_client::on_tick,
                                                      this, _1, _2, _3));
}

void imc_client::stop_imc() {
  m_conn.disconnect();
  m_adapter.unbindAsynchronous();
}


void imc_client::on_tick(imc_client::conn const &c,
                         europtus::clock &clk,
                         europtus::clock::tick_type tick) {
  // ensure that we won;t have mutiple calls of this concurrently
  sig2::shared_connection_block lock(c);

  if( m_conn!=c )
    c.disconnect(); // remove the dangling dude
  else {
    UNIQ_PTR<imc::Message> msg(m_adapter.pollAsynchronous());
    
    if( NULL!=msg.get() ) {
    }
  }
}



//void imc_client::next_poll() {
//  m_timer.expires_from_now(m_wait);
//  m_timer.async_wait(boost::bind(&imc_client::async_poll, this, _1));
//}
//
//
//void imc_client::start_imc(int id, int port,
//                           imc_client::duration_type const &pseudo_freq) {
//  {
//    mutex_type::scoped_lock lock(m_mtx);
//    if( m_running )
//      throw exception("imc_client already connected");
//    m_running = true;
//  }
//  m_adapter.setTrexId(id);
//  m_adapter.bindAsynchronous(port);
//  m_wait = pseudo_freq;
//  next_poll();
//}
//
//
//void imc_client::async_poll(bs::error_code const &ec) {
//  if( !ec ) {
//    // No error: I can do my things
//    UNIQ_PTR<imc::Message> msg(m_adapter.pollAsynchronous());
//    
//    if( NULL!=msg.get() ) {
//      
//    }
//    if( active() )
//      next_poll();
//  } else {
//    stop_imc();
//  }
//}
//
//void imc_client::stop_imc() {
//  {
//    mutex_type::scoped_lock lock(m_mtx);
//    if( !m_running )
//      return;
//    m_running = false;
//  }
//  m_adapter.unbindAsynchronous();
//  m_timer.cancel();
//}


