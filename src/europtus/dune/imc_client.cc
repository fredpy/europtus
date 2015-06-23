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
#include "europtus/date_handler.hh"

#include <boost/signals2/shared_connection_block.hpp>

using namespace europtus::dune;
namespace tlog=TREX::utils::log;
namespace tu=TREX::utils;
namespace tr=TREX::transaction;

namespace asio=boost::asio;
namespace bs=boost::system;
namespace sig2=boost::signals2;
namespace bpt=boost::property_tree;
namespace imc=DUNE::IMC;

namespace {
  class clock_proxy :public TREX::LSTS::ImcAdapter::tick_proxy {
  public:
    clock_proxy(europtus::clock &c):m_clk(c) {
    }
    ~clock_proxy() {}
    
    
    tick_type current_tick() {
      // ask clock tick without update
      return m_clk.current();
    }
    date_type tick_to_date(tick_type const &tck) {
      return m_clk.to_date(tck);
    }
    tick_type date_to_tick(date_type const &date) {
      return m_clk.to_tick(date);
    }
    std::string date_str(tick_type const &tck) {
      return boost::posix_time::to_iso_extended_string(tick_to_date(tck));
    }
    std::string duration_str(tick_type const &tck) {
      duration_type dt = m_clk.tick_duration()*tck;
      typedef TREX::utils::chrono_posix_convert<duration_type> cvt;
      
      typename cvt::posix_duration p_dur = cvt::to_posix(dt);
      std::ostringstream oss;
      oss<<p_dur;
      return oss.str();
    }
    tick_type as_date(std::string const &date) {
      return date_to_tick(TREX::utils::string_cast<date_type>(date));
    }
    tick_type as_duration(std::string const &date) {
      boost::posix_time::time_duration
      dur=TREX::utils::string_cast<boost::posix_time::time_duration>(date);
      typedef TREX::utils::chrono_posix_convert< CHRONO::duration<double> > cvt;
      
      CHRONO::duration<double> ratio = m_clk.tick_duration(),
      val(cvt::to_chrono(dur));
      
      double value = val.count()/ratio.count();
      
      return static_cast<tick_type>(std::floor(value));
    }

  private:
    europtus::clock &m_clk;
    
  };
}


/*
 * class europtus::dune::imc_client
 */

// structors

imc_client::imc_client(tlog::text_log &out)
:m_strand(out.service()), m_polling(false), m_log(out) {}

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


tr::goal_id imc_client::get_token(imc::TrexToken *g, bool is_goal) {
  tr::goal_id ret(new tr::Goal(m_adapter.genericGoal(g, is_goal)));
  
  if( !is_goal ) {
    CHRONO::duration<double> t_stamp(g->getTimeStamp());
    typedef chrono_posix_convert< CHRONO::duration<double> > cvt;
    europtus::clock::date_type
      pdate = boost::posix_time::from_time_t(0)+cvt::to_posix(t_stamp);
    
    ret->restrictStart(m_adapter.time_conv().date_to_tick(pdate));
  }
  return ret;
}



void imc_client::start_imc(int id, int port, clock &clk) {
  if( active() )
    throw exception("imc already connected");
  m_adapter.setTrexId(id);
  m_adapter.set_proxy(new clock_proxy(clk));
  if( !m_adapter.bind(port) ) {
    std::cerr<<"Bind failure to "<<port<<std::endl;
  } else
    std::cout<<"Bound to port "<<port<<std::endl;
  m_conn = clk.on_tick().connect_extended(boost::bind(&imc_client::on_tick,
                                                      this, _1, _2, _3));
}

void imc_client::stop_imc() {
  m_polling = false;
  m_conn.disconnect();
  m_adapter.unbind();
}


void imc_client::on_tick(imc_client::conn const &c,
                         europtus::clock &clk,
                         europtus::clock::tick_type tick) {
  sig2::shared_connection_block lock(c);
  
  if( m_conn!=c )
    c.disconnect(); // in case we connect to more than one clock
                    // we just keep the one attached to this class
  else if( !m_polling ) {
    // we need to reinitiate poll
    m_polling = true;
    m_strand.post(boost::bind(&imc_client::async_poll, this));
  }
}

void imc_client::async_poll() {
  sig2::shared_connection_block lock(m_conn);
  if( m_polling ) {
    UNIQ_PTR<imc::Message> msg(m_adapter.poll());
    
    if( NULL==msg.get() ) {
      // done polling
      m_polling = false;
    } else {
      // post next poll
      m_strand.post(boost::bind(&imc_client::async_poll, this));
      
      // Now process the message
      if( imc::TrexOperation::getIdStatic()==msg->getId() ) {
        imc::TrexOperation *op(static_cast<imc::TrexOperation *>(msg.get()));
        tr::goal_id tok;
        
        log()<<"IMC TREX OP: "<<op->op;

        switch(op->op) {
          case imc::TrexOperation::OP_POST_TOKEN:
            tok = get_token(op->token.get(), false);
            log()<<"Observation: "<<*tok;
            m_tok_sig(fact_t, tok);
            break;
          case imc::TrexOperation::OP_POST_GOAL:
            tok = get_token(op->token.get(), true);
            log()<<"Request: "<<*tok;
            m_tok_sig(rejectable_t, tok);
            break;
          default:
            log(warn)<<"Ignoring TREX messages other than post_goal or post_token";
        }
      } /*else
        log()<<"Ignore IMC type "<<msg->getId();
         */
    }
  }
}


