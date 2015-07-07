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
#include <boost/asio/ip/tcp.hpp>


using namespace europtus::dune;
namespace tlog=TREX::utils::log;
namespace tu=TREX::utils;
namespace tr=TREX::transaction;

namespace asio=boost::asio;
namespace ip=asio::ip;

namespace bs=boost::system;
namespace sig2=boost::signals2;
namespace bpt=boost::property_tree;
namespace imc=DUNE::IMC;

namespace {
  class clock_proxy :public TREX::LSTS::ImcAdapter::tick_proxy {
  public:
    clock_proxy(europtus::clock &c, tlog::text_log &l):m_clk(c), m_log(l) {
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
      
      cvt::posix_duration p_dur = cvt::to_posix(dt);
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
    tlog::stream log(tu::Symbol const &kind) {
      return m_log("imc", kind);
    }

  private:
    europtus::clock &m_clk;
    tlog::text_log  &m_log;
    
  };
}


/*
 * class europtus::dune::imc_client
 */

// structors

imc_client::imc_client(tlog::text_log &out)
:m_strand(out.service()), m_polling(false), m_log(out),
m_timer(out.service()) {}

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
  log("DBG")<<(*ret);
  
  if( !is_goal ) {
    CHRONO::duration<double> t_stamp(g->getTimeStamp());
    typedef chrono_posix_convert< CHRONO::duration<double> > cvt;
    europtus::clock::date_type
      pdate = boost::posix_time::from_time_t(0)+cvt::to_posix(t_stamp);
    
    ret->restrictStart(m_adapter.time_conv().date_to_tick(pdate));
  }
  return ret;
}



void imc_client::start_imc(int id, int port, std::string const &neptus_ip, int neptus_port,
                           clock &clk) {
  if( active() )
    throw exception("imc already connected");
  m_neptus_ip = neptus_ip;
  m_neptus_port = neptus_port;
  m_adapter.setTrexId(id);
  m_adapter.set_proxy(new clock_proxy(clk, m_log));
  if( !m_adapter.bind(port) ) {
    log(tlog::error)<<"Failed to bind to port "<<port;
    std::cerr<<"Bind failure to "<<port<<std::endl;
  } else {
    log(tlog::info)<<"Listening to IMC using port "<<port;
    std::cout<<"Bound to port "<<port<<std::endl;
  }
  m_announce.reset(new IMC::Announce());
  m_announce->sys_name = "Europtus";
  m_announce->sys_type = SYSTEMTYPE_CCU; // CCU
  m_announce->owner = 0xFFFF;
  // if I find a way to determine my IP ... which is not easy at all
  //m_addapter.service = "imc+udp://"+ip+":"+port+"/"
  m_strand.post(boost::bind(&imc_client::async_announce, this));
  
  m_conn = clk.on_tick().connect_extended(boost::bind(&imc_client::on_tick,
                                                      this, _1, _2, _3));
}

void imc_client::stop_imc() {
  m_polling = false;
  m_conn.disconnect();
  m_timer.cancel();
  m_adapter.unbind();
}

void imc_client::request(TREX::transaction::goal_id g) {
  if( g && active() ) {
    m_strand.post(boost::bind(&imc_client::async_request, this, g));
  }
}

void imc_client::async_request(TREX::transaction::goal_id g) {
  if( active() ) {
    imc::TrexOperation op;
    imc::TrexToken tok;
    
    std::ostringstream oss;
    oss<<g;
    op.goal_id = oss.str();
    
    m_adapter.asImcMessage(m_date, *g, &tok);
    op.token.set(&tok);
    op.op = imc::TrexOperation::OP_POST_GOAL;
    
    
    if( m_neptus_port!=0 && !m_neptus_ip.empty() ) {
      Goal tmp = m_adapter.genericGoal(&tok);
      log()<<'['<<op.goal_id<<"] "<<tmp;
      
      if( m_adapter.send(&op, m_neptus_ip, m_neptus_port) )
        op.toText(log("neptus")<<"sent to "<<m_neptus_ip<<':'<<m_neptus_port<<"\n");
      else
        log(tlog::error)<<"Failed to send goal to "<<m_neptus_ip<<':'<<m_neptus_port;
    }
  }
}


void imc_client::on_tick(imc_client::conn const &c,
                         europtus::clock &clk,
                         europtus::clock::tick_type tick) {
  sig2::shared_connection_block lock(c);
  
  if( m_conn!=c ) {
    c.disconnect(); // in case we connect to more than one clock
                    // we just keep the one attached to this class
  }
  else {
    if( !m_polling ) {
      // we need to reinitiate poll
      m_polling = true;
      m_strand.post(boost::bind(&imc_client::async_poll, this, tick));
    }
  }
}

void imc_client::async_poll(clock::tick_type date) {
  sig2::shared_connection_block lock(m_conn);
  m_date = date;
  if( m_polling ) {
    UNIQ_PTR<imc::Message> msg(m_adapter.poll());
    
    if( NULL==msg.get() ) {
      // done polling
      m_polling = false;
    } else {
      // post next poll
      m_strand.post(boost::bind(&imc_client::async_poll, this, date));
      log("MSG")<<msg->getId();
      
      // Now process the message
      if( imc::TrexOperation::getIdStatic()==msg->getId() ) {
        imc::TrexOperation *op(static_cast<imc::TrexOperation *>(msg.get()));
        tr::goal_id tok;
        
        log()<<"IMC TREX OP: "<<static_cast<unsigned int>(op->op);
        
        

        switch(op->op) {
          case imc::TrexOperation::OP_POST_TOKEN:
            tok = get_token(op->token.get(), false);
            log()<<"Observation: "<<*tok;
            m_tok_sig(fact_t, tok);
            break;
          case imc::TrexOperation::OP_POST_GOAL:
            log()<<"request to be parsed";
            try {
              tok = get_token(op->token.get(), true);
              log()<<"Request: "<<*tok;
              m_tok_sig(rejectable_t, tok);
            } catch(std::exception const &e) {
              log(tlog::error)<<e.what();
            }
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

void imc_client::timer_event(boost::system::error_code const &ec) {
  if( !ec ) {
    m_strand.post(boost::bind(&imc_client::async_announce, this));
  } else
    log("timer")<<ec<<": "<<ec.message()<<" ("<<boost::asio::error::operation_aborted<<")";
}

void imc_client::async_announce() {
  IMC::Announce *ann = m_announce.get();
  if( NULL!=ann ) {
    m_adapter.send(ann, m_neptus_ip, m_neptus_port);
    log("imc")<<"Sent announce";
    m_timer.expires_from_now(boost::posix_time::seconds(10));
    m_timer.async_wait(boost::bind(&imc_client::timer_event, this, _1));
  }
}





