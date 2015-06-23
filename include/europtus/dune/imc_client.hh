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
#ifndef H_europtus_dune_imc_client
# define H_europtus_dune_imc_client

# include <trex/lsts/ImcAdapter.hh>
# include <trex/utils/TimeUtils.hh>

# include <boost/asio/strand.hpp>
# include <europtus/clock.hh>


namespace europtus {
  namespace dune {
    
    class exception :public std::runtime_error {
    public:
      
      
      exception(std::string const &msg) throw()
      :std::runtime_error(msg) {}
      virtual ~exception() throw() {}
      
    }; // europtus::dune::exception
    
    class imc_client :boost::noncopyable {
    public:
      enum token_type {
        fact_t,
        rejectable_t
      };
      
      typedef boost::signals2::signal<void (token_type, TREX::transaction::goal_id)> token_event;
      
      
      imc_client(TREX::utils::log::text_log &log);
      ~imc_client();
      
      bool active() const;
      
      // NOTE: start_imc is not thread safe. I.e multiple
      // concurrent calls to it may result on undefined behavior
      void start_imc(int id, int port, clock &clk);
      void stop_imc();
      
      TREX::utils::log::stream log(TREX::utils::log::id_type const &what) const;
      TREX::utils::log::stream log() const {
        return log(TREX::utils::log::null);
      }
      
      token_event &on_token() {
        return m_tok_sig;
      }
      
    private:
      goal_id get_token(DUNE::IMC::TrexToken *g, bool is_goal);
      
      typedef boost::signals2::connection conn;
      
      void on_tick(conn const &c,
                   clock &clk, clock::tick_type tick);
      void async_poll();
      
      
      boost::asio::strand         m_strand;
      bool                        m_polling;
      
      conn                        m_conn;
      TREX::LSTS::ImcAdapter      m_adapter;
      TREX::utils::log::text_log &m_log;
      
      token_event                 m_tok_sig;
    }; // class europtus::dune::imc_client
    
  } // europtus::dune
} // europtus

#endif // H_europtus_dune_imc_client