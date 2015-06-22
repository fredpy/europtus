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
#ifndef H_europtus_date_handler
# define H_europtus_date_handler

# include "clock.hh"
# include <trex/domain/IntegerDomain.hh>


namespace europtus {
  
  class date_handler
  :public TREX::transaction::DomainBase::xml_factory::factory_type::producer {
    
    typedef TREX::transaction::DomainBase::xml_factory::factory_type::producer base_type;
  public:
    typedef clock::date_type date_type;
    typedef clock::tick_type tick_type;
    
    date_handler(TREX::utils::Symbol const &tag, clock const &clk);
    ~date_handler();
    
  private:
    typedef boost::optional<date_type> opt_date;
    
    base_type::result_type produce(base_type::argument_type arg) const;
    
    clock const &m_clock;
  }; // europtus::date_handler

  
  class duration_handler
  :public TREX::transaction::DomainBase::xml_factory::factory_type::producer {
    
    typedef TREX::transaction::DomainBase::xml_factory::factory_type::producer base_type;
  public:
    typedef boost::posix_time::time_duration duration_type;
    typedef clock::tick_type                 tick_type;
    
    duration_handler(TREX::utils::Symbol const &tag, clock const &clk);
    ~duration_handler();
    
  private:
    typedef boost::optional<duration_type> opt_dur;
    
    base_type::result_type produce(base_type::argument_type arg) const;
    
    clock const &m_clock;
  }; // europtus::duration_handler

  
} // europtus

#endif // H_europtus_date_handler