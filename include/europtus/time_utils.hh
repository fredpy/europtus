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
#ifndef H_europtus_time_utils
# define H_europtus_time_utils

# include <boost/chrono/duration.hpp>
# include <boost/date_time/posix_time/posix_time_types.hpp>
# include <boost/system/system_error.hpp>

namespace europtus {

  namespace details {
    
    namespace ch=boost::chrono;
    namespace pt=boost::posix_time;
    namespace sy=boost::system;
    
    template <class ChronoDuration>
    struct chrono_posix_cvt;
    
    template<typename Rep, class Period>
    struct chrono_posix_cvt< ch::duration<Rep,Period> > {
      typedef Rep rep;
    public:
      typedef ch::duration<Rep,Period> chrono_duration;
      typedef pt::time_duration        posix_duration;
      
      typedef ch::duration<typename posix_duration::hour_type,
                           typename ch::hours::period> ch_hours;
      typedef typename boost::common_type<ch_hours, chrono_duration>::type sub_hour;
      
      static posix_duration const &max_posix() {
        compute_max();
        return s_pt_max;
      }
      static chrono_duration const &max_chrono() {
        compute_max();
        return s_ch_max;
      }
      static chrono_duration to_chrono(posix_duration const &pd,
                                       sy::error_code &ec);
      static chrono_duration to_chrono(posix_duration const &pd) {
        sy::error_code ec;
        chrono_duration ret = to_chrono(pd,ec);
        if( ec )
          throw sy::system_error(ec);
        return ret;
      }
      static posix_duration  to_posix(chrono_duration const &cd,
                                      sy::error_code &ec);
      static posix_duration  to_posix(chrono_duration const &cd) {
        
        sy::error_code ec;
        posix_duration ret = to_posix(cd,ec);
        if( ec )
          throw sy::system_error(ec);
        return ret;
      }
      
      static pt::ptime const &epoch() {
        return s_epoch;
      }
      
    private:
      static void compute_max();
      static chrono_duration to_chrono_impl(posix_duration const &val);
      static posix_duration to_posix_impl(chrono_duration const &val);
      
      
      static bool s_inited;
      static pt::ptime       s_epoch;
      static posix_duration  s_pt_max;
      static chrono_duration s_ch_max;
    }; // europtus::details::chrono_posix_cvt<>

    // Note I assume that the epoch date is after posix epoch (Jan 1 1970)
    template<class ChronoDuration>
    class date_cvt {
    public:
      typedef chrono_posix_cvt<ChronoDuration> convert;

      typedef ChronoDuration chrono_date;
      typedef pt::ptime      posix_date;
      
      date_cvt():m_epoch(convert::epoch()), m_shift(0) {}
      explicit date_cvt(pt::ptime const &epoch)
      :m_epoch(epoch) {
        m_shift = convert::to_chrono(m_epoch-convert::epoch());
      }
      ~date_cvt() {}
      
      pt::ptime const &epoch() const {
        return m_epoch;
      }
      void epoch(pt::ptime const &date) {
        m_epoch = date;
        m_shift = convert::to_chrono(date-convert::epoch());
      }
      pt::ptime posix_max() const {
        return convert::epoch()+convert::max_posix();
      }
      chrono_date chrono_max() const {
        return convert::max_chrono()-m_shift;
      }
      
      pt::ptime to_posix(chrono_date const &val) const {
        return epoch()+convert::to_posix(val);
      }
      chrono_date to_chrono(pt::ptime const &val) const {
        return convert::to_chrono(val-epoch());
      }
    
    private:
      pt::ptime                        m_epoch;
      chrono_date                      m_shift;
      
    };
    
  } // europtus::details
  
} // europtus

# include "bits/time_utils.tcc"

#endif // H_europtus_time_utils