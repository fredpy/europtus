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
#ifdef H_europtus_time_utils
# ifndef T_europtus_bits_time_utils
#  define T_europtus_bits_time_utils

#  include <limits>
#  include <boost/system/error_code.hpp>
#  include <boost/date_time/posix_time/posix_time_io.hpp>


namespace europtus {
  namespace details {

    template<typename R, class P>
    bool chrono_posix_cvt< ch::duration<R,P> >::s_inited(false);

    
    template<typename R, class P>
    pt::ptime chrono_posix_cvt< ch::duration<R,P> >::s_epoch(pt::from_time_t(0));

    template<typename R, class P>
    typename chrono_posix_cvt< ch::duration<R,P> >::posix_duration
    chrono_posix_cvt< ch::duration<R,P> >::s_pt_max(pt::max_date_time);
    
    template<typename R, class P>
    typename chrono_posix_cvt< ch::duration<R,P> >::chrono_duration
    chrono_posix_cvt< ch::duration<R,P> >::s_ch_max(std::numeric_limits<R>::max());
    
    template<typename R, class P>
    typename chrono_posix_cvt< ch::duration<R,P> >::chrono_duration
    chrono_posix_cvt< ch::duration<R,P> >::to_chrono_impl
    (typename chrono_posix_cvt< ch::duration<R,P> >::posix_duration const &val) {
      ch_hours hh(val.hours());
      ch::minutes mm(val.minutes());
      ch::seconds ss(val.seconds());
      long long sub_s = val.fractional_seconds(),
        max_s = val.ticks_per_second();
      
      if( boost::nano::den!=max_s )
        sub_s = (sub_s * boost::nano::den)/max_s;
      ch::nanoseconds ns(sub_s);
      
      typedef ch::duration<long long, ch::nanoseconds::period> my_ns;
      typename boost::common_type<chrono_duration, my_ns>::type remainder(0);
      
      chrono_duration ret = ch::duration_cast<chrono_duration>(hh), tmp;
      remainder += hh-ret;
      tmp = ch::duration_cast<chrono_duration>(mm);
      ret += tmp;
      remainder += mm-tmp;
      tmp = ch::duration_cast<chrono_duration>(ss);
      ret += tmp;
      remainder += ss-tmp;
      tmp = ch::duration_cast<chrono_duration>(ns);
      ret += tmp;
      remainder += ns-tmp;
      
      ret += ch::duration_cast<chrono_duration>(remainder);
      return ret;
    }
    
    template<typename R, class P>
    typename chrono_posix_cvt< ch::duration<R,P> >::posix_duration
    chrono_posix_cvt< ch::duration<R,P> >::to_posix_impl
    (typename chrono_posix_cvt< ch::duration<R,P> >::chrono_duration const &val) {
      ch_hours hh = ch::duration_cast<ch_hours>(val);
      posix_duration ret = pt::hours(hh.count());

      sub_hour mod_h = val-hh;
      ch::seconds secs = ch::duration_cast<ch::seconds>(mod_h);
      ret += pt::seconds(secs.count());
      
      ch::nanoseconds ns = ch::duration_cast<ch::nanoseconds>(mod_h-secs);
#  ifdef BOOST_DATE_TIME_HAS_NANOSECONDS
      ret += pt::nanoseconds(ns.count());
#  else // !BOOST_DATE_TIME_HAS_NANOSECONDS
      // note I round to the floor, do not know if it is a good idea yet
      ret += pt::microseconds(ns.count()/1000);
#  endif // BOOST_DATE_TIME_HAS_NANOSECONDS
      return ret;
    }

    template<typename R, class P>
    void  chrono_posix_cvt< ch::duration<R,P> >::compute_max() {
      if( !s_inited ) {
        // Identify the longest duration for posix time
        pt::ptime const end_of_time(pt::max_date_time);
        s_pt_max = end_of_time-epoch();
        
        ch_hours const p_max_hours(s_pt_max.hours());
        bool p_high = false, c_high=false;
        
        if( s_ch_max<p_max_hours )
          p_high = true;
        else {
          sub_hour c_mod_h = s_ch_max-p_max_hours;
          if( ch_hours(1)<=c_mod_h ) {
            c_high = true;
          } else {
            // ideally I should double check that posix remains
            // is right below an hour but I know it should be that
            p_high = true;
          }
        }
        
        if( c_high ) {
          // identified that chrono max is larger than posix max
          s_ch_max = to_chrono_impl(s_pt_max);
        } else if( p_high ) {
          // identified that posix max is larger than chrono max
          s_pt_max = to_posix_impl(s_ch_max);
        }
        s_inited = true;
      }
    }

    template<typename R, class P>
    typename chrono_posix_cvt< ch::duration<R,P> >::chrono_duration
    chrono_posix_cvt< ch::duration<R,P> >::to_chrono(typename chrono_posix_cvt< ch::duration<R,P> >:: posix_duration const &pd,
                                     sy::error_code &ec) {
      compute_max();
      if( pd>s_pt_max ) {
        ec = sy::errc::make_error_code(sy::errc::argument_out_of_domain);
        return s_ch_max;
      } else {
        ec.clear();
        return to_chrono_impl(pd);
      }
    }

    template<typename R, class P>
    typename chrono_posix_cvt< ch::duration<R,P> >::posix_duration
    chrono_posix_cvt< ch::duration<R,P> >::to_posix(typename chrono_posix_cvt< ch::duration<R,P> >::chrono_duration const &cd,
                                                    sy::error_code &ec) {
      compute_max();
      if( cd>s_ch_max ) {
        ec = sy::errc::make_error_code(sy::errc::argument_out_of_domain);
        return s_pt_max;
      } else {
        ec.clear();
        return to_posix_impl(cd);
      }
    }
 
  } // europtus::details
} // europtus

# endif // T_europtus_bits_time_utils
#endif // H_europtus_time_utils