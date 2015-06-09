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
#ifndef H_europtus_dune_clock
# define H_europtus_dune_clock

# include "europtus/rt_clock.hh"

namespace europtus {
  namespace dune {
    
    class steady_clock_t {
    public:
      typedef boost::chrono::nanoseconds duration;
      typedef duration::rep              rep;
      typedef duration::period           period;
      
      typedef boost::chrono::time_point<steady_clock_t> time_point;
      
      static const bool is_steady = true;
      
      static time_point now();
      
    }; // europtus::dune::steady_clock
    
    typedef rt_clock<boost::milli, steady_clock_t> steady_clock;
  } // europtus::dune
} // europtus

namespace boost {
  namespace chrono {
    
    template<class CharT>
    struct clock_string<europtus::dune::steady_clock_t, CharT> {
      static std::basic_string<CharT> name() {
        static const CharT u[] =
        { 'D', 'u', 'n', 'e', '_', 's', 't', 'e', 'a', 'd', 'y', '_', 'c', 'l', 'o', 'c', 'k' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      }
      static std::basic_string<CharT> since() {
        static const CharT u[] =
        { ' ', 's', 'i', 'n', 'c', 'e', ' ', 'b', 'o', 'o', 't' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      }
    }; // boost::chrono::clock_string<europtus::dune::steady_clock_t, >
    
  } // boost::chrono
} // boost

#endif // H_europtus_dune_clock