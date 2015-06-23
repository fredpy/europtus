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
#include "europtus/planner/extensions/deg_to_rad.hh"
#include <PLASMA/Domains.hh>

# include <boost/numeric/interval.hpp>

namespace {
  
  bool
  intersect(EUROPA::Domain& dom, EUROPA::edouble::basis_type lb, EUROPA::edouble::basis_type ub,
            EUROPA::edouble::basis_type decimal_places)
  {
    EUROPA::edouble const p_inf = std::numeric_limits<EUROPA::edouble>::infinity();
    EUROPA::edouble const n_inf = std::numeric_limits<EUROPA::edouble>::minus_infinity();
    EUROPA::edouble const prec = decimal_places;
    
    
    if( lb>ub || lb>=p_inf || ub<=n_inf ) {
      dom.empty();
      return true;
    } else {
      EUROPA::edouble lo = n_inf, hi = p_inf;
      
      if( lb>lo ) {
        lo = lb;
        if( lo > dom.getUpperBound() )
          lo -= prec;
      }
      if( ub<hi ) {
        hi = ub;
        if( hi < dom.getLowerBound() )
          hi += prec;
      }
      bool ret = dom.intersect(lo, hi);
            
      return ret;
    }
  }

  /*
   * A replacement to boost::rounded_arith_opp as it has a bug under clang :
   *   - the to_int call made in the boost version needed to be replaced by
   *     Rounding::to_int to allow the compiler know that this static method
   *     is coming from the base class
   */
  template<class T, class Rounding>
  struct my_rounded_arith_opp: Rounding {
    void init() { this->upward(); }
# define BOOST_DN(EXPR) \
this->downward(); \
T r = this->force_rounding(EXPR); \
this->upward(); \
return r
# define BOOST_NR(EXPR) \
this->to_nearest(); \
T r = this->force_rounding(EXPR); \
this->upward(); \
return r
# define BOOST_UP(EXPR) return this->force_rounding(EXPR)
# define BOOST_UP_NEG(EXPR) return -this->force_rounding(EXPR)
    template<class U> T conv_down(U const &v) { BOOST_UP_NEG(-v); }
    template<class U> T conv_up  (U const &v) { BOOST_UP(v); }
    T add_down(const T& x, const T& y) { BOOST_UP_NEG((-x) - y); }
    T sub_down(const T& x, const T& y) { BOOST_UP_NEG(y - x); }
    T mul_down(const T& x, const T& y) { BOOST_UP_NEG(x * (-y)); }
    T div_down(const T& x, const T& y) { BOOST_UP_NEG(x / (-y)); }
    T add_up  (const T& x, const T& y) { BOOST_UP(x + y); }
    T sub_up  (const T& x, const T& y) { BOOST_UP(x - y); }
    T mul_up  (const T& x, const T& y) { BOOST_UP(x * y); }
    T div_up  (const T& x, const T& y) { BOOST_UP(x / y); }
    T median  (const T& x, const T& y) { BOOST_NR((x + y) / 2); }
    T sqrt_down(const T& x)
    { BOOST_NUMERIC_INTERVAL_using_math(sqrt); BOOST_DN(sqrt(x)); }
    T sqrt_up  (const T& x)
    { BOOST_NUMERIC_INTERVAL_using_math(sqrt); BOOST_UP(sqrt(x)); }
    T int_down(const T& x) { return -Rounding::to_int(-x); }
    T int_up  (const T& x) { return Rounding::to_int(x); }
# undef BOOST_DN
# undef BOOST_NR
# undef BOOST_UP
# undef BOOST_UP_NEG
  };
  
  
  using namespace boost::numeric;
  using namespace interval_lib;
  
  typedef EUROPA::edouble::basis_type eur_dbl;
  typedef rounding_control<eur_dbl> rnd_ctrl;
  typedef my_rounded_arith_opp<eur_dbl, rnd_ctrl> rnd_arith;
  typedef rounded_transc_opp<eur_dbl, rnd_arith> rnd_transc;
  typedef save_state<rnd_transc> my_rnd;
  typedef checking_base<EUROPA::edouble::basis_type> my_check;
  typedef interval<EUROPA::edouble::basis_type, policies<my_rnd, my_check> > boost_flt;
  
  boost_flt convert(EUROPA::Domain const &dom) {
    EUROPA::edouble lo, hi;
    EUROPA::edouble::basis_type b_lo = boost_flt::whole().lower(),
    b_hi = boost_flt::whole().upper();
    dom.getBounds(lo, hi);
    
    if( lo>std::numeric_limits<EUROPA::edouble>::minus_infinity() )
      b_lo = cast_basis(lo);
    if( hi<std::numeric_limits<EUROPA::edouble>::infinity() )
      b_hi = cast_basis(hi);
    return boost_flt(b_lo, b_hi);
  }
  
  inline boost_flt i_deg_to_rad(boost_flt const &dom) {
    return pi<boost_flt>()*dom/180.0;
  }
  inline boost_flt i_rad_to_deg(boost_flt const &dom) {
    return 180.0*dom/pi<boost_flt>();
  }

}

using namespace europtus::planner;
namespace eu=EUROPA;

/*
 * class europtus::planner::ceil_constraint
 */

// structors

deg_to_rad::deg_to_rad(eu::LabelStr const &name,
                                 eu::LabelStr const &propagatorName,
                                 eu::ConstraintEngineId const &cstr,
                                 std::vector<eu::ConstrainedVariableId> const &vars)
:eu::Constraint(name, propagatorName, cstr, vars),
m_rad(getCurrentDomain(vars[0])),
m_deg(getCurrentDomain(vars[1])) {
  checkError(vars.size()==2, "Exactly 2 parameters required.");
}

deg_to_rad::~deg_to_rad() {}

// manipulators

void deg_to_rad::handleExecute() {
  boost_flt b_rad(i_deg_to_rad(convert(m_deg))),
    b_deg(i_rad_to_deg(convert(m_rad)));

  intersect(m_rad, b_rad.lower(), b_rad.upper(), 1.0e-8);
  intersect(m_deg, b_deg.lower(), b_deg.upper(), 1.0e-6);
}
