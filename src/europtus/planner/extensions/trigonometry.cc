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
#include "europtus/planner/extensions/trigonometry.hh"

# include "europtus/planner/bits/europa_cfg.hh"
# include <PLASMA/ConstrainedVariable.hh>

#include <boost/numeric/interval.hpp>


using namespace europtus::planner;
namespace eu=EUROPA;

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
      
      // std::cerr<<dom.toString()<<" * ["<<lb<<", "<<ub<<"]"<<std::flush;
      
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
      
      // if( dom.isEmpty() )
      // 	std::cerr<<" EMPTY"<<std::endl;
      // else
      // 	std::cerr<<" = "<<dom.toString()<<std::endl;
      
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
  
  inline boost_flt deg_to_rad(boost_flt const &dom) {
    return pi<boost_flt>()*dom/180.0;
  }
//  inline boost_flt rad_to_deg(boost_flt const &dom) {
//    return 180.0*dom/pi<boost_flt>();
//  }
  
}

/*
 * class europtus::planner::cosine_cstr
 */


cosine_cstr::cosine_cstr(eu::LabelStr const &name,
                         eu::LabelStr const &propagatorName,
                         eu::ConstraintEngineId const &cstr,
                         std::vector<eu::ConstrainedVariableId> const &vars)
:eu::Constraint(name, propagatorName, cstr, vars),
m_angle(getCurrentDomain(vars[1])),
m_cos(getCurrentDomain(vars[0])) {
}


cosine_cstr::~cosine_cstr() {
  
}

void cosine_cstr::handleExecute() {
  if( m_angle.intersect(-180.0, 180.0) && m_angle.isEmpty() )
    return;
  if( m_cos.intersect(-1.0, 1.0) && m_cos.isEmpty() )
    return;
  if( m_angle.isSingleton() ) {
    eu::edouble::basis_type tmp = std::cos(M_PI*eu::cast_basis(m_angle.getSingletonValue())/180.0);
    intersect(m_cos, tmp, tmp, 1e-6);
  } else if( m_cos.isSingleton() ) {
    eu::edouble::basis_type tmp = std::acos(eu::cast_basis(m_cos.getSingletonValue()));
    tmp *= 180.0/M_PI;
    intersect(m_angle, -tmp, tmp, 1e-6);
    if( m_angle.getUpperBound()+1e-6 < tmp )
      intersect(m_angle, -tmp, -tmp, 1e-6);
    else if( 1e-6-tmp < m_angle.getLowerBound() )
      intersect(m_angle, tmp, tmp, 1e-6);
  } else {
    boost_flt bang(deg_to_rad(convert(m_angle))), bcos;
    bcos = sin((pi<boost_flt>()/2.0)-bang);
    
    eu::edouble::basis_type c_lo(fmax(-1.0,bcos.lower())),
      c_hi(fmin(1.0, bcos.upper()));
    intersect(m_cos, c_lo, c_hi, 1e-6);
  }
}

/*
 * class europtus::planner::sine_cstr
 */


sine_cstr::sine_cstr(eu::LabelStr const &name,
                     eu::LabelStr const &propagatorName,
                     eu::ConstraintEngineId const &cstr,
                     std::vector<eu::ConstrainedVariableId> const &vars)
:eu::Constraint(name, propagatorName, cstr, vars),
m_angle(getCurrentDomain(vars[1])),
m_sin(getCurrentDomain(vars[0])) {
}


sine_cstr::~sine_cstr() {
  
}

void sine_cstr::handleExecute() {
  if( m_angle.intersect(-180.0, 180.0) && m_angle.isEmpty() )
    return;
  if( m_sin.intersect(-1.0, 1.0) && m_sin.isEmpty() )
    return;
  if( m_angle.isSingleton() ) {
    eu::edouble::basis_type tmp = std::sin(M_PI*eu::cast_basis(m_angle.getSingletonValue())/180.0);
    intersect(m_sin, tmp, tmp, 1e-6);
  } else if( m_sin.isSingleton() ) {
    eu::edouble::basis_type tmp = std::sin(eu::cast_basis(m_sin.getSingletonValue()));
    tmp *= 180.0/M_PI;
    eu::edouble::basis_type a_lo, a_hi;
    
    if( tmp>1e-6 ) {
      a_lo = tmp;
      a_hi = 180.0-tmp;
      
      if( m_angle.getUpperBound()+1e-6<a_hi )
        a_hi = a_lo;
      if( a_lo+1e-6 < m_angle.getLowerBound() )
        a_lo = a_hi;
      intersect(m_angle, a_lo, a_hi, 1e-6);
    } else if( tmp<-1e-6 ) {
      a_lo = -180.0-tmp;
      a_hi = tmp;
      
      if( m_angle.getUpperBound()+1e-6 < a_hi )
        a_hi = a_lo;
      if( a_lo+1e-6 < m_angle.getLowerBound() )
        a_lo = a_hi;
      intersect(m_angle, a_lo, a_hi, 1e-6);
    } else {
      a_lo = -180.0;
      a_hi = 180.0;
      if( m_angle.getLowerBound()-1e-6 > a_lo ) {
        if( m_angle.getLowerBound()<=1e-6 )
          a_lo = 0.0;
        else
          a_lo = 180.0;
      }
      if( m_angle.getUpperBound()+1e-6 < a_hi ) {
        if( m_angle.getUpperBound()>=-1e-6 )
          a_hi = 0.0;
        else
          a_hi = -180.0;
      }
      intersect(m_angle, a_lo, a_hi, 1e-6);
    }
  } else {
    boost_flt bang(deg_to_rad(convert(m_angle))), bsin;
    bsin = sin(bang);
    
    eu::edouble::basis_type c_lo(fmax(-1.0,bsin.lower())),
    c_hi(fmin(1.0, bsin.upper()));
    intersect(m_sin, c_lo, c_hi, 1e-6);
  }
}

