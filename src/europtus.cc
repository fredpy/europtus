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
#include <iostream>

#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>


#include "europtus/asio_pool.hh"
#include "europtus/dune/clock.hh"
#include "europtus/version.hh"
#include "europtus/planner/assembly.hh"
#include "europtus/planner/exception.hh"

// just ofr testing purpose
#include "europtus/planner/europa_protect.hh"

namespace tlog=TREX::utils::log;
namespace tr=TREX::transaction;
namespace po=boost::program_options;
namespace pco=po::command_line_style;

namespace {
  po::options_description opt("Usage:\n"
                              "  europtus [options] <nddl_file>\n\n"
                              "with options");
  
  std::pair<std::string, std::string> reg_threads(std::string const &s) {
    if( s.find("-j")==0 )
      return std::make_pair(s.substr(1, 1), s.substr(2));
    else
      return std::make_pair(std::string(), std::string());
  }

}

int main(int argc, char *argv[]) {
  // Use dune API to measure tick time
  typedef europtus::dune::steady_clock s_clock;

  // number of threads for asio (1 by default)
  size_t threads=1;
  // tick frequency info (1 second by default)
  unsigned long long hours=0, minutes=0, seconds=1, millis=0;
  
  // default end date (default largest possible date)
  boost::posix_time::ptime final(boost::posix_time::max_date_time);
  
  
  /* ======================================================================== *
   * command line options                                                     *
   * ======================================================================== */

  // cmd line options parsing
  po::options_description hidden("Hidden options"),
    cmd_line;
  
  hidden.add_options()("nddl",
                       po::value<std::string>(),
                       "NDDL plan file");
  po::positional_options_description p;
  p.add("nddl", 1);

  // toto add option and parser for final date
  opt.add_options()
    ("path,I", po::value< std::vector<std::string> >(),
      "add a directory to search path" )
    ("threads,j", po::value<size_t>(&threads)->implicit_value(1),
     "Set the number of threads (minimum is 1)")
    ("hours,H", po::value<unsigned long long>(&hours)->implicit_value(0),
     "Set hours in tick frequency")
    ("minutes,M", po::value<unsigned long long>(&minutes)->implicit_value(0),
     "Set minutes in tick frequency")
    ("seconds,s", po::value<unsigned long long>(&seconds)->implicit_value(1),
     "Set seconds in tick frequency")
    ("ms", po::value<unsigned long long>(&millis)->implicit_value(0),
     "Set milliseconds in tick frequency")
    ("help,h", "Produce this help message")
    ("version,v", "Print version number");
  

  cmd_line.add(opt).add(hidden);
  po::variables_map opt_val;

  try {
    // trigger cmd line parsing
    po::store(po::command_line_parser(argc,argv).style(pco::default_style|
						       pco::allow_long_disguise).options(cmd_line).positional(p).extra_parser(&reg_threads).run(),
	      opt_val);
    // propagate parsing results
    po::notify(opt_val);
						      
  } catch(po::error const &cmd_except) {
    std::cerr<<"Command line error: "<<cmd_except.what()<<'\n'<<opt<<std::endl;
    exit(1);
  }

  if( opt_val.count("help") ) {
    // print help and exit
    std::cout<<"Europa based neptus planner.\n"<<opt<<std::endl;
    exit(0);
  }
  if( opt_val.count("version") ) {
    // print version eand exit
    std::cout<<"Europtus version "<<europtus::version::str()<<std::endl;
    exit(0);
  }
  
  // identify tick frequency from options
  if( opt_val.count("hours") || opt_val.count("minutes") || opt_val.count("ms") ) {
    // special case where any other otpions is set but not seconds
    //   then I need to set seconds to 0
    if( !opt_val.count("seconds") )
      seconds = 0;
  }
  
  s_clock::clock_rate freq(0);
  
  freq += boost::chrono::hours(hours);
  freq += boost::chrono::minutes(minutes);
  freq += boost::chrono::seconds(seconds);
  freq += boost::chrono::milliseconds(millis);
  
  if( freq<=s_clock::clock_rate::zero() ) {
    std::cerr<<"Error click frequency is not strictly positive."<<std::endl;
    exit(1);
  } else {
    typedef europtus::details::chrono_posix_cvt<s_clock::clock_rate> cvt;
    std::cout<<"tick duration set to "<<cvt::to_posix(freq)<<std::endl;
  }
  
  boost::filesystem::path model;
  
  
  if( !opt_val.count("nddl") ) {
    std::cerr<<"Error missing nddl file argument\n"<<opt<<std::endl;
    exit(1);
  } else {
    model = opt_val["nddl"].as<std::string>();
  }
  
  // TODO parse it from options
  final = boost::posix_time::microsec_clock::universal_time()+boost::posix_time::hours(1);
  std::cout<<"Final date specified for "<<final<<std::endl;
  

  /* ======================================================================== *
   * This is where the program really starts                                  *
   * ======================================================================== */
  
  // Create additional threads as specified
  europtus::asio_pool pool(1);
  pool.thread_count(threads, true);
  tlog::text_log log(pool.service());
  
  SHARED_PTR<tlog::out_file> log_f = MAKE_SHARED<tlog::out_file>("europtus.log");
  log.direct_connect(log.stranded(*log_f).track_foreign(log_f));
  

  s_clock clock(freq);
  
  
  // log.msg(tlog::null, tlog::info)<<"Test";
  
  europtus::planner::details::europa_protect::init(pool.service());
  
  
  // Todo: wrap europa calls into a strand
  europtus::planner::assembly europa(pool.service(), clock, log);
  
  // I needed assembly to do this part so this option is parsed after
  // the main inits
  europa.add_search_path(".");
  if( opt_val.count("path") ) {
    std::vector<std::string> const &incs = opt_val["path"].as< std::vector<std::string> >();
    for(std::vector<std::string>::const_iterator i=incs.begin();
        incs.end()!=i; ++i) {
      europa.add_search_path(*i);
    }
  }
  
  // Now we load the model
  try {
    europa.set_debug("Debug.cfg");
    europa.load_solver("PlannerConfig.xml");
    std::cerr<<"Loading model: "<<model<<std::endl;
    europa.load_nddl(model);
    std::cerr<<"Model loaded"<<std::endl;
    
    clock.start();
    
    clock.restrict_end(final);
    
    long long cur;
    
    clock.tick();
    while( clock.active() ) {
      clock.sleep();
      cur = clock.tick();
      
      if( cur>=15 && cur<=22 && cur%2 ) {
        TREX::transaction::Goal obs("game", "ping");
        obs.restrictStart(TREX::transaction::IntegerDomain(cur-(cur%3)));
        europa.observation(obs);
      }
    }
    
    
  } catch(europtus::planner::exception const &e) {
    std::cerr<<"planner exception:"<<e.what()<<std::endl;
    return 1;
  }
  
  return 0;
}
