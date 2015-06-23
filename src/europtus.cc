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
#include <vector>

#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>


#include "europtus/asio_pool.hh"
#include "europtus/dune/clock.hh"
#include "europtus/dune/imc_client.hh"
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
                              "  \teuroptus [options] <nddl_file>\n"
                              "  \teuroptus @<ini_file> [options] [<nddl_file>]\n\n"
                              "with options");
  
  void new_imc_tok(europtus::planner::assembly &db,
                   europtus::dune::imc_client::token_type type,
                   TREX::transaction::goal_id tok) {
    switch( type ) {
      case europtus::dune::imc_client::fact_t:
        db.observation(tok);
        break;
      case europtus::dune::imc_client::rejectable_t:
        db.request(tok);
        break;
    }
  }
  
  std::pair<std::string, std::string> at_option_parser(std::string const&s) {
    if ('@' == s[0])
      return std::make_pair(std::string("cfg"), s.substr(1));
    else
      return std::pair<std::string, std::string>();
  }
  

}

namespace boost {
  
  void validate(boost::any& v,
                const std::vector<std::string>& values,
                boost::posix_time::ptime * target_type, int) {
    // make sure there was no previous assignment
    po::validators::check_first_occurrence(v);
    
    // extract the string
    std::string const &s = po::validators::get_single_string(values);
    bool full = true;
    
    typedef boost::posix_time::time_input_facet facet;
    
    
    std::istringstream iss(s);
    
    iss.imbue(std::locale(iss.getloc(), new facet("%Y-%m-%dT%H:%M:%S%F")));
    
    boost::posix_time::ptime date(boost::date_time::not_a_date_time),
    today(boost::posix_time::second_clock::universal_time());
    
    if( (iss>>date).fail() ) {
      full = false;
      iss.clear();
      iss.seekg(0);
      
      // try ot check if it just specifies time with no date
      iss.imbue(std::locale(iss.getloc(), new facet("%H:%M:%S%F")));
      
      if( (iss>>date).fail() )
        throw po::validation_error(po::validation_error::invalid_option_value);
      
      boost::posix_time::time_duration delta = date-boost::posix_time::ptime(boost::date_time::min_date_time);
      
      date = boost::posix_time::ptime(today.date());
      date += delta;
      
    }
    // Now look for the time zone
    std::string tz(iss.str().substr(iss.tellg()));
    if( "Z"!=tz && !tz.empty() ) {
      try {
        date -= boost::posix_time::duration_from_string(tz);
      } catch(boost::bad_lexical_cast const &e) {
        throw po::validation_error(po::validation_error::invalid_option_value);
      }
    }
    
    
    if( !full && date<=today )
      // handle the case where the time specified
      // is obvioulsy tomorrow
      date += boost::posix_time::hours(24);
    
    v = boost::any(date);
  }
  
  void validate(boost::any& v,
                const std::vector<std::string>& values,
                boost::posix_time::time_duration * target_type, int) {
    // make sure there was no previous assignment
    po::validators::check_first_occurrence(v);
    
    // extract the string
    std::string const &s = po::validators::get_single_string(values);
    
    v = boost::posix_time::duration_from_string(s);
  }

}


int main(int argc, char *argv[]) {
  // Use dune API to measure tick time
  typedef europtus::dune::steady_clock s_clock;

  // number of threads for asio (2 by default)
  size_t threads=2;
  // tick frequency info (1 second by default)
  unsigned long long hours=0, minutes=0, seconds=1, millis=0;
  unsigned imc_port=7030, imc_id=65432;
  std::string europa_dbg_cfg("Debug.cfg"), europa_solver("PlannerConfig.xml"), log_file("europtus.log");
  
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
    ("cfg", po::value<std::string>()->value_name("<file>"), "equivalent to @<file> where <file> is a ini"
     " file. It allows to set options from a ini file instead of the command line.")
    ("path,I", po::value< std::vector<std::string> >()->value_name("<dir>"),
      "add <dir> to europa search path" )
    ("threads", po::value<size_t>(&threads)->implicit_value(threads)->value_name("<int>"),
     "Set the number of threads (minimum is 2)")
    ("hours", po::value<unsigned long long>(&hours)->implicit_value(0)->value_name("<int>"),
     "Set hours in tick frequency")
    ("minutes", po::value<unsigned long long>(&minutes)->implicit_value(0)->value_name("<int>"),
     "Set minutes in tick frequency")
    ("seconds", po::value<unsigned long long>(&seconds)->implicit_value(1)->value_name("<int>"),
     "Set seconds in tick frequency")
    ("milliseconds", po::value<unsigned long long>(&millis)->implicit_value(0)->value_name("<int>"),
     "Set milliseconds in tick frequency")
    ("port_imc,p", po::value<unsigned>(&imc_port)->implicit_value(imc_port)->value_name("<int>"),
     "Set the UDP port for IMC")
    ("id_imc", po::value<unsigned>(&imc_id)->implicit_value(imc_id)->value_name("<int>"),
     "Set the IMC id for this program")
    ("dbg_europa", po::value<std::string>(&europa_dbg_cfg)->implicit_value(europa_dbg_cfg)->value_name("<file>"),
     "Load europa debug information from <file>.\n"
     "All europa log outputs from this file will be in europa_dbg.log")
    ("solver_cfg", po::value<std::string>(&europa_solver)->implicit_value(europa_solver)->value_name("<file>"),
     "Load europa solver configuration from <file>")
    ("log", po::value<std::string>(&log_file)->implicit_value(log_file)->value_name("<file>"),
     "write log messages in <file>")
    ("end_date", po::value<boost::posix_time::ptime>()->value_name("<date>"),
     "Final tick date in posix format (YYYY-MM-DDTHH:MM:SS[TZ]) "
     "or just time of day (HH:MM:SS[TZ]) "
     "with optional TZ being either Z or the shift (e.g. +01:00)")
    ("duration", po::value<boost::posix_time::time_duration>()->value_name("<duration>"),
     "Mission max duration\n"
     "If both duration and end_date are specified the shortest is taken")
    ("daemon", "run as a daemon")
    ("help", "Produce this help message")
    ("version,v", "Print version number");
  

  cmd_line.add(opt).add(hidden);
  po::variables_map opt_val;

  try {
    // trigger cmd line parsing
    po::store(po::command_line_parser(argc,argv).style(pco::default_style|pco::allow_long_disguise).options(cmd_line).positional(p).extra_parser(&at_option_parser).run(),
	      opt_val);
    // propagate parsing results
    po::notify(opt_val);
						      
  } catch(po::error const &cmd_except) {
    std::cerr<<"Command line error: "<<cmd_except.what()<<'\n'<<opt<<std::endl;
    exit(1);
  }
  

  if( opt_val.count("help") ) {
    // print help and exit
    std::cout<<"Europa based neptus planner.\n"<<opt<<"\n\n"
      <<"Note: all option names can be truncated as long as it does not result on an\n"
      <<"      ambiguous name. For example:\n"
      <<"        europtus -min=2 foo.nddl\n"
      <<"      is valid while:\n"
      <<"        europtus -mi=2 foo.nddl\n"
      <<"      is not due to the confusion between minutes and milliseconds"
      <<std::endl;
    exit(0);
  }
  if( opt_val.count("version") ) {
    // print version eand exit
    std::cout<<"Europtus version "<<europtus::version::str()<<std::endl;
    exit(0);
  }

  if( opt_val.count("cfg") ) {
    std::string cfg_name = opt_val["cfg"].as<std::string>();
    std::cout<<"  - loading cfg \""<<cfg_name<<'\"'<<std::endl;
    std::ifstream cfg(cfg_name.c_str());
    
    if( !cfg ) {
      std::cerr<<"Failed to open file \""<<cfg_name<<"\""<<std::endl;
      exit(1);
    }
    
    po::store(po::parse_config_file(cfg, cmd_line), opt_val);
    po::notify(opt_val);
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
  
  
  if( opt_val.count("daemon") ) {
    pid_t pid = fork();
    if( pid<0 ) {
      std::cerr<<"Failed to spawn the daemon process"<<std::endl;
      exit(2);
    }
    if( pid>0 ) {
      // I was the caller and can safely leave
      exit(0);
    }
    
    // I am the child and need to detach myself
    setsid();
    umask(0);
    
    // fork another time to ensure that I won;t acquire a terminal
    if( (pid=fork()) ) {
      if( pid>0 ) {
        std::cout<<"Spawned dameon with pid "<<pid
        <<"\nmessages should be logged in "<<log_file<<std::endl;
        exit(0);
      } else {
        std::cerr<<"failed to spawn the dameon process (2)"<<std::endl;
        exit(2);
      }
    }
    // I should be the dameon now close all the standard io
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
  }
  
  

  /* ======================================================================== *
   * This is where the program really starts                                  *
   * ======================================================================== */
  
  // Create additional threads as specified
  europtus::asio_pool pool(1);
  
  std::cout<<" - Setting number of threads to "<<threads<<std::endl;
  pool.thread_count(threads, true);
  tlog::text_log log(pool.service());
  
  SHARED_PTR<tlog::out_file> log_f = MAKE_SHARED<tlog::out_file>(log_file);
  log.direct_connect(log.stranded(*log_f).track_foreign(log_f));
  

  s_clock clock(freq);
  
  if( opt_val.count("daemon") ) {
    log(tlog::null, tlog::info)<<"Running as dameon with pid="<<getpid()<<std::endl;
  }

  
  // log.msg(tlog::null, tlog::info)<<"Test";
  
  europtus::planner::details::europa_protect::init(pool.service());
  
  
  // Todo: wrap europa calls into a strand
  europtus::planner::assembly europa(pool.service(), clock, log);
  europtus::dune::imc_client imc(log);
  
  imc.on_token().connect(boost::bind(&new_imc_tok, boost::ref(europa), _1, _2));
  // TODO: set the proper id and port
  imc.start_imc(imc_id, imc_port, clock);
  
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
    europa.set_debug(europa_dbg_cfg);
    europa.load_solver(europa_solver);
    europa.load_nddl(model);
    
    clock.start();
    
    // identify final date based on options
    if( opt_val.count("end_date") ) {
      final = opt_val["end_date"].as<boost::posix_time::ptime>();
      std::cout<<"Final date specified for "<<final<<std::endl;
      clock.restrict_end(final);
    }
    if( opt_val.count("duration") ) {
      boost::posix_time::time_duration dur;
      dur = opt_val["duration"].as<boost::posix_time::time_duration>();
      std::cout<<"Max duration specified as "<<dur<<std::endl;
      typedef TREX::utils::chrono_posix_convert<europtus::clock::duration_type> cvt;
      clock.restrict_final(cvt::to_chrono(dur).count()/clock.tick_duration().count());
    }
    // display reulting info
    std::cout<<" - started at "<<clock.epoch()<<std::endl
    <<" - final at "<<clock.end()<<std::endl
    <<" - duration: "<<(clock.end()-clock.epoch())<<std::endl;
    
    log.msg("clock", tlog::info)<<" - started at "<<clock.epoch()<<std::endl
    <<" - final at "<<clock.end()<<std::endl
    <<" - duration: "<<(clock.end()-clock.epoch())<<std::endl;

    long long cur;
    
    // run the clock this is what produce the ticks
    // which are used both to poll messages and update the plan
    clock.tick();
    while( clock.active() ) {
      clock.sleep();
      cur = clock.tick();
    }
    // Not necessary as destruction does it but always better
    // to leave clean
    imc.stop_imc();
    
  } catch(europtus::planner::exception const &e) {
    log(tlog::null, tlog::error)<<"Planner exception: "<<e.what();
    std::cerr<<"planner exception: "<<e.what()<<std::endl;
    return 1;
  }
  
  return 0;
}
