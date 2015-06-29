# -*- cmake -*- 
#######################################################################
# Software License Agreement (BSD License)                            #
#                                                                     #
#  Copyright (c) 2011, MBARI.                                         #
#  All rights reserved.                                               #
#                                                                     #
#  Redistribution and use in source and binary forms, with or without #
#  modification, are permitted provided that the following conditions #
#  are met:                                                           #
#                                                                     #
#   * Redistributions of source code must retain the above copyright  #
#     notice, this list of conditions and the following disclaimer.   #
#   * Redistributions in binary form must reproduce the above         #
#     copyright notice, this list of conditions and the following     #
#     disclaimer in the documentation and/or other materials provided #
#     with the distribution.                                          #
#   * Neither the name of the TREX Project nor the names of its       #
#     contributors may be used to endorse or promote products derived #
#     from this software without specific prior written permission.   #
#                                                                     #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS #
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT   #
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   #
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE      #
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, #
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,#
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    #
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER    #
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT  #
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN   #
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE     #
# POSSIBILITY OF SUCH DAMAGE.                                         #
#######################################################################

cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake;${CMAKE_SOURCE_DIR}/cmake/Modules/")

########################################################################
# Project definitions                                                  #
########################################################################
project(europtus)

set(PROJECT_MAJOR 0)
set(PROJECT_MINOR 1)
set(PROJECT_PATCH 0)

# Look for boost
find_package(Boost 1.47 REQUIRED COMPONENTS
  system filesystem thread date_time program_options chrono)

# Look for trex
find_package(TREX 0.6.1 REQUIRED)
message(STATUS "Looking for TREX>=0.6.1: ${trex_VERSION}") 

# Check if TREX was compiled with europa core
list(FIND trex_extra_LIBRARIES TREXeuropa_core HAS_TREX_EUROPA)

if(${HAS_TREX_EUROPA} LESS 0)
  message(FATAL_ERROR "TREX was not compiled with europa support")
else()
  message(STATUS "Found TREX europa extensions")
endif()

# Check if TREX was compiled with europa core
list(FIND trex_extra_LIBRARIES LSTSshared HAS_TREX_LSTS)

if(${HAS_TREX_LSTS} LESS 0)
  message(FATAL_ERROR "TREX was not compiled with lsts support")
else()
  message(STATUS "Found TREX lsts extensions")
endif()

include(CMakeParseArguments)
include(europtus_macros)


include_directories(
  ${CMAKE_SOURCE_DIR}/include # where project c++ headers will be located
  ${CMAKE_BINARY_DIR}/include # where project c++ headers will be generated
  ${Boost_INCLUDE_DIR}
  ${EUROPA_INCLUDE_DIR}
  ${trex_INCLUDE_DIRS}
  ${DUNE_INCLUDE_DIR})

add_definitions(${trex_CPPFLAGS})


configure_file(include/europtus/bits/version.hh.in
  include/europtus/bits/version.hh)
configure_file(include/europtus/planner/bits/europa_config.hh.in
  include/europtus/planner/bits/europa_cfg.hh)
# A simple configuration file
configure_file(src/europtus.ini.in europtus.ini)

add_executable(europtus
  # source
  src/europtus.cc
  src/europtus/asio_pool.cc
  src/europtus/clock.cc
  src/europtus/date_handler.cc
  src/europtus/priority_strand.cc
  src/europtus/version.cc
  src/europtus/dune/clock.cc
  src/europtus/dune/imc_client.cc
  src/europtus/planner/assembly.cc
  src/europtus/planner/europa_protect.cc
  src/europtus/planner/exception.cc
  src/europtus/planner/ModuleEuroptus.cc
  src/europtus/planner/extensions/ceil_constraint.cc
  src/europtus/planner/extensions/deg_to_rad.cc
  src/europtus/planner/extensions/do_not_match.cc
  src/europtus/planner/extensions/numeric.cc
  src/europtus/planner/extensions/trigonometry.cc
  src/europtus/planner/extensions/fact_filters.cc
  src/europtus/planner/private/assembly_impl.cc
  src/europtus/private/priority_strand_impl.cc

  # headers
  include/europtus/asio_pool.hh
  include/europtus/clock.hh
  include/europtus/date_handler.hh
  include/europtus/priority_strand.hh
  include/europtus/rt_clock.hh
  include/europtus/tick_clock.hh
  include/europtus/time_utils.hh
  include/europtus/version.hh
  include/europtus/bits/async_result.hh
  ${CMAKE_BINARY_DIR}/include/europtus/bits/version.hh
  include/europtus/dune/clock.hh
  include/europtus/dune/imc_client.hh
  include/europtus/planner/assembly.hh
  include/europtus/planner/europa_protect.hh
  include/europtus/planner/exception.hh
  include/europtus/planner/ModuleEuroptus.hh
  ${CMAKE_BINARY_DIR}/include/europtus/planner/bits/europa_cfg.hh
  include/europtus/planner/extensions/ceil_constraint.hh
  include/europtus/planner/extensions/deg_to_rad.hh
  include/europtus/planner/extensions/do_not_match.hh
  include/europtus/planner/extensions/numeric.hh
  include/europtus/planner/extensions/trigonometry.hh
  include/europtus/planner/extensions/fact_filters.hh
  src/europtus/private/priority_strand_impl.hh
  src/europtus/planner/private/assembly_impl.hh

  # template code
  include/europtus/bits/priority_strand.tcc
  include/europtus/bits/rt_clock.tcc
  include/europtus/bits/time_utils.tcc
  )

organize_target(europtus)

target_link_libraries(europtus
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_THREAD_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_CHRONO_LIBRARY}
  LSTSshared
  TREXeuropa_core
  )


  