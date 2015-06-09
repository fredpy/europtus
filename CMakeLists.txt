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
set(PROJECT_MINOR 0)
set(PROJECT_PATCH 0)

find_package(Europa 
  COMPONENTS PlanDatabase ConstraintEngine TemporalNetwork Utils TinyXml 
  NDDL RulesEngine Solvers System)

if(NOT EUROPA_FOUND)
  message(FATAL_ERROR "Failed to locate Europa planner libraries")
endif(NOT EUROPA_FOUND)    


find_package(Boost 1.47 REQUIRED COMPONENTS
  system filesystem thread date_time program_options chrono)

set(DUNE_HOME $ENV{DUNE_HOME} CACHE PATH
  "Home for the Dune library")

find_library(DUNE_CORE_LIB NAMES dune-core HINTS ${DUNE_HOME}/lib 
  DOCS "Looking for dune core library")
if(NOT DUNE_CORE_LIB)
  message(SEND_ERROR "Failed to locate dune-core library\n"
    "Set your DUNE_HOME where dune is installed.")
endif(NOT DUNE_CORE_LIB)

find_path(DUNE_INCLUDE_DIR "Dune/Dune.hpp" HINTS ${DUNE_HOME}/include)
if(NOT DUNE_INCLUDE_DIR)
  message(SEND_ERROR "Failed to locate dune headers\n"
    "Set your DUNE_HOME where dune is installed.")
endif(NOT DUNE_INCLUDE_DIR)


include(CMakeParseArguments)

macro(truncate_dir var dir base)
  # Make sure that dirt has one and only one trailing slash
  string(REGEX REPLACE "//*" "/" dir "${dir}")
  string(REGEX REPLACE "/$" "" dir "${dir}")
  set(result "${dir}")

  string(REGEX REPLACE "//*" "/" base "${base}")
  string(REGEX REPLACE "/$" "" base "${base}")

  if( dir )
    set(xdir "${dir}/")
    set(xbase "${base}/")
  
    if("${xdir}" STREQUAL "${xbase}")
      set(result "")
    else()
      string(LENGTH "${xdir}" dir_len)
      string(LENGTH "${xbase}" base_len)


      if(dir_len GREATER ${base_len})
	string(SUBSTRING ${xdir} 0 ${base_len} tmp)
	if(tmp STREQUAL "${xbase}")
	  math(EXPR len "${dir_len} - ${base_len}")
	  string(SUBSTRING "${xdir}" ${base_len} ${len} result)
	endif()
      endif()
    endif()
  endif()

  set(${var} "${result}")
endmacro(truncate_dir)

macro(organize_target target)
  get_target_property(src ${target} SOURCES)
       
  foreach(file ${src})
    if(NOT IS_DIRECTORY ${file})
      get_filename_component(dir ${file} PATH)
      get_filename_component(ext ${file} EXT)

      if(dir)
	truncate_dir(short_dir ${dir} ${CMAKE_CURRENT_SOURCE_DIR})
	if(short_dir STREQUAL ${dir})
	  truncate_dir(short_dir ${dir} ${CMAKE_CURRENT_BINARY_DIR})
	endif()
	set(dir "${short_dir}")	
      endif(dir)
      if(dir)
      endif(dir)

      # Check if header
      string(REGEX MATCH "^\\.(h|hh|hpp)$" header "${ext}")
      if(header) 
	if(dir)
	  truncate_dir(h_dir ${dir} "include")
	  string(REPLACE "/" "\\" dir "${h_dir}")
	  source_group("Header\ Files\\${dir}" FILES ${file})
	endif(dir)
      else(header)
	string(REGEX MATCH "^\\.(c|cc|cpp)$" source "${ext}")
	if(source) 
	  if(dir)
	    truncate_dir(c_dir ${dir} "src")
	    string(REPLACE "/" "\\" dir "${c_dir}")
	    source_group("Source\ Files\\${dir}" FILES ${file})
	  endif(dir)
	else(source)
	  string(REGEX MATCH "^\\.tcc$" template "${ext}")
	  if(template)
	    truncate_dir(h_dir ${dir} "include")
	    string(REPLACE "/" "\\" dir "${h_dir}")
	    source_group("Template\ Files\\${dir}" FILES ${file})
	  else(template)
	    string(REPLACE "/" "\\" dir "${dir}")
	    source_group("Other\ Files\\${dir}" FILES ${file})	  
	  endif(template)
	endif(source)
      endif(header)
    endif()
  endforeach(file ${src})
endmacro(organize_target)


include_directories(
  ${CMAKE_SOURCE_DIR}/include # where project c++ headers will be located
  ${CMAKE_BINARY_DIR}/include # where project c++ headers will be generated
  ${Boost_INCLUDE_DIR}
  ${EUROPA_INCLUDE_DIR}
  ${DUNE_INCLUDE_DIR})

configure_file(include/europtus/bits/version.hh.in
  include/europtus/bits/version.hh)
configure_file(include/europtus/planner/bits/europa_config.hh.in
  include/europtus/planner/bits/europa_cfg.hh)

add_executable(europtus
  # source
  src/europtus.cc
  src/europtus/asio_pool.cc
  src/europtus/clock.cc
  src/europtus/priority_strand.cc
  src/europtus/version.cc
  src/europtus/dune/clock.cc
  src/europtus/dune/ll_dist_constraint.cc
  src/europtus/planner/assembly.cc
  src/europtus/planner/europa_protect.cc
  src/europtus/planner/exception.cc
  src/europtus/planner/ModuleEuroptus.cc
  src/europtus/private/priority_strand_impl.cc
  src/europtus/planner/extensions/ceil_constraint.cc
  src/europtus/planner/extensions/deg_to_rad.cc
  src/europtus/planner/private/assembly_impl.cc

  # headers
  include/europtus/asio_pool.hh
  include/europtus/clock.hh
  include/europtus/priority_strand.hh
  include/europtus/rt_clock.hh
  include/europtus/tick_clock.hh
  include/europtus/time_utils.hh
  include/europtus/version.hh
  include/europtus/bits/async_result.hh
  ${CMAKE_BINARY_DIR}/include/europtus/bits/version.hh
  include/europtus/dune/clock.hh
  include/europtus/dune/ll_dist_constraint.hh
  include/europtus/planner/assembly.hh
  include/europtus/planner/europa_protect.hh
  include/europtus/planner/exception.hh
  include/europtus/planner/ModuleEuroptus.hh
  ${CMAKE_BINARY_DIR}/include/europtus/planner/bits/europa_cfg.hh
  include/europtus/planner/extensions/ceil_constraint.hh
  include/europtus/planner/extensions/deg_to_rad.hh
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
  ${EUROPA_LIBRARIES}
  ${DUNE_CORE_LIB}
)
  