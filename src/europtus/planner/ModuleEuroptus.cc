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
#include "europtus/planner/ModuleEuroptus.hh"

#include "europtus/planner/extensions/ceil_constraint.hh"
#include "europtus/planner/extensions/deg_to_rad.hh"
#include "europtus/dune/ll_dist_constraint.hh"

#include "private/assembly_impl.hh"

#include <PLASMA/CFunctions.hh>


using namespace europtus::planner;
using namespace EUROPA;
using europtus::dune::ll_dist_constraint;

namespace {
  
  DECLARE_FUNCTION_TYPE(ceil_constraint, ceil,
                        "ceilf", EUROPA::IntDT, 1);
  
  DECLARE_FUNCTION_TYPE(ll_dist_constraint, ll_dist, "dist_lat_lon",
                        EUROPA::FloatDT, 4);
  
  DECLARE_FUNCTION_TYPE(deg_to_rad, to_rad,
                        "deg_to_rad_f", EUROPA::FloatDT, 1);


}

/*
 * class europtus::planner::ModuleEuroptus
 */
ModuleEuroptus::ModuleEuroptus(assembly::pimpl *ref)
  :Module("Europtus"),m_assembly(ref) {}

ModuleEuroptus::~ModuleEuroptus() {
}

void ModuleEuroptus::initialize() {
  std::cout<<"europtus injected"<<std::endl;
}

void ModuleEuroptus::uninitialize() {
  std::cout<<"europtus removed"<<std::endl;
  m_assembly = NULL;
}

void ModuleEuroptus::initialize(EngineId engine) {
 ConstraintEngine* ce = (ConstraintEngine*)engine->getComponent("ConstraintEngine");
  CESchema* ceSchema = (CESchema*)engine->getComponent("CESchema");

  REGISTER_CONSTRAINT(ceSchema,
                      ceil_constraint,
                      "ceilf", "Default");
  ce->getCESchema()->registerCFunction((new ceil_constraintFunction())->getId());
  
  REGISTER_CONSTRAINT(ceSchema,
                      ll_dist_constraint,
                      "dist_lat_lon", "Default");
  ce->getCESchema()->registerCFunction((new ll_dist_constraintFunction())->getId());

  REGISTER_CONSTRAINT(ceSchema,
                      deg_to_rad,
                      "deg_to_rad_f", "Default");
  ce->getCESchema()->registerCFunction((new deg_to_radFunction())->getId());

  
  std::cout<<"europtus initialized"<<std::endl;
}

void ModuleEuroptus::uninitialize(EngineId engine) {
  std::cout<<"europtus disabled"<<std::endl;
}
