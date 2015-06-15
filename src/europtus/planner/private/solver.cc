/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Frederic Py.
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
#include "solver.hh"

#include <PLASMA/PlanDatabase.hh>

using namespace europtus::planner;
namespace eu=EUROPA;

/*
 * class europtus::planner::solver::db_listener
 */

solver::db_listener::db_listener(solver &me)
:eu::PlanDatabaseListener(me.m_db), m_self(me) {}


solver::db_listener::~db_listener() {}

void solver::db_listener::notifyAdded(EUROPA::TokenId const &tok) {
  
}

void solver::db_listener::notifyRemoved(EUROPA::TokenId const &tok) {
  
}


/*
 * class europtus::planner::solver::db_listener
 */

solver::ce_listener::ce_listener(solver &me)
:eu::ConstraintEngineListener(me.m_db->getConstraintEngine()), m_self(me) {}


solver::ce_listener::~ce_listener() {}


void solver::ce_listener::notifyRemoved(eu::ConstrainedVariableId const &var) {
  
}


void solver::ce_listener::notifyChanged(eu::ConstrainedVariableId const &var,
                                        eu::DomainListener::ChangeType const &changeType) {
  
}

void solver::ce_listener::notifyAdded(eu::ConstraintId const &constraint) {
  
}

void solver::ce_listener::notifyRemoved(eu::ConstraintId const &constraint) {
  
}


/*
 * class europtus::planner::solver
 */

// structors

solver::solver(EUROPA::PlanDatabaseId const &db)
:m_steps(0), m_depth(0), m_db(db) {
  m_db_listen.reset(new db_listener(*this));
}


solver::~solver() {
  m_db_listen.reset();
}

// manipulators

void solver::step() {
  
}

void solver::backtrack(size_t n) {
  
}
