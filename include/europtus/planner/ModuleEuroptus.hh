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
#ifndef H_europtus_planner_ModuleEuroptus
# define H_europtus_planner_ModuleEuroptus

# include "europtus/planner/assembly.hh"
# include "europtus/planner/bits/europa_cfg.hh"

# include <PLASMA/Module.hh>


namespace europtus {
  namespace planner {
    
    /*
     * class used to inject europa extensions
     *
     * This class is injected by assembly into europa as amodule that 
     * can then declare multiple extensions dsuch as flaw handlers,
     * constraints, ...
     */
    class ModuleEuroptus :public EUROPA::Module {
    public:
      ModuleEuroptus(assembly::pimpl *ref);
      ~ModuleEuroptus();
      
      void initialize();
      void uninitialize();
      void initialize(EUROPA::EngineId engine);
      void uninitialize(EUROPA::EngineId engine);
      
    private:
      assembly::pimpl *m_assembly;
    }; // europtus::planner::ModuleEuroptus
    
  }
}

#endif // H_europtus_planner_ModuleEuroptus