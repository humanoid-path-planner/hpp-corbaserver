// Copyright (C) 2019 CNRS-LAAS
// Author: Joseph Mirabel
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_CORBASERVER_BASIC_SERVER_HH
# define HPP_CORBASERVER_BASIC_SERVER_HH

# include <stdexcept>

# include <hpp/corbaserver/server-plugin.hh>

namespace hpp {
  namespace corbaServer {
    class HPP_CORBASERVER_DLLAPI BasicServer: public ServerPlugin
    {
    public:
      virtual ~ServerPlugin () {}

      /// Start corba server
      virtual void startCorbaServer (const std::string& contextId,
         const std::string& contextKind) = 0;

      virtual std::string name() const = 0;

      core::ProblemSolverPtr_t problemSolver () const
      {
        return problemSolverMap_->selected();
      }

      ProblemSolverMapPtr_t problemSolverMap () const
      {
        return problemSolverMap_;
      }

      /// Set planner that will be controlled by server
      void setProblemSolverMap (ProblemSolverMapPtr_t psMap)
      {
        problemSolverMap_ = psMap;
      }

    protected:
      ServerPlugin (bool multithread) : multithread_ (multithread) {}

      bool multithread_;
      ProblemSolverMapPtr_t problemSolverMap_;
    }; // class ServerPlugin
  } // namespace corbaserver
} // namespace hpp

#endif // HPP_CORBASERVER_SERVER_PLUGIN_HH
