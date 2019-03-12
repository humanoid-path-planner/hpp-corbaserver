// Copyright (C) 2019 CNRS-LAAS
// Author: Joseph Mirabel
//
// This file is part of the hpp-corbaserver.
//
// hpp-corbaserver is free software: you can redistribute
// it and/or modify it under the terms of the GNU Lesser General
// Public License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// hpp-corbaserver is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-corbaserver.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CORBASERVER_SERVER_PLUGIN_HH
# define HPP_CORBASERVER_SERVER_PLUGIN_HH

# include <stdexcept>

# include <hpp/corbaserver/config.hh>
# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/problem-solver-map.hh>

#define HPP_CORBASERVER_DEFINE_PLUGIN(PluginClassName)                         \
    extern "C" {                                                               \
      ::hpp::corbaServer::ServerPlugin* createServerPlugin (::hpp::corbaServer::Server* server)  \
      {                                                                        \
        return new PluginClassName (server);                                   \
      }                                                                        \
    }

namespace hpp {
  namespace corbaServer {
    class HPP_CORBASERVER_DLLAPI ServerPlugin
    {
    public:
      virtual ~ServerPlugin () {}

      /// Start corba server
      virtual void startCorbaServer (const std::string& contextId,
         const std::string& contextKind) = 0;

      virtual std::string name() const = 0;

      Server* parent ()
      {
        return parent_;
      }

      core::ProblemSolverPtr_t problemSolver () const throw (std::logic_error)
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
      ServerPlugin (Server* parent)
        : parent_ (parent) {}

      Server* parent_;
      ProblemSolverMapPtr_t problemSolverMap_;
    }; // class ServerPlugin
  } // namespace corbaserver
} // namespace hpp

#endif // HPP_CORBASERVER_SERVER_PLUGIN_HH
