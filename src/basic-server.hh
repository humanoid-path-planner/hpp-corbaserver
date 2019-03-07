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

#ifndef HPP_CORBASERVER_BASIC_SERVER_HH
# define HPP_CORBASERVER_BASIC_SERVER_HH

# include <stdexcept>

# include <hpp/util/exception.hh>

# include <hpp/corba/template/server.hh>
# include <hpp/corbaserver/server-plugin.hh>

# include "robot.impl.hh"
# include "obstacle.impl.hh"
# include "problem.impl.hh"

namespace hpp {
  namespace corbaServer {
    namespace impl {
      class Obstacle;
      class Problem;
      class Robot;
    }

    class HPP_CORBASERVER_LOCAL BasicServer: public ServerPlugin
    {
      public:
        BasicServer (Server* parent, bool multithread);

        ~BasicServer ();

        /// Start corba server
        void startCorbaServer (const std::string& contextId,
            const std::string& contextKind);

        std::string name() const;

      private:
        corba::Server <impl::Obstacle>* obstacleImpl_;
        corba::Server <impl::Problem >* problemImpl_;
        corba::Server <impl::Robot   >* robotImpl_;
    }; // class ServerPlugin

    BasicServer::BasicServer (Server* parent, bool mThd)
      : ServerPlugin (parent, mThd)
      , obstacleImpl_ (new corba::Server <impl::Obstacle> (0, NULL, mThd, "child"))
      , problemImpl_  (new corba::Server <impl::Problem > (0, NULL, mThd, "child"))
      , robotImpl_    (new corba::Server <impl::Robot   > (0, NULL, mThd, "child"))
    {
      obstacleImpl_->implementation ().setServer (this);
      problemImpl_ ->implementation ().setServer (this);
      robotImpl_   ->implementation ().setServer (this);
    }

    BasicServer::~BasicServer ()
    {
      delete obstacleImpl_;
      delete problemImpl_;
      delete robotImpl_;
    }

    std::string BasicServer::name () const
    {
      return "basic";
    }

    /// Start corba server
    void BasicServer::startCorbaServer(const std::string& contextId,
        const std::string& contextKind)
    {
      if (obstacleImpl_->startCorbaServer(contextId, contextKind,
            "basic", "obstacle") != 0) {
        HPP_THROW_EXCEPTION (hpp::Exception,
            "Failed to start corba graph server.");
      }
      if (robotImpl_->startCorbaServer(contextId, contextKind,
            "basic", "robot") != 0) {
        HPP_THROW_EXCEPTION (hpp::Exception,
            "Failed to start corba robot server.");
      }
      if (problemImpl_->startCorbaServer(contextId, contextKind,
            "basic", "problem") != 0) {
        HPP_THROW_EXCEPTION (hpp::Exception,
            "Failed to start corba problem server.");
      }
    }

  } // namespace corbaserver
} // namespace hpp

#endif // HPP_CORBASERVER_SERVER_PLUGIN_HH
