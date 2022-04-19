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
#define HPP_CORBASERVER_BASIC_SERVER_HH

#include <hpp/corba/template/server.hh>
#include <hpp/corbaserver/server-plugin.hh>
#include <hpp/util/exception.hh>
#include <stdexcept>

#include "obstacle.impl.hh"
#include "problem.impl.hh"
#include "robot.impl.hh"

namespace hpp {
namespace corbaServer {
namespace impl {
class Obstacle;
class Problem;
class Robot;
}  // namespace impl

class HPP_CORBASERVER_LOCAL BasicServer : public ServerPlugin {
 public:
  BasicServer(Server* parent);

  ~BasicServer();

  /// Start corba server
  void startCorbaServer(const std::string& contextId,
                        const std::string& contextKind);

  ::CORBA::Object_ptr servant(const std::string& name) const;

  std::string name() const;

 private:
  corba::Server<impl::Obstacle>* obstacleImpl_;
  corba::Server<impl::Problem>* problemImpl_;
  corba::Server<impl::Robot>* robotImpl_;
};  // class ServerPlugin

BasicServer::BasicServer(Server* parent)
    : ServerPlugin(parent),
      obstacleImpl_(NULL),
      problemImpl_(NULL),
      robotImpl_(NULL) {}

BasicServer::~BasicServer() {
  if (obstacleImpl_) delete obstacleImpl_;
  if (problemImpl_) delete problemImpl_;
  if (robotImpl_) delete robotImpl_;
}

std::string BasicServer::name() const { return "basic"; }

/// Start corba server
void BasicServer::startCorbaServer(const std::string& contextId,
                                   const std::string& contextKind) {
  initializeTplServer(obstacleImpl_, contextId, contextKind, name(),
                      "obstacle");
  initializeTplServer(problemImpl_, contextId, contextKind, name(), "problem");
  initializeTplServer(robotImpl_, contextId, contextKind, name(), "robot");

  obstacleImpl_->implementation().setServer(this);
  problemImpl_->implementation().setServer(this);
  robotImpl_->implementation().setServer(this);
}

::CORBA::Object_ptr BasicServer::servant(const std::string& name) const {
  if (name == "obstacle") return obstacleImpl_->implementation()._this();
  if (name == "problem") return problemImpl_->implementation()._this();
  if (name == "robot") return robotImpl_->implementation()._this();
  throw std::invalid_argument("No servant " + name);
}
}  // namespace corbaServer
}  // namespace hpp

#endif  // HPP_CORBASERVER_SERVER_PLUGIN_HH
