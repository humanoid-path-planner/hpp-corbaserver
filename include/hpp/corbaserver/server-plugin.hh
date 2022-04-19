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

#ifndef HPP_CORBASERVER_SERVER_PLUGIN_HH
#define HPP_CORBASERVER_SERVER_PLUGIN_HH

#include <hpp/corba/template/server.hh>
#include <hpp/corbaserver/config.hh>
#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/problem-solver-map.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/util/exception.hh>
#include <stdexcept>

#define HPP_CORBASERVER_DEFINE_PLUGIN(PluginClassName)  \
  extern "C" {                                          \
  ::hpp::corbaServer::ServerPlugin* createServerPlugin( \
      ::hpp::corbaServer::Server* server) {             \
    return new PluginClassName(server);                 \
  }                                                     \
  }

namespace hpp {
namespace corbaServer {
class HPP_CORBASERVER_DLLAPI ServerPlugin {
 public:
  virtual ~ServerPlugin() {}

  /// Start corba server
  virtual void startCorbaServer(const std::string& contextId,
                                const std::string& contextKind) = 0;

  virtual std::string name() const = 0;

  virtual ::CORBA::Object_ptr servant(const std::string& name) const = 0;

  Server* parent() { return parent_; }

  core::ProblemSolverPtr_t problemSolver() const {
    return problemSolverMap_->selected();
  }

  ProblemSolverMapPtr_t problemSolverMap() const { return problemSolverMap_; }

  /// Set planner that will be controlled by server
  void setProblemSolverMap(ProblemSolverMapPtr_t psMap) {
    problemSolverMap_ = psMap;
  }

 protected:
  ServerPlugin(Server* parent) : parent_(parent) {}

  Server* parent_;
  ProblemSolverMapPtr_t problemSolverMap_;

  template <typename T>
  void initializeTplServer(corba::Server<T>*& server,
                           const std::string& contextId,
                           const std::string& contextKind,
                           const std::string& objectId,
                           const std::string& objectKind) {
    server = new corba::Server<T>(0, NULL);
    server->initRootPOA(parent()->multiThread());

    int ret = parent()->nameService()
                  ? server->startCorbaServer(contextId, contextKind, objectId,
                                             objectKind)
                  : server->startCorbaServer();

    if (ret != 0) {
      throw std::runtime_error("Failed to start corba " + contextId + '.' +
                               contextKind + '/' + objectId + '.' + objectKind +
                               " server.");
    }
  }
};  // class ServerPlugin
}  // namespace corbaServer
}  // namespace hpp

#endif  // HPP_CORBASERVER_SERVER_PLUGIN_HH
