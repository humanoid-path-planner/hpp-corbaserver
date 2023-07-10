// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
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
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include "hpp/corbaserver/server.hh"

#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <stdlib.h>

#include <hpp/corbaserver/server-plugin.hh>
#include <hpp/core/plugin.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>
#include <iostream>

#include "basic-server.hh"
#include "hpp/corbaserver/conversions.hh"
#include "hpp/corbaserver/servant-base.hh"
#include "hpp/corbaserver/tools-idl.hh"

namespace hpp {
namespace corbaServer {
using CORBA::Exception;
using CORBA::Object_var;
using CORBA::SystemException;
using omniORB::fatalException;

namespace {
void usage(const char* app) {
  std::cerr << "Usage: " << app << " [options] ...\n"
            << "  --name <name>      \n"
            << "  --host <host>      \n"
            << "  --port <port>      \n"
            << "  --help             \n"
            << "  --verbosity <level>\twhere level is a positive integer between 0 (no logs) to 50 (very verbose).\n"
            << "  --benchmark        \tenable benchmarking (written in the logs).\n"
            << "  --single-thread    \n"
            << "  --multi-thread     \n"
            << "\n"
               "Environment variables:\n"
               "- HPP_LOGGINGDIR: change the directory where logs are written.\n"
               "- HPP_HOST: change the default host.\n"
               "- HPP_PORT: change the default port.\n"
            << std::flush;
}

std::string endPoint(const std::string& host, const int port) {
  std::ostringstream oss;
  oss << "::" << host << ':' << port;
  return oss.str();
}
}  // end of anonymous namespace.

class Tools : public virtual POA_hpp::Tools {
 public:
  Tools() : server_(NULL){};

  void setServer(Server* server) { server_ = server; }

  virtual CORBA::Boolean loadServerPlugin(const char* context,
                                          const char* pluginName) {
    try {
      std::string c(context), pn(pluginName);
      return server_->loadPlugin(c, pn);
    } catch (const std::exception& e) {
      throw hpp::Error(e.what());
    }
  }

  virtual CORBA::Boolean createContext(const char* context) {
    try {
      std::string c(context);
      return server_->createContext(c);
    } catch (const std::exception& e) {
      throw hpp::Error(e.what());
    }
  }

  virtual Names_t* getContexts() {
    try {
      std::vector<std::string> contexts(server_->getContexts());
      return toNames_t(contexts);
    } catch (const std::exception& e) {
      throw hpp::Error(e.what());
    }
  }

  bool deleteContext(const char* context) {
    try {
      std::string c(context);
      return server_->deleteContext(c);
    } catch (const std::exception& e) {
      throw hpp::Error(e.what());
    }
  }

  CORBA::Object_ptr getServer(const char* contextName, const char* pluginName,
                              const char* objectName) {
    try {
      std::string c(contextName);
      std::string p(pluginName);
      std::string o(objectName);
      return server_->getServer(c, p, o);
    } catch (const std::exception& e) {
      throw hpp::Error(e.what());
    }
  }

  virtual void deleteServant(const char* id) {
    try {
      CORBA::Object_ptr obj = server_->orb()->string_to_object(id);
      PortableServer::ObjectId_var objectId =
          server_->poa()->reference_to_id(obj);
      // Remove reference in servant object map
      ServantBase_var servant = server_->poa()->id_to_servant(objectId.in());
      server_->removeServant(servant.in());
      // Deactivate object
      server_->poa()->deactivate_object(objectId.in());
    } catch (const std::exception& e) {
      throw hpp::Error(e.what());
    } catch (PortableServer::POA::ObjectNotActive const& e) {
      // This allows to call `deleteServant` twice on the
      // same object. OmniORB and Python does not offer a nice mechanism
      // to avoid calling this method twice. We simply warn because
      // there should be no harm in continuing the program since
      // the request was to delete an object which has already been
      // deleted.
      hppDout(warning,
              "Trying to delete an non existant servant. Object not active: "
                  << id);
    }
  }

  virtual void deleteAllServants() override {
    try {
      server_->clearServantsMap();
    } catch (const std::exception& e) {
      throw hpp::Error(e.what());
    }
  }

  virtual void shutdown() { server_->requestShutdown(false); }

 private:
  Server* server_;
};

Server::Server(core::ProblemSolverPtr_t problemSolver, int argc,
               const char* argv[], bool inMultiThread)
    : multiThread_(inMultiThread),
      nameService_(false),
      problemSolverMap_(new ProblemSolverMap(problemSolver)) {
  parseArguments(argc, argv);

  const char* options[][2] = {{"endPoint", ORBendPoint.c_str()}, {0, 0}};
  tools_ = new corba::Server<Tools>(argc, argv, "", options);
  if (!nameService_) tools_->initOmniINSPOA("hpp-corbaserver");
  tools_->initRootPOA(multiThread_);
  tools_->implementation().setServer(this);
}

Server::Server(ProblemSolverMapPtr_t problemSolverMap, int argc,
               const char* argv[], bool inMultiThread)
    : multiThread_(inMultiThread),
      nameService_(false),
      problemSolverMap_(problemSolverMap) {
  parseArguments(argc, argv);

  const char* options[][2] = {{"endPoint", ORBendPoint.c_str()}, {0, 0}};
  tools_ = new corba::Server<Tools>(argc, argv, "", options);
  if (!nameService_) tools_->initOmniINSPOA("hpp-corbaserver");
  tools_->initRootPOA(multiThread_);
  tools_->implementation().setServer(this);
}

Server::~Server() {}

void Server::parseArguments(int argc, const char* argv[]) {
  mainContextId_ = "corbaserver";

  std::string host = "localhost";
  int port = 13331;
  bool endPointSet = false;
  // Read environment variables
  char* env = getenv("HPP_HOST");
  if (env != NULL) {
    host = env;
    endPointSet = true;
  }
  env = getenv("HPP_PORT");
  if (env != NULL) {
    port = atoi(env);
    endPointSet = true;
  }

  ORBendPoint = endPoint(host, port);

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--name") == 0) {
      if (i < argc - 1) {
        mainContextId_ = argv[i + 1];
        ++i;
      } else
        usage(argv[0]);
      std::cout << "Server main context: " << mainContextId_ << std::endl;
    } else if (strcmp(argv[i], "--help") == 0) {
      usage(argv[0]);
    } else if (strcmp(argv[i], "--single-thread") == 0) {
      multiThread_ = false;
      std::cout << "Switched to single thread mode." << std::endl;
    } else if (strcmp(argv[i], "--multi-thread") == 0) {
      std::cout << "Switched to multi-thread mode." << std::endl;
      multiThread_ = true;
    } else if (strcmp(argv[i], "--use-name-service") == 0) {
      std::cout << "Using name service." << std::endl;
      nameService_ = true;
    } else if (strcmp(argv[i], "--host") == 0) {
      host = argv[++i];
      ORBendPoint = endPoint(host, port);
      endPointSet = true;
    } else if (strcmp(argv[i], "--port") == 0) {
      port = atoi(argv[++i]);
      ORBendPoint = endPoint(host, port);
      endPointSet = true;
    } else if (strcmp(argv[i], "-ORBendPoint") == 0) {
      ORBendPoint = argv[++i];
      endPointSet = true;
    } else if (strcmp(argv[i], "--verbosity") == 0) {
      int verbosityLevel = atoi(argv[++i]);
      ::hpp::debug::setVerbosityLevel(verbosityLevel);
    } else if (strcmp(argv[i], "--benchmark") == 0) {
      ::hpp::debug::enableBenchmark(true);
    }
  }
  if (endPointSet) std::cout << "End point: " << ORBendPoint << std::endl;
}

void Server::startCorbaServer() {
  if (nameService_)
    tools_->startCorbaServer("hpp", "tools");
  else
    tools_->startCorbaServer();

  // Creation of main context
  createContext(mainContextId());
}

bool Server::createContext(const std::string& name) {
  if (contexts_.find(name) != contexts_.end()) return false;

  getContext(name);
  return true;
}

std::vector<std::string> Server::getContexts() const {
  std::vector<std::string> contexts;
  contexts.reserve(contexts_.size());
  for (auto const pair : contexts_) contexts.push_back(pair.first);
  return contexts;
}

bool Server::deleteContext(const std::string& name) {
  return static_cast<bool>(contexts_.erase(name));
}

core::ProblemSolverPtr_t Server::problemSolver() {
  return problemSolverMap_->selected();
}

ProblemSolverMapPtr_t Server::problemSolverMap() { return problemSolverMap_; }

Server::Context& Server::getContext(const std::string& name) {
  if (contexts_.find(name) != contexts_.end()) {
    return contexts_[name];
  }

  Context context;
  context.main = ServerPluginPtr_t(new BasicServer(this));
  ProblemSolverMapPtr_t psm(new ProblemSolverMap(*problemSolverMap_));
  context.main->setProblemSolverMap(psm);
  context.main->startCorbaServer("hpp", name);
  context.plugins[""] = context.main;

  return contexts_.insert(std::make_pair(name, context)).first->second;
}

CORBA::Object_ptr Server::getServer(const std::string& contextName,
                                    const std::string& pluginName,
                                    const std::string& objectName) {
  if (contexts_.find(contextName) == contexts_.end())
    throw std::invalid_argument("No context " + contextName);

  const Context& context = contexts_[contextName];
  ServerPluginMap_t::const_iterator _plugin = context.plugins.find(pluginName);
  if (_plugin == context.plugins.end())
    throw std::invalid_argument("No plugin " + pluginName);

  return _plugin->second->servant(objectName);
}

bool Server::loadPlugin(const std::string& contextName,
                        const std::string& libFilename) {
  // Load the plugin
  std::string lib = core::plugin::findPluginLibrary(libFilename);

  typedef ::hpp::corbaServer::ServerPlugin* (*PluginFunction_t)(Server*);

  // Clear old errors
  const char* error = dlerror();
  // void* library = dlopen(lib.c_str(), RTLD_NOW|RTLD_GLOBAL);
  void* library = dlopen(lib.c_str(), RTLD_NOW);
  error = dlerror();
  if (error != NULL) {
    HPP_THROW(std::runtime_error,
              "Error loading library " << lib << ": " << error);
  }
  if (library == NULL) {
    // uncaught error ?
    HPP_THROW(std::runtime_error,
              "Unknown error while loading library " << lib << ".");
  }

  PluginFunction_t function =
      reinterpret_cast<PluginFunction_t>(dlsym(library, "createServerPlugin"));
  error = dlerror();
  if (error != NULL) {
    HPP_THROW(std::runtime_error,
              "Error loading library " << lib << ": " << error);
  }
  if (function == NULL) {
    HPP_THROW(std::runtime_error,
              "Symbol createServerPlugin of (correctly loaded) library "
                  << lib << " is NULL.");
  }

  // Get the context.
  Context& context = getContext(contextName);

  ServerPluginPtr_t plugin(function(this));
  if (!plugin) return false;
  const std::string name = plugin->name();
  if (context.plugins.find(name) != context.plugins.end()) {
    hppDout(info, "Plugin " << lib << " already loaded.");
    return false;
  }
  context.plugins[name] = plugin;
  plugin->setProblemSolverMap(context.main->problemSolverMap());
  plugin->startCorbaServer("hpp", contextName);

  // I don't think we should do that because the symbols should not be
  // removed... dlclose (library);

  return true;
}

/// \brief If CORBA requests are pending, process them
int Server::processRequest(bool loop) { return tools_->processRequest(loop); }

void Server::requestShutdown(bool wait) { orb()->shutdown(wait); }

PortableServer::Servant Server::getServant(ServantKey servantKey) const {
  ServantKeyToServantMap_t::const_iterator _servant =
      servantKeyToServantMap_.find(servantKey);
  if (_servant == servantKeyToServantMap_.end()) return NULL;
  return _servant->second;
}

void Server::addServantKeyAndServant(ServantKey servantKey,
                                     PortableServer::Servant servant) {
  typedef std::pair<ServantToServantKeyMap_t::iterator, bool> Ret_t;
  Ret_t ret =
      servantToServantKeyMap_.insert(std::make_pair(servant, servantKey));
  if (!ret.second)  // Object not added because it already exists
  {
    std::cerr << "An servant object was lost." << std::endl;
    ret.first->second = servantKey;
  }

  typedef std::pair<ServantKeyToServantMap_t::iterator, bool> Ret2_t;
  Ret2_t ret2 =
      servantKeyToServantMap_.insert(std::make_pair(servantKey, servant));
  if (!ret2.second) ret2.first->second = servant;
}

void Server::removeServant(PortableServer::Servant servant) {
  ServantToServantKeyMap_t::iterator _it =
      servantToServantKeyMap_.find(servant);
  if (_it == servantToServantKeyMap_.end()) return;
  servantKeyToServantMap_.erase(_it->second);
  servantToServantKeyMap_.erase(_it);
}

void Server::clearServantsMap() {
  PortableServer::POA_var _poa(poa());
  for (const auto& pair : servantKeyToServantMap_) {
    PortableServer::ObjectId_var objectId = _poa->servant_to_id(pair.second);
    _poa->deactivate_object(objectId.in());
  }
  servantKeyToServantMap_.clear();
  servantToServantKeyMap_.clear();
}

}  // end of namespace corbaServer.
}  // end of namespace hpp.
