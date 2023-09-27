// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, Joseph Mirabel,
// JRL.
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

#ifndef HPP_CORBASERVER_SERVER_HH
#define HPP_CORBASERVER_SERVER_HH

#include <boost/thread/mutex.hpp>
#include <omniORB4/CORBA.h>

#include <hpp/corba/template/server.hh>
#include <hpp/corbaserver/config.hh>
#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/problem-solver-map.hh>

namespace hpp {
namespace corbaServer {
/// Implementation of Hpp module Corba server.

///  This class initializes the Corba server and starts the following Corba
///  interface implementations. \li hpp::corbaserver::Robot: to build a
///  hpp::model::Device and to insert it in a hpp::core::ProblemSolver object,
///  \li hpp::corbaserver::Obstacle: to build obstacles and insert them in a
///  hpp::core::ProblemSolver object, \li hpp::corbaserver::Problem: to define a
///  path planning problem and solve it.

///  To use this object, call the constructor

///  \code
///  int argc=1;
///  char *argv[1] = {"program"};
///  core::ProblemSolverPtr_t problemSolver = new core::ProblemSolver;
///  Server server(problemSolver, argc, argv, isMultiThread);
///  \endcode
///  where \c isMultiThread specifies whether the server should process
///  requests using multi-thread policy of not.

///  After starting a name server and configuring your Corba implementation,
///  start the servers.
///  \code
///  server.startCorbaServer();
///  \endcode
///  Then, enter in the loop that handle the Corba requests
///  \code
///  server.processRequest(true);
///  \endcode
///  You can then send request to the servers.
class HPP_CORBASERVER_DLLAPI Server {
 public:
  /// Constructor
  /// \param problemSolver the object that will handle Corba requests.
  /// \param argc, argv parameter to feed ORB initialization.
  /// \param multiThread whether the server may process request using
  ///        multithred policy.
  /// \note It is recommended to configure your Corba implementation
  ///       through environment variables and to set argc to 1 and argv to
  ///       any string.
  /// \note It is highly recommended not to enable multi-thread policy in
  ///       CORBA request processing if this library is run from an openGL
  ///       based GUI, since OpenGL does not support multithreading.
  Server(core::ProblemSolverPtr_t problemSolver, int argc, const char* argv[],
         bool multiThread = false);

  /// Constructor
  /// \param problemSolver the object that will handle Corba requests.
  /// \param multiThread whether the server may process request using
  ///        multithred policy.
  Server(core::ProblemSolverPtr_t problemSolver, bool multiThread = false);

  ///\name CORBA server initialization
  /// \{

  /// Configure using command line argument
  bool parseArguments(int argc, const char* argv[]);

  /// Initialize ORB and CORBA servers.
  void initialize();

  /// \}

  /// \brief Shutdown CORBA server
  ~Server();

  PortableServer::POA_var poa() { return tools_->main_poa(); }

  CORBA::ORB_var orb() { return tools_->orb(); }

  /// \brief Initialize CORBA server to process requests from clients
  /// \return 0 if success, -1 if failure.
  void startCorbaServer();

  /// Get main context ID
  const std::string& mainContextId() const { return mainContextId_; }

  const bool& multiThread() const { return multiThread_; }

  const bool& nameService() const { return nameService_; }

  /// \brief If ORB work is pending, process it
  /// \param loop if true, the function never returns,
  ///             if false, the function processes pending requests and
  ///             returns.
  int processRequest(bool loop);

  /// Request a shutdown
  /// \param wait if true, the method waits for the server to be shut down.
  /// \warning From a servant method, set wait to false. Otherwise the
  ///          application will be deadlocked.
  void requestShutdown(bool wait);

  bool createContext(const std::string& contextName);

  std::vector<std::string> getContexts() const;

  bool deleteContext(const std::string& contextName);

  CORBA::Object_ptr getServer(const std::string& contextName,
                              const std::string& pluginName,
                              const std::string& objectName);

  /// Load a plugin if not already loaded.
  /// \return true if the plugin is correctly loaded, false otherwise (which
  ///         includes the case where the plugin was already loaded).
  bool loadPlugin(const std::string& contextName,
                  const std::string& libFilename);

  ProblemSolverMapPtr_t problemSolverMap();

  core::ProblemSolverPtr_t problemSolver();

  typedef void* ServantKey;

  PortableServer::Servant getServant(ServantKey servantKey) const;

  void addServantKeyAndServant(ServantKey servantKey,
                               PortableServer::Servant servant);

  void removeServant(PortableServer::Servant servant);

  void clearServantsMap();

  void removeExpriredServants();

  std::vector<std::string> getAllObjectIds();

 private:
  corba::Server<Tools>* tools_;

  std::string ORBendPoint;
  std::string mainContextId_;

  bool multiThread_, nameService_;

  /// pointer to core::ProblemSolver Object.
  ///
  /// At initialization, the constructor creates a core::ProblemSolver
  /// object and keeps a pointer to this object. All Corba requests are
  /// processed by this object. Notice that this pointer is passed
  /// to each constructor of implementation classes of the server
  /// Corba interface.
  ProblemSolverMapPtr_t problemSolverMap_;

  typedef shared_ptr<ServerPlugin> ServerPluginPtr_t;
  typedef std::map<std::string, ServerPluginPtr_t> ServerPluginMap_t;
  struct Context {
    ServerPluginPtr_t main;
    ServerPluginMap_t plugins;
  };
  std::map<std::string, Context> contexts_;

  Context& getContext(const std::string& name);

  typedef std::map<ServantKey, PortableServer::Servant>
      ServantKeyToServantMap_t;
  typedef std::map<PortableServer::Servant, ServantKey>
      ServantToServantKeyMap_t;
  ServantKeyToServantMap_t servantKeyToServantMap_;
  ServantToServantKeyMap_t servantToServantKeyMap_;
  // Mutex for accessing servant / servant key maps.
  boost::mutex mutex_;
};
}  // end of namespace corbaServer.
}  // end of namespace hpp.
#endif
