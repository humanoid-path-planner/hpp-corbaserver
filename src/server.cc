// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include "hpp/corbaserver/server.hh"

#include <errno.h>
#include <pthread.h>
#include <iostream>
#include <dlfcn.h>

#include <hpp/util/debug.hh>
#include <hpp/core/plugin.hh>
#include <hpp/corbaserver/server-plugin.hh>

#include "server-private.hh"


namespace hpp
{
  namespace corbaServer
  {
    using CORBA::Exception;
    using CORBA::Object_var;
    using CORBA::SystemException;
    using CORBA::ORB_init;
    using CORBA::PolicyList;
    using omniORB::fatalException;

    namespace
    {
      /// \brief Forward logging messages to hpp logging mechanism.
      /// If debug is disabled, CORBA logging will be disabled too.
      ///
      /// Tracing has to be enabled in your ``omniORB.cfg'' to use this
      /// feature.
      /// See ``omniORB configuration and API'' > ``Tracing options''
      /// section of omniORB manual for more information.
      void logFunction (const char* msg);

      void logFunction (const char* hppDebugStatement (msg))
      {
	hppDout (info, "omniORB: " << msg);
      }

      void usage (const char* app)
      {
        std::cerr << "Usage: " << app << " [options] ..." << '\n'
                  << "  --name <name>" << '\n'
                  << "  --help" << '\n'
                  << "  --single-thread" << '\n'
                  << "  --multi-thread"  << std::endl;
      }
    } // end of anonymous namespace.


    Server::Server(core::ProblemSolverPtr_t problemSolver, int argc,
		   const char *argv[], bool inMultiThread) :
      multiThread_ (inMultiThread),
      problemSolverMap_ (new ProblemSolverMap (problemSolver))
    {
      // Register log function.
      omniORB::setLogFunction (&logFunction);

      private_ = new impl::Server;

      parseArguments (argc, argv);

      initORBandServers (argc, argv, multiThread_);
    }

    Server::Server(ProblemSolverMapPtr_t problemSolverMap, int argc,
		   const char *argv[], bool inMultiThread) :
      multiThread_ (inMultiThread),
      problemSolverMap_ (problemSolverMap)
    {
      // Register log function.
      omniORB::setLogFunction (&logFunction);

      private_ = new impl::Server;

      parseArguments (argc, argv);

      initORBandServers (argc, argv, multiThread_);
    }

    /// \brief Shutdown CORBA server
    Server::~Server()
    {
      private_->deactivateAndDestroyServers();
      delete private_;
      private_ = NULL;
    }

    /// CORBA SERVER INITIALIZATION

    void Server::initORBandServers(int argc, const char* argv[],
				   bool inMultiThread)
    {
      Object_var obj;
      PortableServer::ThreadPolicy_var threadPolicy;
      PortableServer::POA_var rootPoa;

      /// ORB init
      private_->orb_ = ORB_init (argc, const_cast<char **> (argv));
      if (is_nil(private_->orb_)) {
	std::string msg ("failed to initialize ORB");
	hppDout (error, msg.c_str ());
	throw std::runtime_error (msg.c_str ());
      }
      /// ORB init
      obj = private_->orb_->resolve_initial_references("RootPOA");

      /// Create thread policy

      //
      // Make the CORBA object single-threaded to avoid GUI krash
      //
      // Create a sigle threaded policy object
      rootPoa = PortableServer::POA::_narrow(obj);

      if (inMultiThread) {
	threadPolicy = rootPoa->create_thread_policy
	  (PortableServer::ORB_CTRL_MODEL);
      }
      else {
	threadPolicy = rootPoa->create_thread_policy
	  (PortableServer::MAIN_THREAD_MODEL);
      }
      /// Duplicate thread policy
      PolicyList policyList;
      policyList.length(1);
      policyList[0] = PortableServer::ThreadPolicy::_duplicate(threadPolicy);

      try {
        private_->poa_ = rootPoa->create_POA
          ("child", PortableServer::POAManager::_nil(), policyList);
      } catch (PortableServer::POA::AdapterAlreadyExists& /*e*/) {
        private_->poa_ = rootPoa->find_POA ("child", false);
      }

      // Destroy policy object
      threadPolicy->destroy();
      private_->createAndActivateServers(this);
    }

    void Server::parseArguments (int argc, const char* argv[])
    {
      mainContextId_ = "corbaserver";

      for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--name") == 0) {
          if (i < argc - 1) {
            mainContextId_ = argv[i+1];
            ++i;
          }
          else              usage(argv[0]);
          std::cout << "Server main context: " << mainContextId_ << std::endl;
        } else if (strcmp(argv[i], "--help") == 0) {
          usage(argv[0]);
        } else if (strcmp(argv[i], "--single-thread") == 0) {
          multiThread_ = false;
          std::cout << "Switched to single thread mode." << std::endl;
        } else if (strcmp(argv[i], "--multi-thread") == 0) {
          std::cout << "Switched to multi-thread mode." << std::endl;
          multiThread_ = true;
        }
      }
    }

    void Server::startCorbaServer()
    {
      // Obtain a reference to objects, and register them in
      // the naming service.
      Object_var robotObj = private_->robotServant_->_this();
      Object_var obstacleObj = private_->obstacleServant_->_this();
      Object_var problemObj = private_->problemServant_->_this();

      private_->createHppContext (mainContextId());
      // Bind robotObj with name Robot to the hppContext:
      CosNaming::Name objectName;
      objectName.length(1);
      objectName[0].id   = (const char*) "basic";   // string copied
      objectName[0].kind = (const char*) "robot"; // string copied

      private_->bindObjectToName(robotObj, objectName);
      private_->robotServant_->_remove_ref();

      // Bind obstacleObj with name Obstacle to the hppContext:
      objectName.length(1);
      objectName[0].id   = (const char*) "basic"; // string copied
      objectName[0].kind = (const char*) "obstacle";   // string copied

      private_->bindObjectToName(obstacleObj, objectName);
      private_->obstacleServant_->_remove_ref();

      // Bind problemObj with name Problem to the hppContext:
      objectName.length(1);
      objectName[0].id   = (const char*) "basic"; // string copied
      objectName[0].kind = (const char*) "problem";   // string copied

      private_->bindObjectToName(problemObj, objectName);
      private_->problemServant_->_remove_ref();

      PortableServer::POAManager_var pman = private_->poa_->the_POAManager();
      pman->activate();
    }

    core::ProblemSolverPtr_t Server::problemSolver ()
    {
      return problemSolverMap_->selected();
    }

    ProblemSolverMapPtr_t Server::problemSolverMap ()
    {
      return problemSolverMap_;
    }

    bool Server::loadPlugin (const std::string& libFilename)
    {
      std::string lib = core::plugin::findPluginLibrary (libFilename);

      typedef ::hpp::corbaServer::ServerPlugin* (*PluginFunction_t) (bool);

      // Clear old errors
      const char* error = dlerror ();
      //void* library = dlopen(lib.c_str(), RTLD_NOW|RTLD_GLOBAL);
      void* library = dlopen(lib.c_str(), RTLD_NOW);
      error = dlerror ();
      if (error != NULL) {
        hppDout (error, "Error loading library " << lib << ": " << error);
        return false;
      }
      if (library == NULL) {
        // uncaught error ?
        return false;
      }

      PluginFunction_t function = reinterpret_cast<PluginFunction_t>(dlsym(library, "createServerPlugin"));
      error = dlerror ();
      if (error != NULL) {
        hppDout (error, "Error loading library " << lib << ":\n" << error);
        return false;
      }
      if (function == NULL) {
        hppDout (error, "Symbol createServerPlugin of (correctly loaded) library " << lib << " is NULL.");
        return false;
      }

      ServerPluginPtr_t plugin (function(multiThread()));
      if (!plugin) return false;
      const std::string name = plugin->name();
      if (plugins_.find (name) != plugins_.end()) {
        hppDout (info, "Plugin " << lib << " already loaded.");
        return false;
      }
      plugins_[name] = plugin;
      plugin->setProblemSolverMap (problemSolverMap_);
      plugin->startCorbaServer ("hpp", mainContextId());

      // I don't think we should do that because the symbols should not be removed...
      // dlclose (library);

      return true;
    }

    /// \brief If CORBA requests are pending, process them
    int Server::processRequest (bool loop)
    {
      if (loop)
	{
	  hppDout (info, "start processing CORBA requests for ever.");
	  private_->orb_->run();
	}
      else
	{
	  if (private_->orb_->work_pending())
	    private_->orb_->perform_work();
	}
      return 0;
    }

    void Server::requestShutdown (bool wait)
    {
      private_->orb_->shutdown (wait);
    }

  } // end of namespace corbaServer.
} // end of namespace hpp.
