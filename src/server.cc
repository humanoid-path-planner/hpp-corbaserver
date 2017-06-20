// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <errno.h>
#include <pthread.h>
#include <iostream>

#include <hpp/util/debug.hh>
#include "hpp/corbaserver/server.hh"

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
    } // end of anonymous namespace.


    Server::Server(core::ProblemSolverPtr_t problemSolver, int argc,
		   const char *argv[], bool inMultiThread) :
      problemSolverMap_ (new ProblemSolverMap (problemSolver))
    {
      // Register log function.
      omniORB::setLogFunction (&logFunction);

      private_ = new impl::Server;

      initORBandServers (argc, argv, inMultiThread);
    }

    Server::Server(ProblemSolverMapPtr_t problemSolverMap, int argc,
		   const char *argv[], bool inMultiThread) :
      problemSolverMap_ (problemSolverMap)
    {
      // Register log function.
      omniORB::setLogFunction (&logFunction);

      private_ = new impl::Server;

      initORBandServers (argc, argv, inMultiThread);
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

    void Server::startCorbaServer(std::string nb_server)
    {
      // Obtain a reference to objects, and register them in
      // the naming service.
      Object_var robotObj = private_->robotServant_->_this();
      Object_var obstacleObj = private_->obstacleServant_->_this();
      Object_var problemObj = private_->problemServant_->_this();

      private_->createHppContext (nb_server);
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

  } // end of namespace corbaServer.
} // end of namespace hpp.
