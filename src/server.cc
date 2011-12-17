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

#include <kwsPlus/directPath/kwsPlusSteeringMethodFactory.h>
#include <kwsPlus/directPath/kwsPlusDistanceFactory.h>
#include <kwsPlus/roadmap/kwsPlusDiffusionNodePickerFactory.h>
#include <kwsPlus/roadmap/kwsPlusDiffusionShooterFactory.h>

#include <hpp/util/debug.hh>

#include "hpp/corbaserver/server.hh"

#include "server-private.hh"


//FIXME: remove me.
#define HPPCI_CATCH(msg, ret)						\
  catch(CORBA::SystemException&) {					\
    hppDout (error, "hppCorbaServer: CORBA::SystemException: " << msg);	\
    return ret;								\
  }									\
  catch(CORBA::Exception&) {						\
    hppDout (error, "hppCorbaServer: CORBA::Exception: " << msg);	\
    return ret;								\
  }									\
  catch(omniORB::fatalException& fe) {					\
    hppDout (error, "hppCorbaServer: CORBA::fatalException: " << msg);	\
    return ret;								\
  }									\
  catch(...) {								\
    hppDout (error, "hppCorbaServer: unknown exception: " << msg);	\
    return ret;								\
  }

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

      void logFunction (const char* msg)
      {
	hppDout (info, "omniORB: " << msg);
      }
    } // end of anonymous namespace.


    Server::Server(core::Planner *inHppPlanner, int argc, const char *argv[], bool inMultiThread) :
      hppPlanner(inHppPlanner)
    {
      // Register log function.
      omniORB::setLogFunction (&logFunction);

      attPrivate = new impl::Server;

      initORBandServers (argc, argv, inMultiThread);
      initMapSteeringMethodFactory();
      initMapDistanceFunctionFactory();
      initMapDiffusionNodePickerFactory();
      initMapDiffusionShooterFactory();
    }

    /// \brief Shutdown CORBA server
    Server::~Server()
    {
      attPrivate->deactivateAndDestroyServers();
      attPrivate->orb_->shutdown(0);
      delete attPrivate;
      attPrivate = NULL;
      destroySteeringMethodFactory();
      destroyDistanceFunctionFactory();
      destroyDiffusionNodePickerFactory();
      destroyDiffusionShooterFactory();
    }

    /*
      STEERING METHOD FACTORIES
    */

    void Server::initMapSteeringMethodFactory()
    {
      attMapSteeringMethodFactory["linear"] = new CkwsPlusLinearSteeringMethodFactory;
      attMapSteeringMethodFactory["rs"] = new CkwsPlusRSSteeringMethodFactory(1.0);
    }

    void Server::destroySteeringMethodFactory()
    {
      std::map<std::string, CkwsPlusSteeringMethodFactory*>::iterator start
	= attMapSteeringMethodFactory.begin();
      std::map<std::string, CkwsPlusSteeringMethodFactory*>::iterator end
	= attMapSteeringMethodFactory.end();

      for (std::map<std::string, CkwsPlusSteeringMethodFactory*>::iterator it=start;
	   it != end; it++) {
	CkwsPlusSteeringMethodFactory* factory = it->second;
	hppDout (info, "deleting steering method factory" << it->first);
	delete factory;
      }
    }

    bool Server::steeringMethodFactoryAlreadySet(std::string inName)
    {
      if (attMapSteeringMethodFactory.count(inName) == 1) {
	return true;
      }
      return false;
    }


    bool Server::addSteeringMethodFactory(std::string inName,
						CkwsPlusSteeringMethodFactory* inSteeringMethodFactory)
    {
      if(steeringMethodFactoryAlreadySet(inName)) {
	return false;
      }
      attMapSteeringMethodFactory[inName] = inSteeringMethodFactory;
      return true;
    }

    CkppSteeringMethodComponentShPtr
    Server::createSteeringMethod(std::string inName, bool inOriented)
    {
      CkppSteeringMethodComponentShPtr result;

      if (steeringMethodFactoryAlreadySet(inName)) {
	result =
	  attMapSteeringMethodFactory[inName]->makeSteeringMethod(inOriented);
      }
      return result;
    }

    /*
      DISTANCE FUNCTION FACTORIES
    */

    void Server::initMapDistanceFunctionFactory()
    {
      attMapDistanceFunctionFactory["linear"] = new CkwsPlusLinearDistanceFactory;
      attMapDistanceFunctionFactory["rs"] = new CkwsPlusRSDistanceFactory(1.0);
    }

    void Server::destroyDistanceFunctionFactory()
    {
      std::map<std::string, CkwsPlusDistanceFactory*>::iterator start
	= attMapDistanceFunctionFactory.begin();
      std::map<std::string, CkwsPlusDistanceFactory*>::iterator end
	= attMapDistanceFunctionFactory.end();

      for (std::map<std::string, CkwsPlusDistanceFactory*>::iterator it=start;
	   it != end; it++) {
	CkwsPlusDistanceFactory* factory = it->second;
	hppDout (info, " deleting distance function factory" << it->first);
	delete factory;
      }
    }

    bool Server::distanceFactoryAlreadySet(std::string inName)
    {
      if (attMapDistanceFunctionFactory.count(inName) == 1) {
	return true;
      }
      return false;
    }


    bool Server::addDistanceFactory(std::string inName,
					  CkwsPlusDistanceFactory* inDistanceFunctionFactory)
    {
      if(distanceFactoryAlreadySet(inName)) {
	return false;
      }
      attMapDistanceFunctionFactory[inName] = inDistanceFunctionFactory;
      return true;
    }

    CkwsDistanceShPtr Server::createDistanceFunction(std::string inName,
							   bool inOriented)
    {
      CkwsDistanceShPtr result;

      if (distanceFactoryAlreadySet(inName)) {
	result = attMapDistanceFunctionFactory[inName]->makeDistance(inOriented);
      }
      return result;
    }


    /*
      DIFFUSION NODE PICKER FACTORIES
    */

    void Server::initMapDiffusionNodePickerFactory()
    {
      attMapDiffusionNodePickerFactory["basic"] = new CkwsPlusBasicDiffusionNodePickerFactory;
      attMapDiffusionNodePickerFactory["smallestTree"] =
	new CkwsPlusSmallestTreeDiffusionNodePickerFactory;
    }

    void Server::destroyDiffusionNodePickerFactory()
    {
      std::map<std::string, CkwsPlusDiffusionNodePickerFactory*>::iterator start
	= attMapDiffusionNodePickerFactory.begin();
      std::map<std::string, CkwsPlusDiffusionNodePickerFactory*>::iterator end
	= attMapDiffusionNodePickerFactory.end();

      for (std::map<std::string, CkwsPlusDiffusionNodePickerFactory*>::iterator it=start;
	   it != end; it++) {
	CkwsPlusDiffusionNodePickerFactory* factory = it->second;
	hppDout (info, " deleting diffusion node picker factory" << it->first);
	delete factory;
      }
    }

    bool Server::diffusionNodePickerFactoryAlreadySet(std::string inName)
    {
      if (attMapDiffusionNodePickerFactory.count(inName) == 1) {
	return true;
      }
      return false;
    }


    bool Server::addDiffusionNodePickerFactory(std::string inName,
						     CkwsPlusDiffusionNodePickerFactory* inDiffusionNodePickerFactory)
    {
      if(diffusionNodePickerFactoryAlreadySet(inName)) {
	return false;
      }
      attMapDiffusionNodePickerFactory[inName] = inDiffusionNodePickerFactory;
      return true;
    }

    CkwsDiffusionNodePickerShPtr Server::createDiffusionNodePicker(std::string inName)
    {
      CkwsDiffusionNodePickerShPtr result;

      if (diffusionNodePickerFactoryAlreadySet(inName)) {
	result = attMapDiffusionNodePickerFactory[inName]->makeDiffusionNodePicker();
      }
      return result;
    }

    /*
      DIFFUSION SHOOTER FACTORIES
    */

    void Server::initMapDiffusionShooterFactory()
    {
      attMapDiffusionShooterFactory["config space"] = new CkwsPlusShooterConfigSpaceFactory;
      attMapDiffusionShooterFactory["roadmap box"] = new CkwsPlusShooterRoadmapBoxFactory;
      attMapDiffusionShooterFactory["roadmap node"] = new CkwsPlusShooterRoadmapNodesFactory;
    }

    void Server::destroyDiffusionShooterFactory()
    {
      std::map<std::string, CkwsPlusDiffusionShooterFactory*>::iterator start
	= attMapDiffusionShooterFactory.begin();
      std::map<std::string, CkwsPlusDiffusionShooterFactory*>::iterator end
	= attMapDiffusionShooterFactory.end();

      for (std::map<std::string, CkwsPlusDiffusionShooterFactory*>::iterator it=start;
	   it != end; it++) {
	CkwsPlusDiffusionShooterFactory* factory = it->second;
	hppDout (info, " deleting diffusion shooter factory" << it->first);
	delete factory;
      }
    }

    bool Server::diffusionShooterFactoryAlreadySet(std::string inName)
    {
      if (attMapDiffusionShooterFactory.count(inName) == 1) {
	return true;
      }
      return false;
    }


    bool Server::addDiffusionShooterFactory(std::string inName,
						  CkwsPlusDiffusionShooterFactory* inDiffusionShooterFactory)
    {
      if(diffusionShooterFactoryAlreadySet(inName)) {
	return false;
      }
      attMapDiffusionShooterFactory[inName] = inDiffusionShooterFactory;
      return true;
    }

    CkwsDiffusionShooterShPtr Server::createDiffusionShooter(std::string inName)
    {
      CkwsDiffusionShooterShPtr result;

      if (diffusionShooterFactoryAlreadySet(inName)) {
	result = attMapDiffusionShooterFactory[inName]->makeDiffusionShooter();
      }
      return result;
    }



    /*
      CORBA SERVER INITIALIZATION
    */

    ktStatus Server::initORBandServers(int argc, const char* argv[], bool inMultiThread)
    {
      Object_var obj;
      PortableServer::ThreadPolicy_var threadPolicy;
      PortableServer::POA_var rootPoa;

      /*
	 Fine granularity in exception handling
      */

      /*
	ORB init
      */
      try {
	attPrivate->orb_ = ORB_init (argc, const_cast<char **> (argv)); //FIXME: handle this properly.
	if (is_nil(attPrivate->orb_)) {
	  hppDout (error, "failed to initialize ORB");
	  return KD_ERROR;
	}
      }
      HPPCI_CATCH("failed to initialize ORB", KD_ERROR)

	/*
	  ORB init
	*/

	try {
	  obj = attPrivate->orb_->resolve_initial_references("RootPOA");
	}
      HPPCI_CATCH("failed to resolve initial references", KD_ERROR)

	/*
	  Create thread policy
	*/

	try {
	  //
	  // Make the CORBA object single-threaded to avoid GUI krash
	  //
	  // Create a sigle threaded policy object
	  rootPoa = PortableServer::POA::_narrow(obj);

	  if (inMultiThread) {
	    threadPolicy = rootPoa->create_thread_policy(PortableServer::ORB_CTRL_MODEL);
	  }
	  else {
	    threadPolicy = rootPoa->create_thread_policy(PortableServer::MAIN_THREAD_MODEL);
	  }
	}
      HPPCI_CATCH("failed to create thread policy", KD_ERROR)

	/*
	  Duplicate thread policy
	*/

	try {
	  PolicyList policyList;
	  policyList.length(1);
	  policyList[0] = PortableServer::ThreadPolicy::_duplicate(threadPolicy);

	  attPrivate->poa_ = rootPoa->create_POA("child", PortableServer::POAManager::_nil(),
						policyList);

	}
      HPPCI_CATCH("failed to duplicate thread policy", KD_ERROR)

	/*
	  Destroy thread policy
	*/

	try {
	  // Destroy policy object
	  threadPolicy->destroy();

	}
      HPPCI_CATCH("failed to destroy thread policy", KD_ERROR);

      return attPrivate->createAndActivateServers(this);
    }

    int Server::startCorbaServer()
    {
      try {
	// Obtain a reference to objects, and register them in
	// the naming service.
	Object_var robotObj = attPrivate->robotServant_->_this();
	Object_var obstacleObj = attPrivate->obstacleServant_->_this();
	Object_var problemObj = attPrivate->problemServant_->_this();

	if (!attPrivate->createHppContext()) {
	  return KD_ERROR;
	}
	// Bind robotObj with name Robot to the hppContext:
	CosNaming::Name objectName;
	objectName.length(1);
	objectName[0].id   = (const char*) "robots";   // string copied
	objectName[0].kind = (const char*) "servant"; // string copied

	if(!attPrivate->bindObjectToName(robotObj, objectName)) {
	  return KD_ERROR;
	}
	attPrivate->robotServant_->_remove_ref();

	// Bind obstacleObj with name Obstacle to the hppContext:
	objectName.length(1);
	objectName[0].id   = (const char*) "obstacles";   // string copied
	objectName[0].kind = (const char*) "servant"; // string copied

	if(!attPrivate->bindObjectToName(obstacleObj, objectName)) {
	  return KD_ERROR;
	}
	attPrivate->obstacleServant_->_remove_ref();

	// Bind problemObj with name Problem to the hppContext:
	objectName.length(1);
	objectName[0].id   = (const char*) "problems";   // string copied
	objectName[0].kind = (const char*) "servant"; // string copied

	if(!attPrivate->bindObjectToName(problemObj, objectName)) {
	  return KD_ERROR;
	}
	attPrivate->problemServant_->_remove_ref();

	PortableServer::POAManager_var pman = attPrivate->poa_->the_POAManager();
	pman->activate();
      }
      HPPCI_CATCH("failed to start CORBA server", KD_ERROR);
      return KD_OK;
    }

    const core::Planner* Server::planner() const
    {
      return hppPlanner;
    }

    core::Planner* Server::planner()
    {
      return hppPlanner;
    }




    /// \brief If CORBA requests are pending, process them
    int Server::processRequest (bool loop)
    {
      if (loop)
	{
	  hppDout (info, "start processing CORBA requests for ever.");
	  attPrivate->orb_->run();
	}
      else
	{
	  if (attPrivate->orb_->work_pending())
	    attPrivate->orb_->perform_work();
	}
      return 0;
    }

  } // end of namespace corbaServer.
} // end of namespace hpp.
