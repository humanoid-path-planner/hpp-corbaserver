// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <iostream>

#include <hpp/util/debug.hh>

#include "obstacle.impl.hh"
#include "problem.impl.hh"
#include "robot.impl.hh"
#include "precomputation.impl.hh"
#include "server-private.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      using CORBA::Exception;
      using CORBA::ORB_init;
      using CORBA::Object_ptr;
      using CORBA::Object_var;
      using CORBA::PolicyList;
      using CORBA::SystemException;
      using CORBA::COMM_FAILURE;
      using omniORB::fatalException;

      typedef CORBA::ORB::InvalidName InvalidName;

      Server::~Server ()
      {
	delete robotServantid_;
	delete problemServantid_;
	delete obstacleServantid_;
	delete precomputationServantid_;
      }

      void
      Server::createAndActivateServers (corbaServer::Server* inServer)
      {
	robotServant_ = new Robot (inServer);
	obstacleServant_ = new Obstacle (inServer);
	problemServant_ = new Problem (inServer);
	precomputationServant_ = new Precomputation (inServer);

	robotServantid_ = poa_->activate_object(robotServant_);
	obstacleServantid_ = poa_->activate_object(obstacleServant_);
	problemServantid_ = poa_->activate_object(problemServant_);
	precomputationServantid_ = poa_->activate_object(precomputationServant_);
      }

      void Server::deactivateAndDestroyServers()
      {
	if (robotServant_) {
	  poa_->deactivate_object(*robotServantid_);
	  delete robotServant_;
	}
	if (obstacleServant_) {
	  poa_->deactivate_object(*obstacleServantid_);
	  delete obstacleServant_;
	}
	if (problemServant_) {
	  poa_->deactivate_object(*problemServantid_);
	  delete problemServant_;
	}
	if (precomputationServant_) {
	  poa_->deactivate_object(*precomputationServantid_);
	  delete precomputationServant_;
	}
      }


      void Server::createHppContext ()
      {
	CosNaming::NamingContext_var rootContext;
	Object_var localObj;
	CosNaming::Name contextName;

	// Obtain a reference to the root context of the Name service:
	localObj = orb_->resolve_initial_references("NameService");
	try {
	  // Narrow the reference returned.
	  rootContext = CosNaming::NamingContext::_narrow(localObj);
	  if( is_nil(rootContext) ) {
	    std::string msg ("Failed to narrow the root naming context.");
	    hppDout (error, msg.c_str ());
	    throw std::runtime_error (msg.c_str ());
	  }
	}
	catch(InvalidName& ex) {
	  // This should not happen!
	  std::string msg ("Service required is invalid [does not exist].");
	  hppDout (error, msg.c_str ());
	  throw std::runtime_error (msg.c_str ());
	}
	// Bind a context called "hpp" to the root context:
	contextName.length(1);
	contextName[0].id   = (const char*) "hpp";       // string copied
	contextName[0].kind = (const char*) "corbaserver"; // string copied
	// Note on kind: The kind field is used to indicate the type
	// of the object. This is to avoid conventions such as that used
	// by files (name.type -- e.g. hpp.ps = postscript etc.)

	try {
	  // Bind the context to root.
	  hppContext_ = rootContext->bind_new_context(contextName);
	}
	catch(CosNaming::NamingContext::AlreadyBound& ex) {
	  // If the context already exists, this exception will be raised.
	  // In this case, just resolve the name and assign hppContext
	  // to the object returned:
	  Object_var localObj;
	  localObj = rootContext->resolve(contextName);
	  hppContext_ = CosNaming::NamingContext::_narrow(localObj);
	  if( is_nil(hppContext_) ) {
	    std::string msg ("Failed to narrow naming context.");
	    hppDout (error, msg.c_str ());
	    throw std::runtime_error (msg.c_str ());
	  }
	}
      }

      void Server::bindObjectToName(Object_ptr objref,
				    CosNaming::Name objectName)
      {
	try {
	  hppContext_->bind(objectName, objref);
	}
	catch(CosNaming::NamingContext::AlreadyBound& ex) {
	  hppContext_->rebind(objectName, objref);
	}
	// Note: Using rebind() will overwrite any Object previously bound
	//       to /hpp/RobotConfig with localObj.
	//       Alternatively, bind() can be used, which will raise a
	//       CosNaming::NamingContext::AlreadyBound exception if the name
	//       supplied is already bound to an object.

	// Amendment: When using OrbixNames, it is necessary to first
	// try bind and then rebind, as rebind on it's own will throw
	// a NotFoundexception if the Name has not already been
	// bound. [This is incorrect behaviour - it should just bind].
      }
    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.
