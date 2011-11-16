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
#include "server-private.hh"

//FIXME: remove me.
#define HPPCI_CATCH(msg, ret)						\
  catch(CORBA::SystemException&) {					\
    hppDout (error, "hppCorbaChppciServerPrivate: CORBA::SystemException: " << msg);	\
    return ret;								\
  }									\
  catch(CORBA::Exception&) {						\
    hppDout (error, "hppCorbaChppciServerPrivate: CORBA::Exception: " << msg);	\
    return ret;								\
  }									\
  catch(omniORB::fatalException& fe) {					\
    hppDout (error, "hppCorbaChppciServerPrivate: CORBA::fatalException: " << msg);	\
    return ret;								\
  }									\
  catch(...) {								\
    hppDout (error, "hppCorbaChppciServerPrivate: unknown exception: " << msg);	\
    return ret;								\
  }

namespace hpp
{
  namespace corbaChppciServerPrivate
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

      ChppciServerPrivate::~ChppciServerPrivate ()
      {
	delete robotServantid_;
	delete problemServantid_;
	delete obstacleServantid_;
      }

      ktStatus
      ChppciServerPrivate::createAndActivateChppciServerPrivates (corbaChppciServerPrivate::ChppciServerPrivate* inChppciServerPrivate)
      {
	try {
	  robotServant_ = new Robot (inChppciServerPrivate);
	}
	HPPCI_CATCH("failed to create implementation of hpp::Robot", KD_ERROR)

	  try {
	    obstacleServant_ = new Obstacle (inChppciServerPrivate);
	  }
	HPPCI_CATCH("failed to create implementation of hpp::Obstacle", KD_ERROR)

	  try {
	    problemServant_ = new Problem (inChppciServerPrivate);
	  }
	HPPCI_CATCH("failed to create implementation of hpp::Problem", KD_ERROR)

	  try {

	    robotServantid_ = poa_->activate_object(robotServant_);
	  }
	HPPCI_CATCH("failed to activate implementation of hpp::Robot", KD_ERROR)

	  try {
	    obstacleServantid_ = poa_->activate_object(obstacleServant_);
	  }
	HPPCI_CATCH("failed to activate implementation of hpp::Obstacle", KD_ERROR)

	  try {
	    problemServantid_ = poa_->activate_object(problemServant_);
	  }
	HPPCI_CATCH("failed to activate implementation of hpp::Problem", KD_ERROR)

	  return KD_OK;
      }

      void ChppciServerPrivate::deactivateAndDestroyChppciServerPrivates()
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
      }


      bool ChppciServerPrivate::createHppContext ()
      {
	CosNaming::NamingContext_var rootContext;
	Object_var localObj;
	CosNaming::Name contextName;

	try {
	  // Obtain a reference to the root context of the Name service:
	  localObj = orb_->resolve_initial_references("NameService");
	}
	HPPCI_CATCH("failed to get the name service", false);

	try {
	  // Narrow the reference returned.
	  rootContext = CosNaming::NamingContext::_narrow(localObj);
	  if( is_nil(rootContext) ) {
	    hppDout (error, "Failed to narrow the root naming context.");
	    return false;
	  }
	}
	catch(InvalidName& ex) {
	  // This should not happen!
	  hppDout (error, "Service required is invalid [does not exist].");
	  return false;
	}
	HPPCI_CATCH("failed to narrow the root naming context.", false);

	try {
	  // Bind a context called "hpp" to the root context:

	  contextName.length(1);
	  contextName[0].id   = (const char*) "hpp";       // string copied
	  contextName[0].kind = (const char*) "plannerContext"; // string copied
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
	      hppDout (error, "Failed to narrow naming context.");
	      return false;
	    }
	  }
	}
	catch(COMM_FAILURE& ex) {
	  hppDout (error, "Caught system exception COMM_FAILURE -- unable to contact the "
		   << "naming service.");
	  return false;
	}
	catch(SystemException&) {
	  hppDout (error, "Caught a SystemException while creating the context.");
	  return false;
	}

	return true;
      }

      bool ChppciServerPrivate::bindObjectToName(Object_ptr objref,
						 CosNaming::Name objectName)
      {
	try {
	  try {
	    hppContext_->bind(objectName, objref);
	  }
	  catch(CosNaming::NamingContext::AlreadyBound& ex)
	    {
	      hppContext_->rebind(objectName, objref);
	    }
	  // Note: Using rebind() will overwrite any Object previously bound
	  //       to /hpp/RobotConfig with localObj.
	  //       Alternatively, bind() can be used, which will raise a
	  //       CosNaming::NamingContext::AlreadyBound exception if the name
	  //       supplied is already bound to an object.

	  // Amendment: When using OrbixNames, it is necessary to first try bind
	  // and then rebind, as rebind on it's own will throw a NotFoundexception if
	  // the Name has not already been bound. [This is incorrect behaviour -
	  // it should just bind].
	}
	catch(COMM_FAILURE& ex) {
	  hppDout (error, "Caught system exception COMM_FAILURE -- unable to contact the "
		   << "naming service.");
	  return false;
	}
	catch(SystemException&) {
	  hppDout (error, "Caught a SystemException while binding object to name service.");
	  return false;
	}

	return true;
      }

    } // end of namespace impl.
  } // end of namespace corbaChppciServerPrivate.
} // end of namespace hpp.
