/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#include <iostream>
#include "hppCorbaServer/hppciServerPrivate.h"

CORBA::Boolean ChppciServerPrivate::createHppContext()
{
  CosNaming::NamingContext_var rootContext;

  try {
    // Obtain a reference to the root context of the Name service:
    CORBA::Object_var localObj;
    localObj = orb->resolve_initial_references("NameService");

    // Narrow the reference returned.
    rootContext = CosNaming::NamingContext::_narrow(localObj);
    if( CORBA::is_nil(rootContext) ) {
      std::cerr << "Failed to narrow the root naming context." << std::endl;
      return 0;
    }
  }
  catch(CORBA::ORB::InvalidName& ex) {
    // This should not happen!
    std::cerr << "Service required is invalid [does not exist]." << std::endl;
    return 0;
  }

  try {
    // Bind a context called "hpp" to the root context:

    CosNaming::Name contextName;
    contextName.length(1);
    contextName[0].id   = (const char*) "hpp";       // string copied
    contextName[0].kind = (const char*) "my_context"; // string copied
    // Note on kind: The kind field is used to indicate the type
    // of the object. This is to avoid conventions such as that used
    // by files (name.type -- e.g. hpp.ps = postscript etc.)

    try {
      // Bind the context to root.
      hppContext = rootContext->bind_new_context(contextName);
    }
    catch(CosNaming::NamingContext::AlreadyBound& ex) {
      // If the context already exists, this exception will be raised.
      // In this case, just resolve the name and assign hppContext
      // to the object returned:
      CORBA::Object_var localObj;
      localObj = rootContext->resolve(contextName);
      hppContext = CosNaming::NamingContext::_narrow(localObj);
      if( CORBA::is_nil(hppContext) ) {
        std::cerr << "Failed to narrow naming context." << std::endl;
        return 0;
      }
    }
  }
  catch(CORBA::COMM_FAILURE& ex) {
    std::cerr << "Caught system exception COMM_FAILURE -- unable to contact the "
         << "naming service." << std::endl;
    return 0;
  }
  catch(CORBA::SystemException&) {
    std::cerr << "Caught a CORBA::SystemException while using the naming service."
	 << std::endl;
    return 0;
  }

  return 1;
}

CORBA::Boolean
ChppciServerPrivate::bindObjectToName(CORBA::Object_ptr objref,
			       CosNaming::Name objectName)
{
  try {
    try {
      hppContext->bind(objectName, objref);
    }
    catch(CosNaming::NamingContext::AlreadyBound& ex) {
      hppContext->rebind(objectName, objref);
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
  catch(CORBA::COMM_FAILURE& ex) {
    std::cerr << "Caught system exception COMM_FAILURE -- unable to contact the "
         << "naming service." << std::endl;
    return 0;
  }
  catch(CORBA::SystemException&) {
    std::cerr << "Caught a CORBA::SystemException while using the naming service."
	 << std::endl;
    return 0;
  }

  return 1;
}
