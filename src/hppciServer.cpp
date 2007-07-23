/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#include <iostream>
#include "hppciServer.h"
#include "hppciServerPrivate.h"

ChppciServer* ChppciServer::s_hppciServer;

ChppciServer::ChppciServer(ChppPlanner *inHppPlanner)
{
  hppPlanner = inHppPlanner;
  attPrivate = new ChppciServerPrivate;
  s_hppciServer = this;
}

/// \brief Shutdown CORBA server
ChppciServer::~ChppciServer()
{
  attPrivate->orb->shutdown(0);
  delete hppPlanner;
  delete attPrivate;
  s_hppciServer = NULL;
}

ChppciServer* ChppciServer::getInstance()
{
  return s_hppciServer;
}

int ChppciServer::startCorbaServer(int argc, char *argv[])
{
  try {
    attPrivate->orb = CORBA::ORB_init(argc, argv);
    CORBA::Object_var obj = attPrivate->orb->resolve_initial_references("RootPOA");
    attPrivate->poa = PortableServer::POA::_narrow(obj);

    attPrivate->robotServant = new ChppciRobot_impl(hppPlanner);
    attPrivate->obstacleServant = new ChppciObstacle_impl(hppPlanner);
    attPrivate->problemServant = new ChppciProblem_impl();

    PortableServer::ObjectId_var robotServantid = 
      attPrivate->poa->activate_object(attPrivate->robotServant);
    PortableServer::ObjectId_var obstacleServantid = 
      attPrivate->poa->activate_object(attPrivate->obstacleServant);
    PortableServer::ObjectId_var problemServantid = 
      attPrivate->poa->activate_object(attPrivate->problemServant);

    // Obtain a reference to objects, and register them in
    // the naming service.
    CORBA::Object_var robotObj = attPrivate->robotServant->_this();
    CORBA::Object_var obstacleObj = attPrivate->obstacleServant->_this();
    CORBA::Object_var problemObj = attPrivate->problemServant->_this();

    if (!attPrivate->createHppContext()) {
      return KD_ERROR;
    }
    // Bind robotObj with name Robot to the hppContext:
    CosNaming::Name objectName;
    objectName.length(1);
    objectName[0].id   = (const char*) "Robot";   // string copied
    objectName[0].kind = (const char*) "Object"; // string copied

    if(!attPrivate->bindObjectToName(robotObj, objectName)) {
      return KD_ERROR;
    }
    attPrivate->robotServant->_remove_ref();

    // Bind obstacleObj with name Obstacle to the hppContext:
    objectName.length(1);
    objectName[0].id   = (const char*) "Obstacle";   // string copied
    objectName[0].kind = (const char*) "Object"; // string copied

    if(!attPrivate->bindObjectToName(obstacleObj, objectName)) {
      return KD_ERROR;
    }
    attPrivate->obstacleServant->_remove_ref();

    // Bind problemObj with name Problem to the hppContext:
    objectName.length(1);
    objectName[0].id   = (const char*) "Problem";   // string copied
    objectName[0].kind = (const char*) "Object"; // string copied

    if(!attPrivate->bindObjectToName(problemObj, objectName)) {
      return KD_ERROR;
    }
    attPrivate->problemServant->_remove_ref();

    PortableServer::POAManager_var pman = attPrivate->poa->the_POAManager();
    pman->activate();
  }
  catch(CORBA::SystemException&) {
    cerr << "Caught CORBA::SystemException." << endl;
    return KD_ERROR;
  }
  catch(CORBA::Exception&) {
    cerr << "Caught CORBA::Exception." << endl;
    return KD_ERROR;
  }
  catch(omniORB::fatalException& fe) {
    cerr << "Caught omniORB::fatalException:" << endl;
    cerr << "  file: " << fe.file() << endl;
    cerr << "  line: " << fe.line() << endl;
    cerr << "  mesg: " << fe.errmsg() << endl;
    return KD_ERROR;
  }
  catch(...) {
    cerr << "Caught unknown exception." << endl;
    return KD_ERROR;
  }
  return KD_OK;
}

ChppPlanner *ChppciServer::getHppPlanner()
{
  return hppPlanner;
}



/// \brief If CORBA requests are pending, process them
int ChppciServer::processRequest(bool loop)
{
  if (loop) {
    // Enter in the Corba control loop. Never return.
    cout << "ChppciServer::processRequest: attPrivate->orb->run();" << endl;
    attPrivate->orb->run();
  } else {
    if (attPrivate->orb->work_pending()) {
      attPrivate->orb->perform_work();
    }
  }
    return 0;
}

