#ifndef CHPPCISERVERPRIVATE_H
#define CHPPCISERVERPRIVATE_H

#include "hppCorbaServer/hppciRobot.h"
#include "hppCorbaServer/hppciObstacle.h"
#include "hppCorbaServer/hppciProblem.h"

class ChppciServerPrivate {
public:
  ~ChppciServerPrivate();

  /**
     \brief Create and activate the Corba servers
  */
  void createAndActivateServers(ChppciServer* inHppciServer);

private:
  CORBA::ORB_var orb;
  PortableServer::POA_var poa;

  /// \brief Implementation of object ChppciRobot
  ChppciRobot_impl *robotServant;
  /// \brief Implementation of object ChppciObstacle
  ChppciObstacle_impl *obstacleServant;
  /// \brief Implementation of object ChppciProblem.
  ChppciProblem_impl *problemServant;

  /** 
      \brief It seems that we need to store this object to deactivate the server.
  */
  //PortableServer::ObjectId_var robotServantid;
  PortableServer::ObjectId* robotServantid;
  /** 
      \brief It seems that we need to store this object to deactivate the server.
  */
  //PortableServer::ObjectId_var obstacleServantid;
  PortableServer::ObjectId* obstacleServantid;
  /** 
      \brief It seems that we need to store this object to deactivate the server.
  */
  //PortableServer::ObjectId_var problemServantid;
  PortableServer::ObjectId* problemServantid;

  /// \brief Corba context.
  CosNaming::NamingContext_var hppContext;
  // methods
  /// \brief Create context.
  CORBA::Boolean createHppContext();
  /// \brief Store objects in Corba name service.
  CORBA::Boolean bindObjectToName(CORBA::Object_ptr objref,
				  CosNaming::Name objectName);


  /**
     \brief Deactivate and destroy servers
     Destroying active servers raises a Corba exception.
  */
  void deactivateAndDestroyServers();

  friend class ChppciServer;
};

#endif
