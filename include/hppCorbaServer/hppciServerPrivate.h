#ifndef CHPPCISERVERPRIVATE_H
#define CHPPCISERVERPRIVATE_H

#include "hppCorbaServer/hppciRobot.h"
#include "hppCorbaServer/hppciObstacle.h"
#include "hppCorbaServer/hppciProblem.h"

class ChppciServerPrivate {
public:
  ~ChppciServerPrivate() {
    if (robotServant) delete robotServant;
    if (obstacleServant) delete obstacleServant;
    if (problemServant) delete problemServant;
  };
private:
  CORBA::ORB_var orb;
  PortableServer::POA_var poa;

  /// \brief Implementation of object ChppciRobot
  ChppciRobot_impl *robotServant;
  /// \brief Implementation of object ChppciObstacle
  ChppciObstacle_impl *obstacleServant;
  /// \brief Implementation of object ChppciProblem.
  ChppciProblem_impl *problemServant;

  /// \brief Corba context.
  CosNaming::NamingContext_var hppContext;
  // methods
  /// \brief Create context.
  CORBA::Boolean createHppContext();
  /// \brief Store objects in Corba name service.
  CORBA::Boolean bindObjectToName(CORBA::Object_ptr objref,
				  CosNaming::Name objectName);


  friend class ChppciServer;
};

#endif
