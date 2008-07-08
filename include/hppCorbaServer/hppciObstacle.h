/*
  Copyright 2006 LAAS-CNRS

  Author: Florent Lamiraux

*/
#ifndef HPPCI_OBSTACLE_H
#define HPPCI_OBSTACLE_H

#include <map>
#include <string>

#include "KineoKCDModel/kppKCDPolyhedron.h"

#include "hppCore/hppPlanner.h"

#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION
#include "hppciObstacleServer.hh"

class ChppciServer;

/**
 * \brief Implementation of corba interface ChppciObstacle.
 */
class ChppciObstacle_impl : public virtual POA_hppCorbaServer::ChppciObstacle {
public:
  /// \brief store pointer to ChppPlanner object.
  ChppciObstacle_impl(ChppciServer *inHppciServer);
  /// \brief Comment in interface hppCorbaServer::ChppciObstacle::setObstacles
  CORBA::Short setObstacles(const char* inListName);
  /// \brief Comment in interface hppCorbaServer::ChppciObstacle::addObstacle
  CORBA::Short addObstacle(const char* inPolyhedronName);

  /// \brief add an obstacle at the given position.
 virtual CORBA::Short addObstacleConfig(const char* inPolyName, 
					const hppCorbaServer::Configuration& cfg)
   throw(CORBA::SystemException);
 virtual CORBA::Short moveObstacleConfig(const char* inPolyName, 
					 const hppCorbaServer::Configuration& cfg)
   throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciObstacle::createCollisionList.
  virtual CORBA::Short createCollisionList(const char* inListName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciObstacle::addPolyToCollList.
  virtual CORBA::Short 
    addPolyToCollList(const char* inListName, const char* inPolyhedronName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciObstacle::createPolyhedron.
  virtual CORBA::Short createPolyhedron(const char* inPolyhedronName)
    throw(CORBA::SystemException);
  /// \brief Comment in ChppciObstacle::createBox.
  virtual CORBA::Short createBox(const char* inBoxName, CORBA::Double x, 
	     CORBA::Double y, CORBA::Double z)
    throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciObstacle::addPoint.
  virtual CORBA::Short 
    addPoint(const char* inPolyhedronName, CORBA::Double x, 
	     CORBA::Double y, CORBA::Double z) throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciObstacle::addTriangle.
  virtual CORBA::Short 
    addTriangle(const char* inPolyhedronName, CORBA::ULong pt1, CORBA::ULong pt2, CORBA::ULong pt3)
    throw(CORBA::SystemException);

#if WITH_OPENHRP
  /// \brief Comment in interface hppCorbaServer::ChppciObstacle::loadModelLoaderObstacle.
  virtual CORBA::Short 
    loadModelLoaderObstacle(const char* inPolyName, const char* inFilename,
			    const char* inOpenHrpPrefix)
    throw(CORBA::SystemException);
#endif

  /// \brief Comment in interface hppCorbaServer::ChppciObstacle::setVisible.
  virtual CORBA::Short 
    setVisible(const char* inPolyname, CORBA::Boolean inVisible)
    throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciObstacle::setTransparent.
  virtual CORBA::Short 
    setTransparent(const char* inPolyname, CORBA::Boolean inTransparent)
    throw(CORBA::SystemException);

private:
  /// \brief map of collision lists in construction.
  std::map<std::string, std::vector<CkcdObjectShPtr> > collisionListMap;

  /// \brief map of polyhedra in construction.
  std::map<std::string, CkppKCDPolyhedronShPtr> polyhedronMap;

  /// \brief Pointer to hppPlanner object of hppciServer.
  ChppPlanner *attHppPlanner;

  /// \brief Pointer to the ChppciServer owning this object
  ChppciServer* attHppciServer;
};

#endif
