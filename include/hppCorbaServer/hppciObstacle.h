/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#ifndef HPPCI_OBSTACLE_H
#define HPPCI_OBSTACLE_H

#include <map>
#include <string>

#include "KineoKCDModel/kppKCDPolyhedron.h"

#include "hppCore/hppPlanner.h"
#include "hppciObstacleServer.hh"

class ChppciServer;

/**
 * \brief Implementation of corba interface ChppciObstacle.
 */
class ChppciObstacle_impl : public virtual POA_ChppciObstacle {
public:
  /// \brief store pointer to ChppPlanner object.
  ChppciObstacle_impl(ChppciServer *inHppciServer);
  /// \brief Comment in interface ChppciObstacle::setObstacles
  CORBA::Short setObstacles(const char* inListName);
  /// \brief Comment in interface ChppciObstacle::addObstacle
  CORBA::Short addObstacle(const char* inPolyhedronName);

  /// \brief add an obstacle at the given position.
 virtual CORBA::Short addObstacleConfig(const char* inPolyName, 
					const Configuration& cfg)
   throw(CORBA::SystemException);
 virtual CORBA::Short moveObstacleConfig(const char* inPolyName, 
					 const Configuration& cfg)
   throw(CORBA::SystemException);

  /// \brief Comment in interface ChppciObstacle::createCollisionList.
  virtual CORBA::Short createCollisionList(const char* inListName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciObstacle::addPolyToCollList.
  virtual CORBA::Short 
    addPolyToCollList(const char* inListName, const char* inPolyhedronName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciObstacle::createPolyhedron.
  virtual CORBA::Short createPolyhedron(const char* inPolyhedronName)
    throw(CORBA::SystemException);
  /// \brief Comment in ChppciObstacle::createBox.
  virtual CORBA::Short createBox(const char* inBoxName, CORBA::Double x, 
	     CORBA::Double y, CORBA::Double z)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciObstacle::addPoint.
  virtual CORBA::Short 
    addPoint(const char* inPolyhedronName, CORBA::Double x, 
	     CORBA::Double y, CORBA::Double z) throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciObstacle::addTriangle.
  virtual CORBA::Short 
    addTriangle(const char* inPolyhedronName, long pt1, long pt2, long pt3)
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
