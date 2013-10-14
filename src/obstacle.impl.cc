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
#include <boost/foreach.hpp>
#include <KineoKCDModel/kppKCDBox.h>

#include <hpp/util/debug.hh>

#include "obstacle.impl.hh"
#include "tools.hh"

#include "hpp/corbaserver/server.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      Obstacle::Obstacle (corbaServer::Server* server)
	: server_ (server),
	  planner_ (server->planner ())
      {}

      Short
      Obstacle::setObstacles (const char* listName)
      {
	// Check that collision list exists.
	if (collisionListMap.count(listName) != 1)
	  {
	    hppDout (error, "collision list "
		     << listName << " does not exist");
	    return -1;
	  }
	planner_->obstacleList (collisionListMap[listName]);
	return 0;
      }

      Short
      Obstacle::addObstacle(const char* polyhedronName)
      {
	// Check that polyhedron exists.
	if (polyhedronMap.count(polyhedronName) != 1)
	  {
	    hppDout (error, "polyhedron "
		     << polyhedronName << " does not exist");
	    return -1;
	  }
	CkppKCDPolyhedronShPtr hppPolyhedron = polyhedronMap[polyhedronName];

	// Build collision entity for KCD.
	hppPolyhedron->makeCollisionEntity(CkcdObject::IMMEDIATE_BUILD);
	planner_->addObstacle(hppPolyhedron);
	return 0;
      }

      Short
      Obstacle::addObstacleConfig
      (const char* polyhedronName, const hpp::Configuration& cfg)
	throw(SystemException)
      {
	// Check that polyhedron exists.
	if (polyhedronMap.count (polyhedronName) != 1)
	  {
	    hppDout(error, "polyhedron "
		    << polyhedronName << " does not exist");
	    return -1;
	  }
	CkppKCDPolyhedronShPtr polyhedron = polyhedronMap[polyhedronName];

	// Build collision entity for KCD.
	polyhedron->makeCollisionEntity(CkcdObject::IMMEDIATE_BUILD);

	CkitMat4 mat;
	ConfigurationToCkitMat4(cfg, mat);

	polyhedron->setAbsolutePosition(mat);
	planner_->addObstacle(polyhedron);
	return 0;
      }

      Short
      Obstacle::moveObstacleConfig
      (const char* solidComponentName, const hpp::Configuration& cfg)
	throw(SystemException)
      {
	std::vector <CkcdObjectShPtr> obstacleList (planner_->obstacleList ());
	hppDout (info,
		 "Number of obstacles: " << planner_->obstacleList ().size ());
	for (std::vector <CkcdObjectShPtr>::iterator it = obstacleList.begin ();
	     it != obstacleList.end (); it++) {
	  CkcdObjectShPtr object = *it;
	  CkppSolidComponentShPtr solidComponent
	    = KIT_DYNAMIC_PTR_CAST (CkppSolidComponent, object);
	  if (solidComponent) {
	    hppDout (info, "obstacle Name: " << solidComponent->name ());
	  }
	  if (solidComponent && solidComponentName == solidComponent->name ())
	    {
	      hppDout (info, "found ``"
		       << solidComponentName << "'' in the tree.");
	      CkitMat4 mat;
	      ConfigurationToCkitMat4 (cfg, mat);
	      solidComponent->setAbsolutePosition (mat);
	      return 0;
	    }
	}
	hppDout (error, "failed to find ``"
		 << solidComponentName << "'' in the tree.");
	return -1;
      }

      Short
      Obstacle::createCollisionList
      (const char* listName) throw (SystemException)
      {
	if (collisionListMap.count (listName) != 0)
	  {
	    hppDout (error, "collision list " << listName << " already exists");
	    return -1;
	  }
	collisionListMap[listName] = std::vector<CkcdObjectShPtr> ();
	return 0;
      }

      Short
      Obstacle::addPolyToCollList
      (const char* listName, const char* polyhedronName)
	throw (SystemException)
      {
	if (collisionListMap.count (listName) != 1)
	  {
	    hppDout (error, "collision list "
		     << listName << " does not exist");
	    return -1;
	  }

	// Check that polyhedron exists.
	if (polyhedronMap.count (polyhedronName) != 1)
	  {
	    hppDout (error, "polyhedron "
		     << polyhedronName << " does not exist");
	    return -1;
	  }

	CkppKCDPolyhedronShPtr polyhedron = polyhedronMap[polyhedronName];
	polyhedron->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);
	collisionListMap[listName].push_back (polyhedron);
	return 0;
      }

      Short
      Obstacle::createPolyhedron
      (const char* polyhedronName) throw (SystemException)
      {
	// Check that polyhedron does not already exist.
	if (polyhedronMap.count(polyhedronName) != 0)
	  {
	    hppDout(error, "polyhedron "
		    << polyhedronName << " already exists.");
	    return -1;
	  }

	CkppKCDPolyhedronShPtr polyhedron =
	  CkppKCDPolyhedron::create (polyhedronName);
	if (!polyhedron)
	  {
	    hppDout (error, "failed to create polyhedron " << polyhedronName);
	    return -1;
	  }
	polyhedronMap[polyhedronName] = polyhedron;
	return 0;
      }

      Short
      Obstacle::createBox
      (const char* boxName, Double x, Double y, Double z)
	throw (SystemException)
      {
	// Check that polyhedron does not already exist.
	if (polyhedronMap.count(boxName) != 0)
	  {
	    hppDout (error, "polyhedron " << boxName << " already exists.");
	    return -1;
	  }

	CkppKCDPolyhedronShPtr polyhedron =
	  CkppKCDBox::create (boxName, x, y, z);

	if (!polyhedron)
	  {
	    hppDout (error, "failed to create polyhedron " << boxName);
	    return -1;
	  }
	polyhedronMap[boxName] = polyhedron;
	return 0;
      }

      Short Obstacle::addPoint
      (const char* polyhedronName, Double x, Double y, Double z)
	throw (SystemException)
      {
	// Check that polyhedron exists.
	if (polyhedronMap.count(polyhedronName) != 1)
	  {
	    hppDout (error, "polyhedron "
		     << polyhedronName << " does not exist");
	    return -1;
	  }

	unsigned int rank = 0;
	polyhedronMap[polyhedronName]->
	  CkcdPolyhedron::addPoint (static_cast<kcdReal> (x),
				    static_cast<kcdReal> (y),
				    static_cast<kcdReal> (z),
				    rank);
	return static_cast<Short> (rank);
      }

      Short
      Obstacle::addTriangle
      (const char* polyhedronName, ULong pt1, ULong pt2, ULong pt3)
	throw (SystemException)
      {
	if (polyhedronMap.count(polyhedronName) != 1)
	  {
	    hppDout (error, "polyhedron "
		     << polyhedronName << " does not exist");
	    return -1;
	  }

	CkppKCDPolyhedronShPtr polyhedron = polyhedronMap[polyhedronName];
	unsigned int rank = 0;

	// Test Kineo preconditions
	if (pt1 == pt2
	    || pt1 == pt3
	    || pt2 == pt3
	    || pt1 >= polyhedron->countPoints ()
	    || pt2 >= polyhedron->countPoints ()
	    || pt3 >= polyhedron->countPoints ())
	  {
	    hppDout (error, "invalid triangle");
	    return -1;
	  }
	polyhedron->addTriangle(pt1, pt2, pt3, rank);
	return static_cast<Short> (rank);
      }


      Short
      Obstacle::setVisible
      (const char* polyhedronName, Boolean visible) throw (SystemException)
      {
	if (polyhedronMap.count (polyhedronName) != 1)
	  {
	    hppDout (error, "collision list "
		     << polyhedronName << " does not exist");
	    return -1;
	  }
	polyhedronMap[polyhedronName]->isVisible (visible);
	return 0;
      }

      Short
      Obstacle::setTransparent
      (const char* polyhedronName, Boolean transparent)
	throw (SystemException)
      {
	if (polyhedronMap.count (polyhedronName) != 1)
	  {
	    hppDout (error, "collision list "
		     << polyhedronName << " does not exist");
	    return -1;
	  }
	polyhedronMap[polyhedronName]->isTransparent (transparent);
	return 0;
      }

    } // end of namespace implementation.
  } // end of namespace corbaServer.
} // end of namespace hpp.
