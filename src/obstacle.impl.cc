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

#include "hpp/corbaserver/server.hh"
#if HPP_CORBASERVER_ENABLE_OPENHRP
# include "hpp/corbaserver/openhrp.hh"
#endif // HPP_CORBASERVER_ENABLE_OPENHRP

#include "obstacle.impl.hh"
#include "tools.hh"

#include "config.h"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      Obstacle::Obstacle (Server* server)
	: server_ (server),
	  planner_ (server->getHppPlanner ())
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
	hppPolyhedron->makeCollisionEntity();
	planner_->addObstacle(hppPolyhedron);
	return 0;
      }

      Short
      Obstacle::addObstacleConfig
      (const char* polyhedronName, const hppCorbaServer::Configuration& cfg)
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
	polyhedron->makeCollisionEntity();

	CkitMat4 mat;
	ConfigurationToCkitMat4(cfg, mat);

	polyhedron->setAbsolutePosition(mat);
	planner_->addObstacle(polyhedron);
	return 0;
      }

      Short
      Obstacle::moveObstacleConfig
      (const char* polyhedronName, const hppCorbaServer::Configuration& cfg)
	throw(SystemException)
      {
	BOOST_FOREACH (planner_->obstacleList (), CkcdObjectShPtr object)
	  {
	    CkppKCDPolyhedronShPtr polyhedron =
	      KIT_DYNAMIC_PTR_CAST (CkppKCDPolyhedron, object);
	    if (polyhedron && polyhedronName == poly->name ())
	      {
		hppDout (info, "found ``"
			 << polyhedronName << "'' in the tree.");
		CkitMat4 mat;
		ConfigurationToCkitMat4 (cfg, mat);
		polyhedron->setAbsolutePosition (mat);
		return 0;
	      }
	  }
	hppDout (error, "failed to find ``"
		 << polyhedronName << "'' in the tree.");
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
	polyhedron->makeCollisionEntity ();
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
	if (polyhedron)
	  {
	    hppDout (error, "failed to create polyhedron " << polyhedronName);
	    return -1;
	  }
	polyhedronMap[polyhedronName] = polyhedron;
	return 0;
      }

      Short
      Obstacle::createBox
      (const char* inBoxName, Double x, Double y, Double z)
	throw (SystemException)
      {
	// Check that polyhedron does not already exist.
	if (polyhedronMap.count(polyhedronName) != 0)
	  {
	    hppDout (error, "polyhedron "
		     << polyhedronName << " already exists.");
	    return -1;
	  }

	CkppKCDPolyhedronShPtr polyhedron =
	  CkppKCDBox::create (polyhedronName, x, y, z);

	if (!polyhedron)
	  {
	    hppDout (error, "failed to create polyhedron " << polyhedronName);
	    return -1;
	  }
	polyhedronMap[polyhedronName] = polyhedron;
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
	polyhedronMap[polyhedronName]->CkcdPolyhedron::addPoint (x, y, z, rank);
	return rank;
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
	return rank;
      }

      Short
      Obstacle::loadModelLoaderObstacle(const char* inPolyName,
					const char* inFilename,
					const char* inDirectory)
	throw (SystemException)
      {
#if HPP_CORBASERVER_ENABLE_OPENHRP
	hppDout (error, "failed to load obstacle: OpenHRP support is disabled");
	return -1;
#else

	ChppciOpenHrpClient openHrpClient (planner_);

	// Create empty polyhedron
	CkppKCDPolyhedronShPtr polyhedron =
	  CkppKCDPolyhedron::create (polyName);

	// Test whether directory is provided.
	// If not take default argument

	ktStatus status = KD_OK;

	if (directory == "")
	  status = openHrpClient.loadObstacleModel
	    (filename, polyName, polyhedron);
	else
	  status = openHrpClient.loadObstacleModel
	    (filename, polyName, polyhedron, directory);

	if (status != KD_OK)
	  {
	    hppDout (error, "failed to load obstacle from Modelloader");
	    hppDout (error, "OpenHRP prefix: "
		     << ((directory == "") ? "default" : directory));
	    return -1;
	  }

	if (planner_->addObstacle (polyhedron) != KD_OK)
	  {
	    hppDout (error, "failed to add obstacle");
	    return -1;
	  }

	hppDout (info, "obstacle ``" << PolyName << "'' sucessfully loaded");
	return 0;
#endif //! HPP_CORBASERVER_ENABLE_OPENHRP
      }

      Short
      Obstacle::setVisible
      (const char* inPolyname, Boolean inVisible) throw (SystemException)
      {
	if (polyhedronMap.count (polyhedronName) != 1)
	  {
	    hppDout (error, "collision list " << listName << " does not exist");
	    return -1;
	  }
	polyhedronMap[polyhedronName]->isVisible (visible);
	return 0;
      }

      Short
      Obstacle::setTransparent
      (const char* inPolyname, Boolean inTransparent) throw (SystemException)
      {
	if (polyhedronMap.count (polyhedronName) != 1)
	  {
	    hppDout (error, "collision list " << listName << " does not exist");
	    return -1;
	  }
	polyhedronMap[polyhedronName]->isTransparent(inTransparent);
	return 0;
      }

    } // end of namespace implementation.
  } // end of namespace corbaServer.
} // end of namespace hpp.
