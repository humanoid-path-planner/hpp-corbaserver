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
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <hpp/model/collision-object.hh>
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
	  problemSolver_ (server->problemSolver ())
      {}

      Short
      Obstacle::addObstacle(const char* objectName, Boolean collision,
			    Boolean distance)
	throw (SystemException)
      {
	std::string objName (objectName);
	CollisionGeometryPtr_t geometry;
	// Check that polyhedron exists.
	VertexMap_t::const_iterator itVertex = vertexMap_.find(objName);
	if (itVertex != vertexMap_.end ()) {
	  PolyhedronPtr_t polyhedron = PolyhedronPtr_t (new Polyhedron_t);
	  int res = polyhedron->beginModel ();
	  if (res != fcl::BVH_OK) {
	    hppDout (error, "fcl BVHReturnCode = " << res);
	    return -1;
	  }

	  polyhedron->addSubModel (itVertex->second, triangleMap_ [objName]);
	  polyhedron->endModel ();
	  geometry = polyhedron;
	} else {
	  ShapeMap_t::iterator itShape = shapeMap_.find (objName);
	  if (itShape != shapeMap_.end ()) {
	    geometry = itShape->second;
	  }
	}
	if (!geometry) {
	  hppDout (error, "Object " << objName << " does not exist.");
	  return -1;
	}

	Transform3f pos; pos.setIdentity ();
	CollisionObjectPtr_t collisionObject
	  (new CollisionObject_t (geometry, pos, objName));
	problemSolver_->addObstacle (collisionObject, collision, distance);
	return 0;
      }

      Short
      Obstacle::moveObstacle
      (const char* objectName, const hpp::Configuration& cfg)
	throw(SystemException)
      {
	const ObjectVector_t& collisionObstacles
	  (problemSolver_->problem ()->collisionObstacles ());
	for (ObjectVector_t::const_iterator it = collisionObstacles.begin ();
	     it != collisionObstacles.end (); it++) {
	  CollisionObjectPtr_t object = *it;
	  if (object->name () == objectName) {
	    hppDout (info, "found \""
		     << object->name () << "\" in the obstacle list.");
	    Transform3f mat;
	    ConfigurationToTransform3f (cfg, mat);
	    object->move (mat);
	    return 0;
	  }
	}
	const ObjectVector_t& distanceObstacles
	  (problemSolver_->problem ()->distanceObstacles ());
	for (ObjectVector_t::const_iterator it = distanceObstacles.begin ();
	     it != distanceObstacles.end (); it++) {
	  CollisionObjectPtr_t object = *it;
	  if (object->name () == objectName) {
	    hppDout (info, "found \""
		     << object->name () << "\" in the obstacle list.");
	    Transform3f mat;
	    ConfigurationToTransform3f (cfg, mat);
	    object->move (mat);
	    return 0;
	  }
	}
	hppDout (error, "failed to find ``" << objectName << "'' in the tree.");
	return -1;
      }

      Short
      Obstacle::createPolyhedron
      (const char* polyhedronName) throw (SystemException)
      {
	// Check that polyhedron does not already exist.
	if (vertexMap_.find(polyhedronName) != vertexMap_.end ()) {
	  hppDout (error, "polyhedron "	 << polyhedronName
		   << " already exists.");
	  return -1;
	}
	vertexMap_ [polyhedronName] = std::vector <fcl::Vec3f> ();
	triangleMap_ [polyhedronName] = std::vector <fcl::Triangle> ();
	return 0;
      }

      Short Obstacle::createBox
      (const char* boxName, Double x, Double y, Double z)
	throw (SystemException)
      {
	std::string shapeName(boxName);
	// Check that object does not already exist.
	if (vertexMap_.find(shapeName) != vertexMap_.end () ||
	    shapeMap_.find (shapeName) != shapeMap_.end ()) {
	  hppDout (info, "object " << shapeName << " already exists.");
	  return -1;
	}
	BasicShapePtr_t box (new fcl::Box ( x, y, z));
	shapeMap_[shapeName] = box;
	return 0;
      }

      Short Obstacle::addPoint
      (const char* polyhedronName, Double x, Double y, Double z)
	throw (SystemException)
      {
	// Check that polyhedron exists.
	VertexMap_t::iterator itVertex = vertexMap_.find (polyhedronName);
	if (itVertex == vertexMap_.end ()) {
	  hppDout (error, "polyhedron " << polyhedronName
		   << " does not exist.");
	  return -1;
	}
	itVertex->second.push_back (fcl::Vec3f (x, y, z));
	return static_cast<Short> (vertexMap_.size ());
      }

      Short
      Obstacle::addTriangle
      (const char* polyhedronName, ULong pt1, ULong pt2, ULong pt3)
	throw (SystemException)
      {
	// Check that polyhedron exists.
	TriangleMap_t::iterator itTriangle = triangleMap_.find (polyhedronName);
	if (itTriangle == triangleMap_.end ()) {
	  hppDout (error, "polyhedron " << polyhedronName
		   << " does not exist.");
	  return -1;
	}

	itTriangle->second.push_back (fcl::Triangle (pt1, pt2, pt3));
	return static_cast<Short> (triangleMap_.size ());
      }
    } // end of namespace implementation.
  } // end of namespace corbaServer.
} // end of namespace hpp.
