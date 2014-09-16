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
#include <hpp/model/urdf/util.hh>
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

      void Obstacle::loadObstacleModel (const char* package,
					const char* filename,
					const char* prefix)
	throw (hpp::Error)
      {
	try {
	  model::DevicePtr_t device (model::Device::create
				     (std::string (filename)));
	  hpp::model::urdf::loadUrdfModel (device,
					   "anchor",
					   std::string (package),
					   std::string (filename));
	  // Detach objects from joints
	  for (ObjectIterator itObj = device->objectIterator
		 (hpp::model::COLLISION); !itObj.isEnd (); ++itObj) {
	    CollisionObjectPtr_t obj = model::CollisionObject::create
	      ((*itObj)->fcl ()->collisionGeometry(), (*itObj)->getTransform (),
	       std::string (prefix) + (*itObj)->name ());
	    problemSolver_->addObstacle (obj, true, true);
	    hppDout (info, "Adding obstacle " << obj->name ());
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Obstacle::removeObstacleFromJoint
      (const char* objectName, const char* jointName, Boolean collision,
       Boolean distance) throw (hpp::Error)
      {
	using model::JointPtr_t;
	using model::ObjectVector_t;
	using model::COLLISION;
	using model::DISTANCE;
	std::string objName (objectName);
	std::string jName (jointName);

	try {
	  if (collision) {
	    problemSolver_->removeObstacleFromJoint (objectName, jointName);
	  }
	  if (distance) {
	    throw std::runtime_error ("Not implemented.");
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void
      Obstacle::addObstacle(const char* objectName, Boolean collision,
			    Boolean distance)
	throw (hpp::Error)
      {
	std::string objName (objectName);
	CollisionGeometryPtr_t geometry;
	// Check that polyhedron exists.
	VertexMap_t::const_iterator itVertex = vertexMap_.find(objName);
	if (itVertex != vertexMap_.end ()) {
	  PolyhedronPtr_t polyhedron = PolyhedronPtr_t (new Polyhedron_t);
	  int res = polyhedron->beginModel ();
	  if (res != fcl::BVH_OK) {
	    std::ostringstream oss ("fcl BVHReturnCode = ");
	    oss << res;
	    throw hpp::Error (oss.str ().c_str ());
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
	  std::ostringstream oss ("Object ");
	  oss << objName << " does not exist.";
	  throw hpp::Error (oss.str ().c_str ());
	}

	Transform3f pos; pos.setIdentity ();
	CollisionObjectPtr_t collisionObject
	  (CollisionObject_t::create (geometry, pos, objName));
	problemSolver_->addObstacle (collisionObject, collision, distance);
      }

      CollisionObjectPtr_t Obstacle::getObstacleByName (const char* name)
      {
	const ObjectVector_t& collisionObstacles
	  (problemSolver_->collisionObstacles ());
	for (ObjectVector_t::const_iterator it = collisionObstacles.begin ();
	     it != collisionObstacles.end (); it++) {
	  CollisionObjectPtr_t object = *it;
	  if (object->name () == name) {
	    hppDout (info, "found \""
		     << object->name () << "\" in the obstacle list.");
	    return object;
	  }
	}
	const ObjectVector_t& distanceObstacles
	  (problemSolver_->distanceObstacles ());
	for (ObjectVector_t::const_iterator it = distanceObstacles.begin ();
	     it != distanceObstacles.end (); it++) {
	  CollisionObjectPtr_t object = *it;
	  if (object->name () == name) {
	    hppDout (info, "found \""
		     << object->name () << "\" in the obstacle list.");
	    return object;
	  }
	}
	return CollisionObjectPtr_t ();
      }

      void Obstacle::moveObstacle
      (const char* objectName, const CORBA::Double* cfg)
	throw(hpp::Error)
      {
	CollisionObjectPtr_t object = getObstacleByName (objectName);
	if (object) {
	  Transform3f mat;
	  hppTransformToTransform3f (cfg, mat);
	  object->move (mat);
	  return;
	}
	std::ostringstream oss ("Object ");
	oss << objectName <<  " not found";
	throw hpp::Error (oss.str ().c_str ());
      }

      void Obstacle::getObstaclePosition (const char* objectName,
					  Double* cfg)
	  throw (hpp::Error)
      {
	CollisionObjectPtr_t object = getObstacleByName (objectName);
	if (object) {
	  Transform3f transform = object->getTransform ();
	  Transform3fTohppTransform (transform, cfg);
	  return;
	}
	std::ostringstream oss ("Object ");
	oss << objectName <<  " not found";
	throw hpp::Error (oss.str ().c_str ());
      }

      void Obstacle::createPolyhedron
      (const char* polyhedronName) throw (hpp::Error)
      {
	// Check that polyhedron does not already exist.
	if (vertexMap_.find(polyhedronName) != vertexMap_.end ()) {
	  std::ostringstream oss ("polyhedron ");
	  oss << polyhedronName << " already exists.";
	  throw hpp::Error (oss.str ().c_str ());
	}
	vertexMap_ [polyhedronName] = std::vector <fcl::Vec3f> ();
	triangleMap_ [polyhedronName] = std::vector <fcl::Triangle> ();
      }

      void Obstacle::createBox
      (const char* boxName, Double x, Double y, Double z)
	throw (hpp::Error)
      {
	std::string shapeName(boxName);
	// Check that object does not already exist.
	if (vertexMap_.find(shapeName) != vertexMap_.end () ||
	    shapeMap_.find (shapeName) != shapeMap_.end ()) {
	  std::ostringstream oss ("object ");
	  oss << shapeName << " already exists.";
	  throw hpp::Error (oss.str ().c_str ());
	}
	BasicShapePtr_t box (new fcl::Box ( x, y, z));
	shapeMap_[shapeName] = box;
      }

      Short Obstacle::addPoint
      (const char* polyhedronName, Double x, Double y, Double z)
	throw (hpp::Error)
      {
	// Check that polyhedron exists.
	VertexMap_t::iterator itVertex = vertexMap_.find (polyhedronName);
	if (itVertex == vertexMap_.end ()) {
	  std::ostringstream oss ("polyhedron ");
	  oss << polyhedronName << " does not exist.";
	  throw hpp::Error (oss.str ().c_str ());
	}
	itVertex->second.push_back (fcl::Vec3f (x, y, z));
	return static_cast<Short> (vertexMap_.size ());
      }

      Short
      Obstacle::addTriangle
      (const char* polyhedronName, ULong pt1, ULong pt2, ULong pt3)
	throw (hpp::Error)
      {
	// Check that polyhedron exists.
	TriangleMap_t::iterator itTriangle = triangleMap_.find (polyhedronName);
	if (itTriangle == triangleMap_.end ()) {
	  std::ostringstream oss ("polyhedron ");
	  oss << polyhedronName << " does not exist.";
	  throw hpp::Error (oss.str ().c_str ());
	}

	itTriangle->second.push_back (fcl::Triangle (pt1, pt2, pt3));
	return static_cast<Short> (triangleMap_.size ());
      }
    } // end of namespace implementation.
  } // end of namespace corbaServer.
} // end of namespace hpp.
