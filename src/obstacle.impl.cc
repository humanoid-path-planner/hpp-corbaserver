// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include "obstacle.impl.hh"

#include <iostream>

#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include "tools.hh"

#include "hpp/corbaserver/server.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      using pinocchio::DeviceObjectVector;

      Obstacle::Obstacle (corbaServer::Server* server)
	: server_ (server)
      {}

      core::ProblemSolverPtr_t Obstacle::problemSolver()
      {
        return server_->problemSolver();
      }

      void Obstacle::loadObstacleModel (const char* package,
					const char* file,
					const char* prefix)
	throw (hpp::Error)
      {
	try {
          std::string pkg (package);
          DevicePtr_t device (Device::create (prefix));
          if (pkg.empty())
            hpp::pinocchio::urdf::loadModelFromString (
                device, 0, "", "anchor", file, "");
          else
            hpp::pinocchio::urdf::loadUrdfModel (
                device, "anchor", pkg, file);
          device->controlComputation(Device::JOINT_POSITION);
          device->computeForwardKinematics();
          device->updateGeometryPlacements();

	  // Detach objects from joints
          DeviceObjectVector& objects = device->objectVector();
          for (DeviceObjectVector::iterator itObj = objects.begin();
              itObj != objects.end(); ++itObj) {
            problemSolver()->addObstacle (
                std::string (prefix) + (*itObj)->name (),
                *(*itObj)->fcl (),
                true, true);
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Obstacle::removeObstacleFromJoint
      (const char* objectName, const char* jointName, Boolean collision,
       Boolean distance) throw (hpp::Error)
      {
	using pinocchio::JointPtr_t;
	using pinocchio::ObjectVector_t;
	using pinocchio::COLLISION;
	using pinocchio::DISTANCE;
	std::string objName (objectName);
	std::string jName (jointName);

	try {
	  if (collision) {
	    problemSolver()->removeObstacleFromJoint (objectName, jointName);
	  }
	  if (distance) {
	    throw std::runtime_error ("Not implemented.");
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Obstacle::cutObstacle (const char* objectName, const floatSeq& aabb)
        throw (Error)
      {
        try {
          vector_t _aabb = floatSeqToVector(aabb, 6);
          fcl::AABB fclaabb (_aabb.head<3>(), _aabb.tail<3>());
          problemSolver()->cutObstacle(objectName, fclaabb);
        } catch (const std::exception& e) {
          throw hpp::Error (e.what ());
        }
      }

      void
      Obstacle::addObstacle(const char* objectName, Boolean collision,
			    Boolean distance)
	throw (hpp::Error)
      {
        DevicePtr_t robot = getRobotOrThrow(problemSolver());

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

        fcl::Transform3f pos; pos.setIdentity ();
        core::FclCollisionObject object (geometry, pos);
	problemSolver()->addObstacle (objName, object, collision, distance);
      }

      CollisionObjectPtr_t Obstacle::getObstacleByName (const char* name)
      {
	ObjectStdVector_t collisionObstacles (problemSolver()->collisionObstacles ());
	for (ObjectStdVector_t::iterator it = collisionObstacles.begin ();
	     it != collisionObstacles.end (); it++) {
	  CollisionObjectPtr_t object = *it;
	  if (object->name () == name) {
	    hppDout (info, "found \""
		     << object->name () << "\" in the obstacle list.");
	    return object;
	  }
	}
	ObjectStdVector_t distanceObstacles
	  (problemSolver()->distanceObstacles ());
	for (ObjectStdVector_t::iterator it = distanceObstacles.begin ();
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
      (const char* objectName, const Transform_ cfg)
	throw(hpp::Error)
      {
	CollisionObjectPtr_t object = getObstacleByName (objectName);
	if (object) {
	  object->move (toTransform3f(cfg));
	  return;
        }
        HPP_THROW(Error, "Object " << objectName << " not found");
      }

      void Obstacle::getObstaclePosition (const char* objectName,
					  Transform_ cfg)
	  throw (hpp::Error)
      {
	CollisionObjectPtr_t object = getObstacleByName (objectName);
	if (object) {
	  Transform3f transform = object->getTransform ();
          toHppTransform (transform, cfg);
	  return;
	}
        HPP_THROW(Error, "Object " << objectName << " not found");
      }

      Names_t* Obstacle::getObstacleNames (bool collision, bool distance)
	throw (hpp::Error)
      {
	std::list <std::string> obstacles =
	  problemSolver()->obstacleNames (collision, distance);
	ULong size = (ULong) obstacles.size ();
	char** nameList = Names_t::allocbuf(size);
	Names_t *result = new Names_t (size, size, nameList);
	std::size_t i = 0;
	for (std::list <std::string>::const_iterator it = obstacles.begin ();
	     it != obstacles.end (); ++it) {
	  std::string name = *it;
	  nameList [i] = (char*) malloc (sizeof (char) * (name.length () + 1));
	  strcpy (nameList [i], name.c_str ());
	  ++i;
	}
	return result;
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

      CORBA::Long Obstacle::addPoint
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
	return static_cast<CORBA::Long> (vertexMap_.size ());
      }

      CORBA::Long
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
	return static_cast<CORBA::Long> (triangleMap_.size ());
      }
    } // end of namespace implementation.
  } // end of namespace corbaServer.
} // end of namespace hpp.
