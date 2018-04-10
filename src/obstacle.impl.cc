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
	CollisionGeometryPtr_t geometry = objectMap_.geometry (objectName);

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
        return toNames_t (obstacles.begin(), obstacles.end());
      }

      void Obstacle::createPolyhedron
      (const char* polyhedronName) throw (hpp::Error)
      {
        objectMap_.createPolyhedron (polyhedronName);
      }

      void Obstacle::createBox
      (const char* boxName, Double x, Double y, Double z)
	throw (hpp::Error)
      {
        objectMap_.createBox (boxName, x, y, z);
      }

      void Obstacle::createSphere (const char* name, Double radius)
	throw (hpp::Error)
      {
        objectMap_.createSphere (name, radius);
      }

      void Obstacle::createCylinder (const char* name, Double radius, Double length)
	throw (hpp::Error)
      {
        objectMap_.createCylinder (name, radius, length);
      }

      ULong Obstacle::addPoint
      (const char* polyhedronName, Double x, Double y, Double z)
	throw (hpp::Error)
      {
	return static_cast<ULong> (
            objectMap_.addPoint (polyhedronName, x, y, z));
      }

      ULong Obstacle::addTriangle
      (const char* polyhedronName, ULong pt1, ULong pt2, ULong pt3)
	throw (hpp::Error)
      {
	return static_cast<ULong> (
            objectMap_.addTriangle (polyhedronName, pt1, pt2, pt3));
      }
    } // end of namespace implementation.
  } // end of namespace corbaServer.
} // end of namespace hpp.
