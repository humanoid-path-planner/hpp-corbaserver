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
#include <pinocchio/parsers/utils.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include "tools.hh"

#include <hpp/corbaserver/server-plugin.hh>

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      Obstacle::Obstacle ()
	: server_ (NULL)
      {}

      core::ProblemSolverPtr_t Obstacle::problemSolver()
      {
        return server_->problemSolver();
      }

      void Obstacle::loadObstacleModel (const char* filename,
					const char* prefix)
      {
	try {
          DevicePtr_t device (Device::create (prefix));
          hpp::pinocchio::urdf::loadModel (device, 0, "", "anchor", filename,
                                           "");
          device->controlComputation(hpp::pinocchio::JOINT_POSITION);

          problemSolver()->addObstacle (device, true, true);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Obstacle::loadObstacleModelFromString (const char* urdfString,
                                                  const char* prefix)
      {
	try {
          DevicePtr_t device (Device::create (prefix));
          hpp::pinocchio::urdf::loadModelFromString
            (device, 0, "", "anchor", urdfString, "");
          device->controlComputation(hpp::pinocchio::JOINT_POSITION);

          problemSolver()->addObstacle (device, true, true);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Obstacle::loadPolyhedron (const char* name, const char* resourcename)
      {
	try {
          fcl::MeshLoader loader (fcl::BV_OBBRSS);
          std::string filename (
              ::pinocchio::retrieveResourcePath(resourcename,
                ::pinocchio::rosPaths()));
          fcl::CollisionGeometryPtr_t geom = loader.load(filename, fcl::Vec3f::Ones());
          fcl::CollisionObject object (geom);
          problemSolver()->addObstacle (name, object, true, true);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Obstacle::removeObstacleFromJoint
      (const char* objectName, const char* jointName, Boolean collision,
       Boolean distance)
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

      void Obstacle::removeObstacle (const char* objectName)
      {
        try {
          problemSolver()->removeObstacle (objectName);
        } catch (std::exception& e) {
          throw Error (e.what ());
        }
      }

      void Obstacle::cutObstacle (const char* objectName, const floatSeq& aabb)
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
        try {
          return problemSolver()->obstacle(name);
        } catch (const std::invalid_argument&) {
          HPP_THROW(Error, "Object " << name << " not found");
        }
      }

      void Obstacle::moveObstacle
      (const char* objectName, const Transform_ cfg)
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
      {
	CollisionObjectPtr_t object = getObstacleByName (objectName);
	if (object) {
	  Transform3f transform = object->getTransform ();
          toHppTransform (transform, cfg);
	  return;
	}
        try {
          Transform3f transform = problemSolver()->obstacleFramePosition (objectName);
          toHppTransform (transform, cfg);
	  return;
        } catch (const std::invalid_argument&) {
          // Ignore this exception.
        } catch (const std::exception& e) {
          throw Error (e.what());
        }
        HPP_THROW(Error, "Object " << objectName << " not found");
      }

      Names_t* Obstacle::getObstacleNames (bool collision, bool distance)
      {
	std::list <std::string> obstacles =
	  problemSolver()->obstacleNames (collision, distance);
        return toNames_t (obstacles.begin(), obstacles.end());
      }

      void Obstacle::createPolyhedron
      (const char* polyhedronName)
      {
        objectMap_.createPolyhedron (polyhedronName);
      }

      void Obstacle::createBox
      (const char* boxName, Double x, Double y, Double z)
      {
        objectMap_.createBox (boxName, x, y, z);
      }

      void Obstacle::createSphere (const char* name, Double radius)
      {
        objectMap_.createSphere (name, radius);
      }

      void Obstacle::createCylinder (const char* name, Double radius, Double length)
      {
        objectMap_.createCylinder (name, radius, length);
      }

      ULong Obstacle::addPoint
      (const char* polyhedronName, Double x, Double y, Double z)
      {
	return static_cast<ULong> (
            objectMap_.addPoint (polyhedronName, x, y, z));
      }

      ULong Obstacle::addTriangle
      (const char* polyhedronName, ULong pt1, ULong pt2, ULong pt3)
      {
	return static_cast<ULong> (
            objectMap_.addTriangle (polyhedronName, pt1, pt2, pt3));
      }
    } // end of namespace implementation.
  } // end of namespace corbaServer.
} // end of namespace hpp.
