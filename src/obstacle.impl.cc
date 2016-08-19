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

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/geometric_shapes.h>

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
      Obstacle::Obstacle (corbaServer::Server* server)
	: server_ (server)
      {}

      core::ProblemSolverPtr_t Obstacle::problemSolver()
      {
        return server_->problemSolver();
      }

      void Obstacle::loadObstacleModel (const char* package,
					const char* filename,
					const char* prefix)
	throw (hpp::Error)
      {
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
	try {
	  pinocchio::DevicePtr_t device (pinocchio::Device::create
				     (std::string (filename)));
	  hpp::pinocchio::urdf::loadUrdfModel (device,
					   "freeflyer",
					   std::string (package),
					   std::string (filename));

          device->computeForwardKinematics();
          se3::updateGeometryPlacements(device->model(), device->data(),
              device->geomModel(), device->geomData());

          using pinocchio::DeviceObjectVector;
          const se3::Model & obsModel = device->model();
          const se3::Data  & obsData  = device->data ();
          se3::Model &       robModel = robot->model();
	  // Copy the bodies
          for(se3::FrameIndex fid = 0; fid < obsModel.nFrames; ++fid) {
            const se3::Frame& frame = obsModel.frames[fid];
            if (frame.type != se3::BODY) continue;
            robModel.addFrame (
                std::string (prefix) + frame.name,
                0,
                obsData.oMi[frame.parent] * frame.placement,
                se3::BODY
                );
          }

	  // Detach objects from joints
          DeviceObjectVector& objects = device->objectVector();
          for (DeviceObjectVector::iterator itObj = objects.begin();
              itObj != objects.end(); ++itObj) {
            CollisionObjectPtr_t obj = *itObj;
            se3::FrameIndex fid = robModel.getBodyId(
                std::string (prefix) + obsModel.getFrameName(obj->pinocchio().parentFrame)
                );
            assert (robModel.getFrameParent(fid) == 0);
            assert (robModel.getFrameType(fid) == se3::BODY);
            se3::GeomIndex idx =
              robot->geomModel().addGeometryObject (
                  robModel,
                  fid, 
                  obj->fcl()->collisionGeometry(),
                  obj->getTransform (),
                  std::string (prefix) + obj->name ());
	    CollisionObjectPtr_t newObj (new pinocchio::CollisionObject (robot, idx));
	    problemSolver()->addObstacle (newObj, true, true);
	    hppDout (info, "Adding obstacle " << newObj->name ());
	  }

          robot->computeForwardKinematics();
          se3::updateGeometryPlacements(robot->model(), robot->data(),
              robot->geomModel(), robot->geomData());

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

	Transform3f pos; pos.setIdentity ();
        se3::GeomIndex idx =
          robot->geomModel().addGeometryObject (
              robot->model(), robot->model().getFrameId(objName),
              geometry, pos, objName);
        CollisionObjectPtr_t obj (new pinocchio::CollisionObject (robot, idx));
	problemSolver()->addObstacle (obj, collision, distance);
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

      void Obstacle::getObstacleLinkPosition (const char* linkName,
					      Double* cfg)
	  throw (hpp::Error)
      {
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
	try {
          /// Get body frame
          const se3::Model & model = robot->model();
          se3::FrameIndex fid = model.getBodyId(linkName);
          if (fid > model.nFrames)
            throw BuildException<std::invalid_argument>() << "No body named " << linkName << BuildExceptionEnd();
          const se3::Frame& body = model.frames[fid];
          if (body.type != se3::BODY)
            throw BuildException<std::invalid_argument>() << "No body named " << linkName << BuildExceptionEnd();

          /// Compute body position
          const se3::Data  & data  = robot->data ();
          Transform3fTohppTransform (
              data.oMi[body.parent] * body.placement,
              cfg);
        } catch (const std::exception& e) {
          throw hpp::Error (e.what ());
        }
      }

      Names_t* Obstacle::getObstacleLinkNames ()
	throw (hpp::Error)
      {
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
	try {
          std::vector<std::string> names;
          const se3::Model & model = robot->model();
          for (se3::FrameIndex fid = 0; fid < model.nFrames; ++fid) {
            const se3::Frame& frame = model.frames[fid];
            if (frame.type == se3::BODY && frame.parent == 0)
              names.push_back(frame.name);
          }
          return toNames_t (names.begin(), names.end());
        } catch (const std::exception& e) {
          throw hpp::Error (e.what ());
        }
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
