// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include "obstacle.impl.hh"

#include <coal/BVH/BVH_model.h>
#include <coal/mesh_loader/loader.h>
#include <coal/shape/geometric_shapes.h>

#include <hpp/corbaserver/server-plugin.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>
#include <iostream>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/utils.hpp>

#include "tools.hh"

namespace hpp {
namespace corbaServer {
namespace impl {
Obstacle::Obstacle() : server_(NULL) {}

core::ProblemSolverPtr_t Obstacle::problemSolver() {
  return server_->problemSolver();
}

void Obstacle::loadObstacleModel(const char* filename, const char* prefix) {
  try {
    DevicePtr_t device(Device::create(prefix));
    hpp::pinocchio::urdf::loadModel(device, 0, "", "anchor", filename, "");

    problemSolver()->addObstacle(device, true, true);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Obstacle::loadObstacleModelFromString(const char* urdfString,
                                           const char* prefix) {
  try {
    DevicePtr_t device(Device::create(prefix));
    hpp::pinocchio::urdf::loadModelFromString(device, 0, "", "anchor",
                                              urdfString, "");
    problemSolver()->addObstacle(device, true, true);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Obstacle::loadPolyhedron(const char* name, const char* resourcename) {
  try {
    coal::MeshLoader loader(coal::BV_OBBRSS);
    std::string filename(::pinocchio::retrieveResourcePath(
        resourcename, ::pinocchio::rosPaths()));
    coal::CollisionGeometryPtr_t coalgeom(
        loader.load(filename, coal::Vec3f::Ones()));
    CollisionGeometryPtr_t geom(coalgeom.get(),
                                [coalgeom](...) mutable { coalgeom.reset(); });
    problemSolver()->addObstacle(name, geom, Transform3s::Identity(), true,
                                 true);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Obstacle::removeObstacleFromJoint(const char* objectName,
                                       const char* jointName, Boolean collision,
                                       Boolean distance) {
  using pinocchio::COLLISION;
  using pinocchio::DISTANCE;
  using pinocchio::JointPtr_t;
  using pinocchio::ObjectVector_t;
  std::string objName(objectName);
  std::string jName(jointName);

  try {
    if (collision) {
      problemSolver()->removeObstacleFromJoint(objectName, jointName);
    }
    if (distance) {
      throw std::runtime_error("Not implemented.");
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Obstacle::removeObstacle(const char* objectName) {
  try {
    problemSolver()->removeObstacle(objectName);
  } catch (std::exception& e) {
    throw Error(e.what());
  }
}

void Obstacle::cutObstacle(const char* objectName, const floatSeq& aabb) {
  try {
    vector_t _aabb = floatSeqToVector(aabb, 6);
    coal::AABB coalaabb(_aabb.head<3>(), _aabb.tail<3>());
    problemSolver()->cutObstacle(objectName, coalaabb);
  } catch (const std::exception& e) {
    throw hpp::Error(e.what());
  }
}

void Obstacle::addObstacle(const char* objectName, Boolean collision,
                           Boolean distance) {
  DevicePtr_t robot = getRobotOrThrow(problemSolver());

  std::string objName(objectName);
  CollisionGeometryPtr_t geometry = objectMap_.geometry(objectName);
  problemSolver()->addObstacle(objName, geometry, Transform3s::Identity(),
                               collision, distance);
}

CollisionObjectPtr_t Obstacle::getObstacleByName(const char* name) {
  try {
    return problemSolver()->obstacle(name);
  } catch (const std::invalid_argument&) {
    HPP_THROW(Error, "Object " << name << " not found");
  }
}

void Obstacle::moveObstacle(const char* objectName, const Transform_ cfg) {
  CollisionObjectPtr_t object = getObstacleByName(objectName);
  if (object) {
    object->move(toTransform3s(cfg));
    return;
  }
  HPP_THROW(Error, "Object " << objectName << " not found");
}

void Obstacle::getObstaclePosition(const char* objectName, Transform_ cfg) {
  CollisionObjectPtr_t object = getObstacleByName(objectName);
  if (object) {
    Transform3s transform = object->getTransform();
    toHppTransform(transform, cfg);
    return;
  }
  try {
    Transform3s transform = problemSolver()->obstacleFramePosition(objectName);
    toHppTransform(transform, cfg);
    return;
  } catch (const std::invalid_argument&) {
    // Ignore this exception.
  } catch (const std::exception& e) {
    throw Error(e.what());
  }
  HPP_THROW(Error, "Object " << objectName << " not found");
}

Names_t* Obstacle::getObstacleNames(bool collision, bool distance) {
  std::list<std::string> obstacles =
      problemSolver()->obstacleNames(collision, distance);
  return toNames_t(obstacles.begin(), obstacles.end());
}

void Obstacle::createPolyhedron(const char* polyhedronName) {
  objectMap_.createPolyhedron(polyhedronName);
}

void Obstacle::createBox(const char* boxName, Double x, Double y, Double z) {
  objectMap_.createBox(boxName, x, y, z);
}

void Obstacle::createSphere(const char* name, Double radius) {
  objectMap_.createSphere(name, radius);
}

void Obstacle::createCylinder(const char* name, Double radius, Double length) {
  objectMap_.createCylinder(name, radius, length);
}

ULong Obstacle::addPoint(const char* polyhedronName, Double x, Double y,
                         Double z) {
  return static_cast<ULong>(objectMap_.addPoint(polyhedronName, x, y, z));
}

ULong Obstacle::addTriangle(const char* polyhedronName, ULong pt1, ULong pt2,
                            ULong pt3) {
  return static_cast<ULong>(
      objectMap_.addTriangle(polyhedronName, pt1, pt2, pt3));
}
}  // namespace impl
}  // end of namespace corbaServer.
}  // end of namespace hpp.
