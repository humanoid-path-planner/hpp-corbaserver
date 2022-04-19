// Copyright (C) 2010 by Thomas Moulard and Joseph Mirabel, CNRS.
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

#ifndef HPP_CORBASERVER_FWD_HH
#define HPP_CORBASERVER_FWD_HH

// FIXME: should be replaced by CORBA base types forward declarations.
#include <omniORB4/CORBA.h>

#include <hpp/core/fwd.hh>

namespace hpp {
namespace fcl {
template <typename T>
class BVHModel;
class CollisionGeometry;
class OBBRSS;
class ShapeBase;
class Triangle;
}  // namespace fcl

namespace corbaServer {
class Server;
class ServerPlugin;
class Tools;
class Client;
class ProblemSolverMap;
typedef shared_ptr<ProblemSolverMap> ProblemSolverMapPtr_t;

typedef pinocchio::BodyPtr_t BodyPtr_t;
using fcl::CollisionGeometry;
using fcl::CollisionGeometryPtr_t;
typedef pinocchio::CollisionObject CollisionObject_t;
typedef pinocchio::CollisionObjectPtr_t CollisionObjectPtr_t;
typedef pinocchio::CollisionObjectConstPtr_t CollisionObjectConstPtr_t;
typedef pinocchio::Configuration_t Configuration_t;
typedef core::ConfigurationPtr_t ConfigurationPtr_t;
typedef core::ConfigIterator_t ConfigIterator_t;
typedef core::ConfigConstIterator_t ConfigConstIterator_t;
typedef core::ConnectedComponent ConnectedComponent;
typedef core::ConnectedComponents_t ConnectedComponents_t;
typedef core::ConnectedComponentPtr_t ConnectedComponentPtr_t;
typedef pinocchio::Device Device;
typedef pinocchio::DevicePtr_t DevicePtr_t;
typedef pinocchio::DistanceResults_t DistanceResults_t;
typedef core::CollisionPairs_t CollisionPairs_t;
typedef core::DistanceBetweenObjects DistanceBetweenObjects;
typedef core::DistanceBetweenObjectsPtr_t DistanceBetweenObjectsPtr_t;
typedef core::Edges_t Edges_t;
typedef pinocchio::Joint Joint;
typedef pinocchio::Frame Frame;
typedef pinocchio::JointPtr_t JointPtr_t;
typedef pinocchio::JointVector_t JointVector_t;
/// Plane polygon represented by its vertices
/// Used to model contact surfaces for manipulation applications
typedef constraints::Shape_t Shape_t;
typedef constraints::JointAndShape_t JointAndShape_t;
typedef constraints::JointAndShapes_t JointAndShapes_t;

typedef core::LockedJoint LockedJoint;
typedef core::LockedJointPtr_t LockedJointPtr_t;
typedef core::Nodes_t Nodes_t;
typedef core::NodeVector_t NodeVector_t;
typedef core::ObjectVector_t ObjectVector_t;
typedef core::ObjectStdVector_t ObjectStdVector_t;
// typedef pinocchio::ObjectIterator ObjectIterator;
typedef core::PathPtr_t PathPtr_t;
typedef core::PathValidationReportPtr_t PathValidationReportPtr_t;
typedef core::PathVector PathVector_t;
typedef core::PathVectorPtr_t PathVectorPtr_t;
typedef core::SteeringMethod SteeringMethod_t;
typedef core::SteeringMethodPtr_t SteeringMethodPtr_t;
typedef pinocchio::Transform3f Transform3f;
typedef fcl::BVHModel<fcl::OBBRSS> Polyhedron_t;
typedef fcl::shared_ptr<Polyhedron_t> PolyhedronPtr_t;
typedef fcl::ShapeBase BasicShape_t;
typedef fcl::shared_ptr<BasicShape_t> BasicShapePtr_t;

typedef pinocchio::value_type value_type;
typedef pinocchio::matrix_t matrix_t;
typedef pinocchio::matrix3_t matrix3_t;
typedef pinocchio::vector_t vector_t;
typedef pinocchio::vector3_t vector3_t;
typedef pinocchio::ComJacobian_t ComJacobian_t;
typedef pinocchio::size_type size_type;
typedef pinocchio::LiegroupElement LiegroupElement;
typedef pinocchio::LiegroupSpace LiegroupSpace;
typedef pinocchio::LiegroupSpacePtr_t LiegroupSpacePtr_t;

namespace impl {
using CORBA::Boolean;
using CORBA::Double;
using CORBA::Short;
using CORBA::SystemException;
using CORBA::ULong;
using CORBA::UShort;

class Problem;
class Obstacle;
class Robot;
class Server;
}  // namespace impl
}  // end of namespace corbaServer.
}  // end of namespace hpp.

#endif  //! HPP_CORBASERVER_FWD_HH
