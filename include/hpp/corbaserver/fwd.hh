// Copyright (C) 2010 by Thomas Moulard and Joseph Mirabel, CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_FWD_HH
# define HPP_CORBASERVER_FWD_HH

//FIXME: should be replaced by CORBA base types forward declarations.
# include <omniORB4/CORBA.h>
# include <hpp/core/fwd.hh>

namespace hpp
{
  namespace fcl {
    template <typename T> class BVHModel;
    class CollisionGeometry;
    class RSS;
    class ShapeBase;
    class Triangle;
  }

  namespace corbaServer
  {
    class Server;
    class ServerPlugin;
    class Tools;
    class Client;
    class ProblemSolverMap;
    typedef boost::shared_ptr <ProblemSolverMap> ProblemSolverMapPtr_t;

    typedef pinocchio::BodyPtr_t BodyPtr_t;
    typedef fcl::CollisionGeometry CollisionGeometry_t;
    typedef boost::shared_ptr <CollisionGeometry_t> CollisionGeometryPtr_t;
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
    typedef fcl::BVHModel < fcl::RSS > Polyhedron_t;
    typedef boost::shared_ptr <Polyhedron_t> PolyhedronPtr_t;
    typedef fcl::ShapeBase BasicShape_t;
    typedef boost::shared_ptr <BasicShape_t> BasicShapePtr_t;

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

    namespace impl
    {
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
    }
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_FWD_HH
