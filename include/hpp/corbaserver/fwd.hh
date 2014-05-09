// Copyright (C) 2010 by Thomas Moulard, CNRS.
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
# include <fcl/math/vec_3f.h>
# include <hpp/core/fwd.hh>

namespace fcl {
  template <typename T> class BVHModel;
  class CollisionGeometry;
  class RSS;
  class ShapeBase;
  class Triangle;
}

namespace hpp
{
  namespace corbaServer
  {
    class Server;
    typedef model::Body* BodyPtr_t;
    typedef fcl::CollisionGeometry CollisionGeometry_t;
    typedef boost::shared_ptr <CollisionGeometry_t> CollisionGeometryPtr_t;
    typedef model::CollisionObject CollisionObject_t;
    typedef model::CollisionObjectPtr_t CollisionObjectPtr_t;
    typedef model::Configuration_t Configuration_t;
    typedef core::ConfigurationPtr_t ConfigurationPtr_t;
    typedef core::ConfigIterator_t ConfigIterator_t;
    typedef core::ConfigConstIterator_t ConfigConstIterator_t;
    typedef core::ConnectedComponent ConnectedComponent;
    typedef core::ConnectedComponents_t ConnectedComponents_t;
    typedef core::ConnectedComponentPtr_t ConnectedComponentPtr_t;
    typedef model::Device Device_t;
    typedef model::DevicePtr_t DevicePtr_t;
    typedef model::DistanceResult DistanceResult_t;
    typedef model::DistanceResults_t DistanceResults_t;
    typedef core::Edges_t Edges_t;
    typedef model::Joint Joint_t;
    typedef model::JointPtr_t JointPtr_t;
    typedef model::JointVector_t JointVector_t;
    typedef core::LockedDof LockedDof;
    typedef core::LockedDofPtr_t LockedDofPtr_t;
    typedef core::Nodes_t Nodes_t;
    typedef model::ObjectVector_t ObjectVector_t;
    typedef model::ObjectIterator ObjectIterator;
    typedef core::PathPtr_t PathPtr_t;
    typedef core::PathVector PathVector_t;
    typedef core::PathVectorPtr_t PathVectorPtr_t;
    typedef core::SteeringMethod SteeringMethod_t;
    typedef core::SteeringMethodPtr_t SteeringMethodPtr_t;
    typedef model::Transform3f Transform3f;
    typedef fcl::BVHModel < fcl::RSS > Polyhedron_t;
    typedef boost::shared_ptr <Polyhedron_t> PolyhedronPtr_t;
    typedef fcl::ShapeBase BasicShape_t;
    typedef boost::shared_ptr <BasicShape_t> BasicShapePtr_t;

    typedef std::map <std::string, BasicShapePtr_t> ShapeMap_t;
    typedef std::map <std::string, std::vector <fcl::Triangle> > TriangleMap_t;
    typedef std::map <std::string, std::vector <fcl::Vec3f> > VertexMap_t;

    typedef model::vector_t vector_t;
    typedef model::vector3_t vector3_t;
    typedef model::ComJacobian_t ComJacobian_t;
    typedef model::size_type size_type;
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
