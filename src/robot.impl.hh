// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_ROBOT_IMPL_HH
# define HPP_CORBASERVER_ROBOT_IMPL_HH
# include <map>
# include <string>
# include <fcl/BVH/BVH_model.h>
# include <hpp/model/object-factory.hh>
# include "hpp/core/problem-solver.hh"
# include "hpp/corbaserver/fwd.hh"
# include "robot.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {

      /**
	 Robots and obstacles are stored in object core::Planner.

	 The kinematic part of a robot is stored in a
	 CkppDeviceComponent object (see KineoWorks documentation).

	 \li To each \em joint is attached a \em body (CkwsBody).

	 \li Each \em body contains a list of CkcdObject (derived into
	 CkppKCDPolyhedron).

	 \li A \em polyhedron is defined by a set of \em vertices and a
	 set of \em facets.

	 Obstacles are stored in collision lists (CkcdCollisionList)
	 composed of polyhedra (CkppKCDPolyhedron).
      */


      /// \brief Implementation of corba interface hpp::Robot.
      ///
      /// The construction of a
      class Robot : public virtual POA_hpp::Robot
      {
      public:
	Robot (corbaServer::Server* server);

	virtual Short
	createRobot (const char* robotName) throw (SystemException);

	virtual Short
	setRobot(const char* robotName) throw (SystemException);

	virtual Short
	setRobotRootJoint(const char* robotName, const char* jointName)
	  throw (SystemException);

	virtual Short loadRobotModel (const char* rootJointType,
				      const char* modelName,
				      const char* urdfSuffix,
				      const char* srdfSuffix);

	virtual Short getConfigSize () throw (SystemException);

	virtual Short getNumberDof () throw (SystemException);

	virtual Short
	createJoint
	(const char* jointName, const char* jointType,
	 const  hpp::Configuration& pos, const hpp::jointBoundSeq& jointBound)
	  throw (SystemException);

	virtual Short
	addJoint
	(const char* parentName, const char* childName) throw (SystemException);

	virtual hpp::nameSeq* getJointNames ();

	virtual Short getJointNumberDof (const char* jointName);
	virtual Short getJointConfigSize (const char* jointName);

	virtual Short
	setJointBounds
	(UShort inJointId, const hpp::jointBoundSeq& jointBound)
	  throw (SystemException);

	virtual Short setCurrentConfig
	(const hpp::floatSeq& dofArray) throw (SystemException);

	virtual hpp::floatSeq* getCurrentConfig() throw (SystemException);

	virtual hpp::nameSeq* getJointInnerObjects (const char* bodyName);

	virtual hpp::nameSeq* getJointOuterObjects (const char* bodyName);

	virtual Short
	collisionTest (Boolean& validity) throw (SystemException);

	virtual Short
	distancesToCollision (hpp::floatSeq_out distances,
			      hpp::nameSeq_out innerObjects,
			      hpp::nameSeq_out outerObjects,
			      hpp::floatSeqSeq_out innerPoints,
			      hpp::floatSeqSeq_out outerPoints);

	virtual Double getMass ();

	virtual hpp::floatSeq* getCenterOfMass ();

	virtual hpp::floatSeqSeq* getJacobianCenterOfMass ();

	virtual Short
	createPolyhedron
	(const char* polyhedronName) throw (SystemException);

	virtual Short
	createBox (const char* name, Double x, Double y, Double z)
	  throw (SystemException);

	virtual Short
	createSphere (const char* name, Double radius)
	  throw (SystemException);

	virtual Short
	addPoint (const char* polyhedronName, Double x, Double y, Double z)
	  throw (SystemException);

	virtual Short
	addTriangle
	(const char* polyhedronName, ULong pt1, ULong pt2, ULong pt3)
	  throw (SystemException);

	virtual Short
	addObjectToJoint (const char* bodyName, const char* objectName,
			  const hpp::Configuration& config)
	  throw (SystemException);

      private:
	typedef std::map <std::string, JointPtr_t> JointMap_t;
	// Store devices, joints and bodies in construction.

	/// Map of devices in construction.
	std::map <std::string, DevicePtr_t> robotMap_;
	/// Map of joints in construction.
	JointMap_t jointMap_;

	typedef std::map <std::string, std::vector <fcl::Vec3f> > VertexMap_t;
	typedef std::map <std::string, std::vector <fcl::Triangle> >
	TriangleMap_t;
	typedef std::map <std::string, BasicShapePtr_t> ShapeMap_t;
	/// Map of polyhedra in construction.
	VertexMap_t vertexMap_;
	TriangleMap_t triangleMap_;
	/// Map of basic shapes
	ShapeMap_t shapeMap_;

	/// Pointer to the hpp::corbaServer::Server owning this object
	corbaServer::Server* server_;

	/// Pointer to Planner object of hpp::corbaServer::Server.
	///
	/// Instantiated at construction.
	core::ProblemSolverPtr_t problemSolver_;
	model::ObjectFactory objectFactory_;
      };
    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif
