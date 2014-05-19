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
      class Robot : public virtual POA_hpp::corbaserver::Robot
      {
      public:
	Robot (corbaServer::Server* server);

	virtual void
	createRobot (const char* robotName) throw (hpp::Error);

	virtual void
	setRobot(const char* robotName) throw (hpp::Error);

	virtual void
	setRobotRootJoint(const char* robotName, const char* jointName)
	  throw (hpp::Error);

	virtual void loadRobotModel (const char* robotName,
				     const char* rootJointType,
				     const char* packageName,
				     const char* modelName,
				     const char* urdfSuffix,
				     const char* srdfSuffix) throw (hpp::Error);

	virtual void loadHumanoidModel (const char* robotName,
					 const char* rootJointType,
					 const char* packageName,
					 const char* modelName,
					 const char* urdfSuffix,
					 const char* srdfSuffix)
	  throw (hpp::Error);

	virtual Short getConfigSize () throw (hpp::Error);

	virtual Short getNumberDof () throw (hpp::Error);

	virtual void
	createJoint
	(const char* jointName, const char* jointType,
	 const  Double* pos,
	 const hpp::corbaserver::jointBoundSeq& jointBound)
	  throw (hpp::Error);

	virtual void
	addJoint
	(const char* parentName, const char* childName) throw (hpp::Error);

	virtual Names_t* getJointNames () throw (hpp::Error);

	virtual Short getJointNumberDof (const char* jointName)
	  throw (hpp::Error);
	virtual Short getJointConfigSize (const char* jointName)
	  throw (hpp::Error);

	virtual void setJointBounds
	(const char* jointName,
	 const hpp::corbaserver::jointBoundSeq& jointBound) throw (hpp::Error);

	virtual void setCurrentConfig
	(const hpp::floatSeq& dofArray) throw (hpp::Error);

	virtual hpp::floatSeq* getCurrentConfig() throw (hpp::Error);

	virtual Names_t* getJointInnerObjects
	(const char* bodyName)
	  throw (hpp::Error);

	virtual Names_t* getJointOuterObjects
	(const char* bodyName)
	  throw (hpp::Error);

	virtual void
	collisionTest (Boolean& validity) throw (hpp::Error);

	virtual void
	distancesToCollision (hpp::floatSeq_out distances,
			      Names_t_out innerObjects,
			      Names_t_out outerObjects,
			      hpp::floatSeqSeq_out innerPoints,
			      hpp::floatSeqSeq_out outerPoints)
	  throw (hpp::Error);

	virtual Double getMass () throw (hpp::Error);

	virtual hpp::floatSeq* getCenterOfMass () throw (hpp::Error);

	virtual hpp::floatSeqSeq* getJacobianCenterOfMass () throw (hpp::Error);

	virtual void
	createPolyhedron
	(const char* polyhedronName) throw (hpp::Error);

	virtual void
	createBox (const char* name, Double x, Double y, Double z)
	  throw (hpp::Error);

	virtual void
	createSphere (const char* name, Double radius)
	  throw (hpp::Error);

	virtual Short
	addPoint (const char* polyhedronName, Double x, Double y, Double z)
	  throw (hpp::Error);

	virtual Short
	addTriangle
	(const char* polyhedronName, ULong pt1, ULong pt2, ULong pt3)
	  throw (hpp::Error);

	virtual void
	addObjectToJoint (const char* bodyName, const char* objectName,
			  const Double* config)
	  throw (hpp::Error);


	virtual hpp::floatSeq* computeVolume () throw (hpp::Error);

	virtual hpp::floatSeq* parseCapsulePoints () throw (hpp::Error);

	virtual hpp::floatSeq* projectConfigurationUntilIrreducible 
          (const hpp::floatSeq& dofArray) throw (hpp::Error);

        virtual hpp::floatSeq* gradientConfigurationWrtProjection 
          (const hpp::floatSeq& dofArray) throw (hpp::Error);

        virtual hpp::floatSeq* gradientConfigurationWrtProjection () 
                throw (hpp::Error);

        virtual hpp::floatSeq* getRandomConfig() 
                throw (hpp::Error);


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
