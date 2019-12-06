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
# include <hpp/fcl/BVH/BVH_model.h>
// # include <hpp/pinocchio/object-factory.hh>
# include "hpp/core/problem-solver.hh"
# include "hpp/corbaserver/fwd.hh"
# include "hpp/corbaserver/robot-idl.hh" 
# include "hpp/corbaserver/object-map.hh" 

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      using CORBA::Long;
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
	Robot ();

        void setServer (ServerPlugin* server)
        {
          server_ = server;
        }

	virtual void
	createRobot (const char* robotName) throw (hpp::Error);

	virtual void loadRobotModel (const char* robotName,
				     const char* rootJointType,
				     const char* urdfName,
				     const char* srdfName) throw (hpp::Error);

	virtual void loadHumanoidModel (const char* robotName,
					 const char* rootJointType,
					 const char* urdfName,
					 const char* srdfName)
	  throw (hpp::Error);

	virtual void loadRobotModelFromString (
            const char* robotName,
            const char* rootJointType,
            const char* urdfString,
            const char* srdfString) throw (hpp::Error);

	virtual void loadHumanoidModelFromString (
            const char* robotName,
            const char* rootJointType,
            const char* urdfString,
            const char* srdfString) throw (hpp::Error);

	virtual char* getRobotName () throw (hpp::Error);

	virtual Long getConfigSize () throw (hpp::Error);

	virtual Long getNumberDof () throw (hpp::Error);

	virtual void appendJoint
	(const char* parentName, const char* jointName, const char* jointType,
	 const Transform_ pos)
	  throw (hpp::Error);

	virtual Names_t* getJointNames () throw (hpp::Error);
	virtual Names_t* getJointTypes () throw (hpp::Error);
	virtual Names_t* getAllJointNames () throw (hpp::Error);
	virtual Names_t* getChildJointNames (const char* jointName)
          throw (hpp::Error);
	virtual char* getParentJointName (const char* jointName)
          throw (hpp::Error);
        virtual hpp::floatSeq* getJointConfig(const char* jointName)
          throw (hpp::Error);
        virtual void setJointConfig(const char* jointName, const floatSeq& cfg)
          throw (hpp::Error);
        virtual char* getJointType(const char* jointName)
          throw (hpp::Error);
        virtual floatSeq* jointIntegrate(const floatSeq& jointCfg,
            const char* jointName, const floatSeq& dq, bool saturate)
          throw (hpp::Error);
	virtual hpp::floatSeqSeq* getCurrentTransformation(const char* jointName)
	  throw (hpp::Error);
        virtual Transform__slice* getJointPositionInParentFrame(const char* jointName)
          throw (hpp::Error);
	virtual Transform__slice* getJointPosition(const char* jointName)
	  throw (hpp::Error);
	virtual TransformSeq* getJointsPosition (const floatSeq& q, const Names_t& jointNames)
	  throw (hpp::Error);
	virtual floatSeq* getJointVelocity(const char* jointName)
	  throw (hpp::Error);
	virtual floatSeq* getJointVelocityInLocalFrame(const char* jointName)
	  throw (hpp::Error);

	virtual Transform__slice* getRootJointPosition () throw (hpp::Error);

	virtual void setRootJointPosition (const Transform_ position)
	  throw (hpp::Error);

        virtual void setJointPositionInParentFrame (const char* jointName,
            const Transform_ position)
	  throw (hpp::Error);

	virtual Long getJointNumberDof (const char* jointName)
	  throw (hpp::Error);
	virtual Long getJointConfigSize (const char* jointName)
	  throw (hpp::Error);

        virtual hpp::floatSeq* getJointBounds (const char* jointName)
          throw (hpp::Error);
	virtual void setJointBounds
	(const char* jointName,
	 const hpp::floatSeq& jointBound) throw (hpp::Error);

	virtual Transform__slice* getLinkPosition (const char* linkName)
	  throw (hpp::Error);

	virtual TransformSeq* getLinksPosition (const floatSeq& q, const Names_t& linkName)
	  throw (hpp::Error);

	virtual Names_t* getLinkNames(const char* jointName)
	  throw (hpp::Error);

	virtual void setDimensionExtraConfigSpace (ULong dimension)
	  throw (hpp::Error);

  virtual ULong getDimensionExtraConfigSpace ()
    throw (hpp::Error);

	virtual void setExtraConfigSpaceBounds
	(const hpp::floatSeq& bounds) throw (hpp::Error);

	virtual void setCurrentConfig
	(const hpp::floatSeq& dofArray) throw (hpp::Error);

	virtual hpp::floatSeq* shootRandomConfig () throw (hpp::Error);
	virtual hpp::floatSeq* getCurrentConfig() throw (hpp::Error);

	virtual void setCurrentVelocity
	(const hpp::floatSeq& qDot) throw (hpp::Error);
	virtual hpp::floatSeq* getCurrentVelocity() throw (hpp::Error);

	virtual Names_t* getJointInnerObjects
	(const char* bodyName)
	  throw (hpp::Error);

	virtual Names_t* getJointOuterObjects
	(const char* bodyName)
	  throw (hpp::Error);

	virtual void getObjectPosition (const char* objectName, Transform_ cfg)
	  throw (hpp::Error);

	virtual void isConfigValid
	(const hpp::floatSeq& dofArray, Boolean& validity,
	 CORBA::String_out report) throw (hpp::Error);

	virtual void
	distancesToCollision (hpp::floatSeq_out distances,
			      Names_t_out innerObjects,
			      Names_t_out outerObjects,
			      hpp::floatSeqSeq_out innerPoints,
			      hpp::floatSeqSeq_out outerPoints)
	  throw (hpp::Error);

	virtual void
	autocollisionCheck (hpp::boolSeq_out collide)
	  throw (hpp::Error);

	virtual void
	autocollisionPairs (Names_t_out innerObjects,
			Names_t_out outerObjects,
                        boolSeq_out active)
	  throw (hpp::Error);

	virtual void
	setAutoCollision (const char* innerObject,
			  const char* outerObject,
                          bool active)
	  throw (hpp::Error);

	virtual Double getMass () throw (hpp::Error);

	virtual hpp::floatSeq* getCenterOfMass () throw (hpp::Error);

	virtual hpp::floatSeq* getCenterOfMassVelocity () throw (hpp::Error);

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

	virtual void
	createCylinder (const char* name, Double radius, Double length)
	  throw (hpp::Error);

	virtual ULong
	addPoint (const char* polyhedronName, Double x, Double y, Double z)
	  throw (hpp::Error);

	virtual ULong
	addTriangle
	(const char* polyhedronName, ULong pt1, ULong pt2, ULong pt3)
	  throw (hpp::Error);

	virtual void
	addObjectToJoint (const char* jointName,
            const char* objectName, const Transform_ config)
	  throw (hpp::Error);

        virtual void
        addPartialCom (const char* comName, const Names_t& jointNames)
          throw (hpp::Error);

        virtual hpp::floatSeq* getPartialCom (const char* comName)
          throw (hpp::Error);

        virtual hpp::floatSeqSeq* getJacobianPartialCom (const char* comName)
          throw (hpp::Error);

        virtual hpp::floatSeq* getVelocityPartialCom (const char* comName)
          throw (hpp::Error);

        hpp::pinocchio_idl::CenterOfMassComputation_ptr getCenterOfMassComputation (const char* name) throw (Error);

        virtual floatSeq* getRobotAABB() throw (hpp::Error);

      private:
	CollisionObjectConstPtr_t getObjectByName (const char* name);

        ObjectMap objectMap_;

        /// \brief Pointer to the ServerPlugin owning this object.
	ServerPlugin* server_;

        /// \brief Pointer to hpp::core::ProblemSolver object of ServerPlugin.
	core::ProblemSolverPtr_t problemSolver();
      };
    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif
