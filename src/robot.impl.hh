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
# include <KineoKCDModel/kppKCDPolyhedron.h>

# include <hpp/model/body-distance.hh>

# include "hpp/core/planner.hh"

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


      //FIXME: this is so WRONG!

      /// \brief List of kcd objects with shared pointer to the joint
      /// owning the objects.
      class KcdObjectVector : public std::vector<CkcdObjectShPtr> 
      {
      public:
	/**
	   \brief Get joint
	*/
	const CkppJointComponentShPtr& kppJoint() const { 
	  return attKppJoint; 
	};
      
	/**
	   \brief Set joint
	*/
	void kppJoint(const CkppJointComponentShPtr& inKppJoint) {attKppJoint = inKppJoint;};
      
      private:
	CkppJointComponentShPtr attKppJoint;
      };


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
	addHppProblem(const char* robotName, double penetration)
	  throw (SystemException);

	virtual Short 
	setRobotRootJoint(const char* robotName, const char* jointName)
	  throw (SystemException);

	virtual Short loadRobotModel (const char* modelName,
				      double penetration,
				      const char* robotDataDir,
				      const char* urdfSuffix = "",
				      const char* srdfSuffix = "",
				      const char* rcpdfSuffix = "");

	virtual Short
	createExtraDof
	(const char* dofName, Boolean revolute, Double valueMin, Double valueMax)
	  throw (SystemException);

	virtual Short
	setDofBounds
	(UShort problemId, UShort dofId, 	Double minValue, Double maxValue)
	  throw (SystemException);

	virtual Short
	setDofLocked
	(UShort problemId, UShort dofId, Boolean locked, Double lockedValue)
	  throw (SystemException);

	virtual Short
	getDeviceDim
	(UShort problemId, UShort& deviceDim) throw (SystemException);

	virtual Short 
	createJoint
	(const char* jointName, const char* jointType, const  hpp::Configuration& pos,
	 const hpp::jointBoundSeq& jointBound, Boolean display)
	  throw (SystemException);

	virtual Short 
	addJoint
	(const char* parentName, const char* childName) throw (SystemException);

	virtual Short
	setJointBounds
	(UShort problemId, UShort inJointId, const hpp::jointBoundSeq& jointBound)
	  throw (SystemException);

	virtual Short
	setJointVisible
	(UShort problemId, UShort jointId, Boolean visible)
	  throw (SystemException);

	virtual Short
	setJointTransparent
	(UShort problemId, UShort jointId, Boolean isTransparent)
	  throw (SystemException);

	virtual Short
	setJointDisplayPath
	(UShort problemId, UShort jointId, Boolean displayPath)
	  throw (SystemException);

	virtual Short
	setCurrentConfig
	(UShort problemId, const hpp::dofSeq& dofArray) throw (SystemException);

#if WITH_OPENHRP
	virtual Short
	setCurrentConfigOpenHRP
	(UShort problemId, const hpp::dofSeq& dofArray) throw (SystemException);

	virtual hpp::dofSeq*
	getCurrentConfigOpenHRP(UShort problemId) throw (SystemException);
#endif

	virtual hpp::dofSeq*
	getCurrentConfig(UShort problemId) throw (SystemException);

	virtual hpp::nameSeq*
	getJointInnerObject
	(const char* bodyName);

	virtual hpp::nameSeq*
	getJointOuterObject
	(const char* bodyName);

	virtual Short
	setPenetration
	(UShort problemId, Double penetration);

	virtual Short
	getPenetration
	(UShort problemId, Double& penetration);

	virtual Short
	checkLinkCollision
	(UShort problemId, UShort jointId, UShort& result)
	  throw (SystemException);

	virtual Short
	createPolyhedron
	(const char* polyhedronName) throw (SystemException);

	virtual Short
	createBox (const char* inBoxName, Double x, Double y, Double z)
	  throw (SystemException);

	virtual Short 
	addPoint
	(const char* polyhedronName, Double x, Double y, Double z) 
	  throw (SystemException);

	virtual Short 
	addTriangle
	(const char* polyhedronName, ULong pt1, ULong pt2, ULong pt3)
	  throw (SystemException);

	virtual Short
	addPolyToBody
	(const char* bodyName, const char* polyhedronName,
	 const hpp::Configuration& config)
	  throw (SystemException);

      private:
	// Store devices, joints and bodies in construction.

	/// \brief map of devices in construction.
	std::map<std::string, CkppDeviceComponentShPtr> robotMap_;
	/// \brief map of extra degrees of freedom in construction.
	std::map<std::string, CkppExtraDofComponentShPtr> extraDofMap_;
	/// \brief map of joints in construction.
	std::map<std::string, CkppJointComponentShPtr> jointMap_;
	/// \brief map of bodies in construction.
	std::map<std::string, model::BodyDistanceShPtr> bodyMap_;
	/// \brief map of polyhedra in construction.
	std::map<std::string, CkppKCDPolyhedronShPtr> polyhedronMap_;

	/// \brief Pointer to the hpp::corbaServer::Server owning this object
	corbaServer::Server* server_;

	/// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
	///
	/// Instantiated at construction.
	core::Planner* planner_;
      };
    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif
