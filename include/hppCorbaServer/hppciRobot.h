/*
  Copyright 2006 LAAS-CNRS

  Author: Florent Lamiraux

*/

#ifndef HPPCI_ROBOT_H
#define HPPCI_ROBOT_H

/**
Robots and obstacles are stored in object ChppPlanner. 
The kinematic part of a robot is stored in a CkppDeviceComponent object (see KineoWorks documentation). 
\li To each \em joint is attached a \em body (CkwsBody).
\li Each \em body contains a list of CkcdObject (derived into CkppKCDPolyhedron).
\li A \em polyhedron is defined by a set of \em vertices and a set of \em facets.

Obstacles are stored in collision lists (CkcdCollisionList) composed of polyhedra (CkppKCDPolyhedron).

 */

#include <map>
#include <string>

#include "hppCore/hppPlanner.h"
#include "hppModel/hppBody.h"

#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION
#include "hppciRobotServer.hh"

#include "KineoKCDModel/kppKCDPolyhedron.h"

class ChppciServer;

/**
   \brief List of kcd objects with shared pointer to the joint owning the objects.
*/
class ChppKcdObjectVector : public std::vector<CkcdObjectShPtr> 
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


/**
 * \brief Implementation of corba interface ChppciRobot.

The construction of a 
 */
class ChppciRobot_impl : public virtual POA_hppCorbaServer::ChppciRobot {
public:
  /// \brief Store pointer to ChppPlanner object of hppciServer.
  ChppciRobot_impl(ChppciServer *inHppciServer);
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::createRobot.
  virtual CORBA::Short createRobot(const char* inRobotName) 
    throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::addHppProblem.
  virtual CORBA::Short addHppProblem(const char* inRobotName, double inPenetration)
    throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::setRobotRootJoint.
  virtual CORBA::Short 
    setRobotRootJoint(const char* inRobotName, const char* inJointName)
    throw(CORBA::SystemException);
#if WITH_OPENHRP
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::loadHrp2Model.
  virtual CORBA::Short loadHrp2Model(double inPenetration);
#endif
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::createExtraDof.
  virtual CORBA::Short createExtraDof(const char* inDofName, CORBA::Boolean inRevolute, 
				      CORBA::Double inValueMin, CORBA::Double inValueMax)
    throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::addExtraDofToRobot.
  virtual CORBA::Short addExtraDofToRobot(const char* inRobotName, const char* inDofName)
    throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::setDofBound.
  virtual CORBA::Short setDofBounds(CORBA::UShort inProblemId, CORBA::UShort inDofId, 
				    CORBA::Double inMinValue, CORBA::Double inMaxValue)
    throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::setDofLocked.
  virtual CORBA::Short setDofLocked(CORBA::UShort inProblemId, CORBA::UShort inDofId, 
				    CORBA::Boolean locked, CORBA::Double lockedValue)
    throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::getDeviceDim.
  virtual CORBA::Short getDeviceDim(CORBA::UShort inProblemId, CORBA::UShort& outDeviceDim)
    throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::createJoint.
  virtual CORBA::Short 
    createJoint(const char* inJointName, const char* inJointType, const  hppCorbaServer::Configuration& pos,
		const hppCorbaServer::jointBoundSeq& jointBound, CORBA::Boolean inDisplay) throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::addJoint.
  virtual CORBA::Short 
    addJoint(const char* inParentName, const char* inChildName)
    throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::setJointBound.
  virtual CORBA::Short setJointBounds(CORBA::UShort inProblemId, CORBA::UShort inJointId, 
				     const hppCorbaServer::jointBoundSeq& jointBound)
    throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::setJointVisible.
  virtual CORBA::Short setJointVisible(CORBA::UShort inProblemId, CORBA::UShort inJointId, CORBA::Boolean inVisible)
    throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::setJointTransparent.
  virtual CORBA::Short setJointTransparent(CORBA::UShort inProblemId, CORBA::UShort inJointId, CORBA::Boolean isTransparent)
    throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::setJointTransparent.
  virtual CORBA::Short setJointDisplayPath(CORBA::UShort inProblemId, CORBA::UShort inJointId, CORBA::Boolean inDisplayPath)
    throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::setCurrentConfig
  virtual CORBA::Short setCurrentConfig(CORBA::UShort inProblemId, const hppCorbaServer::dofSeq& dofArray) 
    throw(CORBA::SystemException);

#if WITH_OPENHRP
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::setCurrentConfig in the order of joint of OpenHRP
  virtual CORBA::Short setCurrentConfigOpenHRP(CORBA::UShort inProblemId, const hppCorbaServer::dofSeq& dofArray) 
    throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::getCurrentConfig in the order of joint of OpenHRP
  virtual hppCorbaServer::dofSeq* getCurrentConfigOpenHRP(CORBA::UShort inProblemId)
    throw(CORBA::SystemException);
#endif
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::getCurrentConfig
  virtual hppCorbaServer::dofSeq* getCurrentConfig(CORBA::UShort inProblemId)
    throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::getCurrentConfig
    /*
  virtual CORBA::Short Test(CORBA::Short inProblemId)
    throw(CORBA::SystemException);
    */

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::attachBodyToJoint.
  virtual CORBA::Short 
    attachBodyToJoint(const char* inJointName, const char* inBodyName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::createBody.
  virtual CORBA::Short createBody(const char* inBodyName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::getBodyInnerObject.
  virtual hppCorbaServer::nameSeq* getBodyInnerObject(const char* inBodyName);
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::getBodyOuterObject.
  virtual hppCorbaServer::nameSeq* getBodyOuterObject(const char* inBodyName);

  /// \brief Comment in interface hppCorbaServer::ChppciRobot::checkLinkCollision.
  virtual CORBA::Short checkLinkCollision(CORBA::UShort inProblemId, 
					  CORBA::UShort inJointId, CORBA::UShort& outResult)
    throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::createPolyhedron.
  virtual CORBA::Short createPolyhedron(const char* inPolyhedronName)
    throw(CORBA::SystemException);
  /// \brief Comment in ChppciObstacle::createBox.
  virtual CORBA::Short createBox(const char* inBoxName, CORBA::Double x, 
	     CORBA::Double y, CORBA::Double z)
    throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::addPoint.
  virtual CORBA::Short 
    addPoint(const char* inPolyhedronName, CORBA::Double x, 
	     CORBA::Double y, CORBA::Double z) throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::addTriangle.
  virtual CORBA::Short 
    addTriangle(const char* inPolyhedronName, CORBA::ULong pt1, CORBA::ULong pt2, CORBA::ULong pt3)
    throw(CORBA::SystemException);
  /// \brief Comment in interface hppCorbaServer::ChppciRobot::addPolyToBody.
  virtual CORBA::Short
  addPolyToBody(const char* inBodyName, const char* inPolyhedronName, const hppCorbaServer::Configuration& inConfig)
    throw(CORBA::SystemException);
private:
  // Store devices, joints and bodies in construction.
  /// \brief map of devices in construction.
  std::map<std::string, CkppDeviceComponentShPtr> robotMap;
  /// \brief map of extra degrees of freedom in construction.
  std::map<std::string, CkppExtraDofComponentShPtr> extraDofMap;
  /// \brief map of joints in construction.
  std::map<std::string, CkppJointComponentShPtr> jointMap;
  /// \brief map of bodies in construction.
  std::map<std::string, ChppBodyShPtr> bodyMap;
  /// \brief map of polyhedra in construction.
  std::map<std::string, CkppKCDPolyhedronShPtr> polyhedronMap;

  /// \brief Pointer to the ChppciServer owning this object
  ChppciServer* attHppciServer;
  /// \brief Pointer to hppPlanner object of hppciServer.
  /// Instantiated at construction.
  ChppPlanner *attHppPlanner;
};


#endif
