/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

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

#include "hppPlanner.h"
#include "hppBody.h"
#include "hppciRobotServer.hh"

class ChppciServer;

/**
 * \brief Implementation of corba interface ChppciRobot.

The construction of a 
 */
class ChppciRobot_impl : public virtual POA_ChppciRobot {
public:
  /// \brief Store pointer to ChppPlanner object of hppciServer.
  ChppciRobot_impl(ChppciServer *inHppciServer);
  /// \brief Comment in interface ChppciRobot::createRobot.
  virtual CORBA::Short createRobot(const char* inRobotName) 
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::addHppProblem.
  virtual CORBA::Short addHppProblem(const char* inRobotName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::setRobotRootJoint.
  virtual CORBA::Short 
    setRobotRootJoint(const char* inRobotName, const char* inJointName)
    throw(CORBA::SystemException);
#if WITH_OPENHRP
  /// \brief Comment in interface ChppciRobot::loadHrp2Model.
  virtual CORBA::Short loadHrp2Model();
#endif
  /// \brief Comment in interface ChppciRobot::createExtraDof.
  virtual CORBA::Short createExtraDof(const char* inDofName, CORBA::Boolean inRevolute, 
				      CORBA::Double inValueMin, CORBA::Double inValueMax)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::addExtraDofToRobot.
  virtual CORBA::Short addExtraDofToRobot(const char* inRobotName, const char* inDofName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::getDeviceDim.
  virtual CORBA::Short getDeviceDim(CORBA::Short problemId, CORBA::Short& deviceDim)
    throw(CORBA::SystemException);

  /// \brief Comment in interface ChppciRobot::createJoint.
  virtual CORBA::Short 
    createJoint(const char* inJointName, const char* inJointType, const matrix4 pos,
		            const jointBoundSeq& jointBound) throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::addJoint.
  virtual CORBA::Short 
    addJoint(const char* inParentName, const char* inChildName)
    throw(CORBA::SystemException);

  /// \brief Comment in interface ChppciRobot::setJointBound.
  virtual CORBA::Short setJointBounds(CORBA::Short problemId, CORBA::Short jointId, 
				     const jointBoundSeq& jointBound)
    throw(CORBA::SystemException);

  /// \brief Comment in interface ChppciRobot::setJointLocked.
  virtual CORBA::Short setJointLocked(CORBA::Short problemId, CORBA::Short jointId, 
				      CORBA::Boolean locked, CORBA::Double lockedValue)
    throw(CORBA::SystemException);

  /// \brief Comment in interface ChppciRobot::setCurrentConfig
  virtual CORBA::Short setCurrentConfig(CORBA::Short inProblemId, const dofSeq& dofArray) 
    throw(CORBA::SystemException);

  /// \brief Comment in interface ChppciRobot::setCurrentConfig in the order of joint of OpenHRP
  virtual CORBA::Short setCurrentConfigOpenHRP(CORBA::Short inProblemId, const dofSeq& dofArray) 
    throw(CORBA::SystemException);

  /// \brief Comment in interface ChppciRobot::getCurrentConfig
  virtual dofSeq* getCurrentConfig(CORBA::Short inProblemId)
    throw(CORBA::SystemException);

  /// \brief Comment in interface ChppciRobot::getCurrentConfig in the order of joint of OpenHRP
  virtual dofSeq* getCurrentConfigOpenHRP(CORBA::Short inProblemId)
    throw(CORBA::SystemException);

  /// \brief Comment in interface ChppciRobot::getCurrentConfig
    /*
  virtual CORBA::Short Test(CORBA::Short inProblemId)
    throw(CORBA::SystemException);
    */

  /// \brief Comment in interface ChppciRobot::attachBodyToJoint.
  virtual CORBA::Short 
    attachBodyToJoint(const char* inJointName, const char* inBodyName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::createBody.
  virtual CORBA::Short createBody(const char* inBodyName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::setBodyInnerObject.
  virtual CORBA::Short 
    setBodyInnerObject(const char* inBodyName, const char* inListName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::getBodyInnerObject.
  virtual nameSeq* getBodyInnerObject(const char* inBodyName);
  /// \brief Comment in interface ChppciRobot::getBodyOuterObject.
  virtual nameSeq* getBodyOuterObject(const char* inBodyName);

  /// \brief Comment in interface ChppciRobot::createCollisionList.
  virtual CORBA::Short createCollisionList(const char* inListName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::addPolyToCollList.
  virtual CORBA::Short 
    addPolyToCollList(const char* inListName, const char* inPolyhedronName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::checkLinkCollision.
  CORBA::Short checkLinkCollision(CORBA::Short problemId, CORBA::Short jointId, CORBA::Short& result)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::createPolyhedron.
  virtual CORBA::Short createPolyhedron(const char* inPolyhedronName)
    throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::addPoint.
  virtual CORBA::Short 
    addPoint(const char* inPolyhedronName, CORBA::Double x, 
	     CORBA::Double y, CORBA::Double z) throw(CORBA::SystemException);
  /// \brief Comment in interface ChppciRobot::addTriangle.
  virtual CORBA::Short 
    addTriangle(const char* inPolyhedronName, long pt1, long pt2, long pt3)
    throw(CORBA::SystemException);
private:
  // Store devices, joints and bodies in construction.
  /// \brief map of devices in construction.
  std::map<std::string, CkppDeviceComponentShPtr> robotMap;
  /// \brief map of extra degrees of freedom in construction.
  std::map<std::string, CkwsDofShPtr> extraDofMap;
  /// \brief map of joints in construction.
  std::map<std::string, CkwsJointShPtr> jointMap;
  /// \brief map of bodies in construction.
  std::map<std::string, ChppBodyShPtr> bodyMap;
  /// \brief map of collision lists in construction.
  std::map<std::string, std::vector<CkcdObjectShPtr> > collisionListMap;
  /// \brief map of polyhedra in construction.
  std::map<std::string, CkppKCDPolyhedronShPtr> polyhedronMap;

  /// \brief Pointer to the ChppciServer owning this object
  ChppciServer* attHppciServer;
  /// \brief Pointer to hppPlanner object of hppciServer.
  /// Instantiated at construction.
  ChppPlanner *attHppPlanner;
};


#endif
