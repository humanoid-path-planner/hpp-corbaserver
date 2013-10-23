// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <iostream>

#include <KineoModel/kppAnchorJointComponent.h>
#include <KineoModel/kppFreeFlyerJointComponent.h>
#include <KineoModel/kppTranslationJointComponent.h>
#include <KineoModel/kppRotationJointComponent.h>
#include <KineoModel/kppSolidComponentRef.h>
#include <KineoModel/kppExtraDofComponent.h>
#include <KineoKCDModel/kppKCDBox.h>

#include <hpp/util/debug.hh>

#include <hpp/model/fwd.hh>
#include <hpp/model/body-distance.hh>
#include <hpp/model/urdf/util.hh>
#include <hpp/model/joint.hh>

#include "hpp/corbaserver/server.hh"

#include <hpp/kwsio/configuration.hh>
#include "robot.impl.hh"
#include "tools.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      namespace
      {
	static void localSetJointBounds(const CkwsJointShPtr& inKwsJoint,
					const hpp::jointBoundSeq& inJointBound)
	{
	  unsigned int nbJointBounds = (unsigned int)inJointBound.length();
	  unsigned int kwsJointNbDofs = inKwsJoint->countDofs();
	  if (nbJointBounds == 2*kwsJointNbDofs) {
	    for (unsigned int iDof=0; iDof<kwsJointNbDofs; iDof++) {
	      double vMin = inJointBound[2*iDof];
	      double vMax = inJointBound[2*iDof+1];
	      CkwsDofShPtr dof = inKwsJoint->dof(iDof);
	      if (vMin <= vMax) {
		/* Dof is actually bounded */
		dof->isBounded(true);
		dof->bounds(vMin, vMax);
	      }
	      else {
		/* Dof is not bounded */
		dof->isBounded(false);
		dof->bounds(vMax,vMin);
	      }
	    }
	  }
	}
      } // end of namespace.


      Robot::Robot(corbaServer::Server* server) :
	server_(server), planner_(server->planner())
      {
      }



      Short Robot::createRobot(const char* inRobotName)
	throw (SystemException)
      {
	std::string robotName(inRobotName);
	// Check that no robot of this name already exists.
	if (robotMap_.count(robotName) != 0) {
	  hppDout (error, ":createRobot: robot " << robotName << " already exists.");
	  return -1;
	}
	// Try to create a robot.
	CkppDeviceComponentShPtr hppDevice=CkppDeviceComponent::create(robotName);
	if (!hppDevice) {
	  hppDout (error, ":createRobot: failed to create a robot.");
	  return -1;
	}
	// Store robot in map.
	robotMap_[robotName] = hppDevice;
	return 0;
      }

      Short Robot::addHppProblem(const char* inRobotName, double inPenetration)
	throw (SystemException)
      {
	std::string robotName(inRobotName);
	// Check that robot of this name exists.
	if (robotMap_.count(robotName) != 1) {
	  hppDout (error, ":addHppProblem: robot " << robotName << " does not exist.");
	  return -1;
	}
	CkppDeviceComponentShPtr hppDevice = robotMap_[robotName];
	// Create a new problem with this robot.
	planner_->addHppProblem(hppDevice, inPenetration);

	return 0;
      }

      Short Robot::setRobotRootJoint(const char* inRobotName,
				     const char* inJointName)
	throw (SystemException)
      {
	std::string robotName(inRobotName);
	std::string jointName(inJointName);

	// Check that robot of this name exists.
	if (robotMap_.count(robotName) != 1) {
	  hppDout (error, ":setRobotRootJoint: robot " << robotName << " does not exist.");
	  return -1;
	}
	// Check that joint of this name exists.
	if (jointMap_.count(jointName) != 1) {
	  hppDout (error, ":setRobotRootJoint: joint " << jointName << " does not exist.");
	  return -1;
	}
	CkppDeviceComponentShPtr hppDevice = robotMap_[robotName];
	CkppJointComponentShPtr kppJoint = jointMap_[jointName];

	if (hppDevice->rootJointComponent(kppJoint)!=KD_OK) {
	  hppDout (error, ":setRobotRootJoint: failed to set joint "
		   << jointName << " as root joint of robot " << robotName << ".");
	  return -1;
	}
	return 0;
      }

      Short Robot::loadRobotModel(const char* modelName,
				  double penetration,
				  const char* urdfSuffix,
				  const char* srdfSuffix,
				  const char* rcpdfSuffix)
      {
	hpp::model::HumanoidRobotShPtr device;
	bool result;
	try {
	  result =
	    hpp::model::urdf::loadRobotModel (device,
					      std::string (modelName),
					      std::string (urdfSuffix),
					      std::string (srdfSuffix),
					      std::string (rcpdfSuffix));
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
	if (!result) {
	  return -1;
	}

	// Add device to the planner
	if (planner_->addHppProblem (device, penetration) != KD_OK)
	  {
	    hppDout (error, "failed to add robot");
	    return -1;
	  }

	return 0;
      }

      Short Robot::loadHrp2Model(double inPenetration)
      {
	return loadRobotModel("hrp2_14", inPenetration);
      }

      Short Robot::createExtraDof(const char* inDofName, Boolean inRevolute,
				  Double inValueMin, Double inValueMax)
	throw (SystemException)
      {
	std::string dofName(inDofName);
	// Check that extra dof of this name does not already exist.
	if (extraDofMap_.count(dofName) != 0) {
	  hppDout (error, ":createExtraDof: extra degree of freedom " << dofName << " already exists.");
	  return -1;
	}
	CkppExtraDofComponentShPtr extraDof = CkppExtraDofComponent::create(inRevolute, dofName);
	// Check whether creation failed.
	if (!extraDof) {
	  hppDout (error, ":createExtraDof: failed to create extra degree of freedom " << dofName);
	  return -1;
	}
	if (inValueMin <= inValueMax) {
	  extraDof->CkppDofComponent::isBounded(true);
	  extraDof->CkppDofComponent::bounds(inValueMin, inValueMax);
	} else {
	  extraDof->CkppDofComponent::isBounded(false);
	}
	// Store extra degree of freedom in map.
	extraDofMap_[dofName] = extraDof;

	return 0;
      }

      Short Robot::createJoint(const char* inJointName,
			       const char* inJointType, const hpp::Configuration& pos,
			       const hpp::jointBoundSeq& inJointBound,
			       Boolean inDisplay)
	throw (SystemException)
      {
	std::string jointName(inJointName);
	std::string jointType(inJointType);

	// Check that joint of this name does not already exist.
	if (jointMap_.count(jointName) != 0) {
	  hppDout (error, ":createJoint: joint " << jointName  << " already exists.");
	  return -1;
	}

	CkppJointComponentShPtr kppJoint;

	// Fill position matrix
	CkitMat4 posMatrix;
	ConfigurationToCkitMat4(pos, posMatrix);

	hppDout (info, "Position matrix = (( " << posMatrix(0, 0) << ",\t " << posMatrix(0, 1) << ",\t " << posMatrix(0, 2) << ",\t " << posMatrix(0, 3) << " )");
	hppDout (info, "                   ( " << posMatrix(1, 0) << ",\t " << posMatrix(1, 1) << ",\t " << posMatrix(1, 2) << ",\t " << posMatrix(1, 3) << " )");
	hppDout (info, "                   ( " << posMatrix(2, 0) << ",\t " << posMatrix(2, 1) << ",\t " << posMatrix(2, 2) << ",\t " << posMatrix(2, 3) << " )");
	hppDout (info, "                   ( " << posMatrix(3, 0) << ",\t " << posMatrix(3, 1) << ",\t " << posMatrix(3, 2) << ",\t " << posMatrix(3, 3) << " ))");

	// Determine type of joint.
	if (jointType == "anchor") {
	  CkppAnchorJointComponentShPtr kppAnchorJoint
	    = CkppAnchorJointComponent::createWithDefaultBodyFactory
	    (std::string(inJointName));
	  kppAnchorJoint->setCurrentPosition(posMatrix);
	  kppJoint = kppAnchorJoint;
	}
	else if (jointType == "freeflyer") {
	  CkppFreeFlyerJointComponentShPtr kppFreeFlyerJoint
	    = CkppFreeFlyerJointComponent::createWithDefaultBodyFactory
	    (std::string(inJointName));
	  kppFreeFlyerJoint->setCurrentPosition(posMatrix);
	  kppJoint = kppFreeFlyerJoint;
	}
	else if (jointType == "plan") {
	  CkppFreeFlyerJointComponentShPtr kppFreeFlyerJoint
	    = CkppFreeFlyerJointComponent::createWithDefaultBodyFactory
	    (std::string(inJointName));
	  kppFreeFlyerJoint->dof(2)->isLocked(true);
	  kppFreeFlyerJoint->dof(3)->isLocked(true);
	  kppFreeFlyerJoint->dof(4)->isLocked(true);

	  kppFreeFlyerJoint->setCurrentPosition(posMatrix);
	  kppJoint = kppFreeFlyerJoint;
	}
	else if (jointType == "rotation") {
	  CkppRotationJointComponentShPtr kppRotationJoint
	    = CkppRotationJointComponent::createWithDefaultBodyFactory
	    (std::string(inJointName));
	  kppRotationJoint->setCurrentPosition(posMatrix);
	  kppJoint = kppRotationJoint;
	}
	else if (jointType == "translation") {
	  CkppTranslationJointComponentShPtr kppTranslationJoint
	    = CkppTranslationJointComponent::createWithDefaultBodyFactory
	    (std::string(inJointName));
	  kppTranslationJoint->setCurrentPosition(posMatrix);
	  kppJoint = kppTranslationJoint;
	}
	else {
	  hppDout (error, ":createJoint: joint type " << jointType  << " does not exist.");
	  return -1;
	}
	// Check whether creation failed.
	if (!kppJoint) {
	  hppDout (error, ":createJoint: failed to create joint " << jointName);
	  return -1;
	}

	// Set the bounds of the joint
	// Bound joint if needed.
	CkwsJointShPtr kwsJoint = KIT_DYNAMIC_PTR_CAST(CkwsJoint, kppJoint);
	localSetJointBounds(kwsJoint, inJointBound);

	kppJoint->doesDisplayPath(inDisplay);
	kppJoint->isVisible(false);
	// Store joint in jointMap_.
	jointMap_[jointName] = kppJoint;

	return 0;
      }




      Short Robot::addJoint(const char* inParentName,
			    const char* inChildName)
	throw (SystemException)
      {
	// Check that joint of this name exists.
	if (jointMap_.count(inParentName) != 1) {
	  hppDout (error, ":addJoint: joint " << inParentName  << " does not exist.");
	  return -1;
	}
	// Check that joint of this name does not already exist.
	if (jointMap_.count(inChildName) != 1) {
	  hppDout (error, ":addJoint: joint " << inChildName  << " does not exist.");
	  return -1;
	}
	CkppJointComponentShPtr parentJoint = jointMap_[inParentName];
	CkppJointComponentShPtr childJoint = jointMap_[inChildName];
	if (parentJoint->addChildJointComponent(childJoint) != KD_OK) {
	  hppDout (error, ":addJoint: failed to attach joint " << inChildName << " to joint " << inParentName);
	  return -1;
	}
	return 0;
      }


      hpp::nameSeq* Robot::getJointNames (UShort problemId)
      {
	std::size_t rank = static_cast <std::size_t> (problemId);
	hpp::model::DeviceShPtr robot =
	  KIT_DYNAMIC_PTR_CAST (hpp::model::Device,
				planner_->robotIthProblem (rank));
	ULong size = static_cast <ULong> (robot->numberDof ());
	char** nameList = hpp::nameSeq::allocbuf(size);
	hpp::nameSeq *jointNames = new hpp::nameSeq (size, size, nameList);
	CkwsDevice::TJointVector jointVector;
	robot->getJointVector (jointVector);
	std::size_t rankInConfig = 0;
	for (std::size_t i = 0; i < jointVector.size (); ++i) {
	  const CjrlJoint* jrlJoint = KIT_DYNAMIC_PTR_CAST
	    (const hpp::model::Joint, jointVector [i])->jrlJoint ();
	  std::string name = jrlJoint->getName ();
	  std::size_t dimension = jrlJoint->numberDof ();
	  if (dimension == 1) {
	    nameList [rankInConfig] =
	      (char*) malloc (sizeof(char)*(name.length ()+1));
	    strcpy (nameList [rankInConfig], name.c_str ());
	    ++rankInConfig;
	  } else if (dimension > 1) {
	    for (std::size_t j = 0; j < dimension; ++j) {
	      nameList [rankInConfig] =
		(char*) malloc (sizeof(char*)*(name.length ()+4));
	      sprintf (nameList [rankInConfig], "%s_%d", name.c_str (), j);
	      ++rankInConfig;
	    }
	  }
	}
	return jointNames;
      }

      Short Robot::setJointBounds(UShort inProblemId, UShort inJointId,
				  const hpp::jointBoundSeq& inJointBound)
	throw (SystemException)
      {
	unsigned int hppProblemId = (unsigned int)inProblemId;
	unsigned int jointId = (unsigned int)inJointId;

	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr hppRobot = planner_->robotIthProblem(hppProblemId);

	  // get joint
	  CkwsDevice::TJointVector jointVector;
	  hppRobot->getJointVector (jointVector);
	  if (jointId < jointVector.size()) {
	    CkwsJointShPtr kwsJoint = jointVector[jointId];
	    localSetJointBounds(kwsJoint, inJointBound);
	  }
	  else {
	    hppDout (error, ":setJointBounds: jointId="  << jointId  <<
		     " should be smaller than number of joints=" << jointVector.size());
	    return -1;
	  }
	}
	else {
	  hppDout (error, ":setJointBounds: inProblemId="  << hppProblemId  <<
		   " should be smaller than number of problems="
		   << nbProblems);
	  return -1;
	}
	return 0;
      }



      Short Robot::setJointVisible(UShort inProblemId, UShort inJointId,
				   Boolean inVisible)
	throw (SystemException)
      {
	unsigned int hppProblemId = (unsigned int)inProblemId;
	unsigned int jointId = (unsigned int)inJointId;

	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr hppRobot = planner_->robotIthProblem(hppProblemId);
	  CkppJointComponentShPtr kppJoint = hppRobot->jointComponent(jointId);

	  if (kppJoint) {
	    kppJoint->isVisible(inVisible);
	    return 0;
	  }
	}
	return -1;
      }



      Short Robot::setJointTransparent(UShort inProblemId, UShort inJointId,
				       Boolean inTransparent)
	throw (SystemException)
      {
	unsigned int hppProblemId = (unsigned int)inProblemId;
	unsigned int jointId = (unsigned int)inJointId;

	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr hppRobot = planner_->robotIthProblem(hppProblemId);
	  CkppJointComponentShPtr kppJoint = hppRobot->jointComponent(jointId);

	  if (kppJoint) {
	    kppJoint->isTransparent(inTransparent);
	    return 0;
	  }
	}
	return -1;
      }

      Short Robot::setJointDisplayPath(UShort inProblemId, UShort inJointId,
				       Boolean inDisplayPath)
	throw (SystemException)
      {
	unsigned int hppProblemId = (unsigned int)inProblemId;
	unsigned int jointId = (unsigned int)inJointId;

	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr hppRobot = planner_->robotIthProblem(hppProblemId);
	  CkppJointComponentShPtr kppJoint = hppRobot->jointComponent(jointId);

	  if (kppJoint) {
	    kppJoint->doesDisplayPath(inDisplayPath);
	    return 0;
	  }
	}
	return -1;
      }

      Short Robot::setCurrentConfig(UShort inProblemId,
				    const hpp::dofSeq& dofArray)
	throw (SystemException)
      {
	unsigned int hppProblemId = (unsigned int)inProblemId;
	unsigned int configDim = (unsigned int)dofArray.length();

	std::vector<double> dofVector;

	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr hppRobot = planner_->robotIthProblem(hppProblemId);

	  // by Yoshida 06/08/25
	  unsigned int deviceDim = hppRobot->countDofs ();


	  // Fill dof vector with dof array.
	  for (unsigned int iDof=0; iDof<configDim; iDof++) {
	    dofVector.push_back(dofArray[iDof]);
	  }

	  // by Yoshida 06/08/25
	  // fill the vector by zero
	  hppDout (info, "robot id "<<hppProblemId<<", configDim "<<configDim<<",  deviceDim "<<deviceDim);
	  if(configDim != deviceDim){
	    hppDout (error, ":setCurrentConfig: dofVector Does not match");
	    return -1;

	    // dofVector.resize(deviceDim, 0);
	  }

	  // Create a config for robot initialized with dof vector.
	  CkwsConfig config(hppRobot, dofVector);

	  return (short)planner_->robotCurrentConfIthProblem(hppProblemId, config);
	}
	else {
	  hppDout (error, "wrong robot Id=" << hppProblemId
		   << ", nb problems=" << nbProblems);
	  return -1;
	}
	return 0;
      }

#if WITH_OPENHRP

#define LARM_JOINT0_OPENHRP 29
#define RHAND_JOINT0_OPENHRP 36
#define LARM_JOINT0_KINEO 34
#define RHAND_JOINT0_KINEO 29

      /// \brief the config is in the order of OpenHRP Joints  RARM, LARM, RHAND, LHAND
      Short Robot::setCurrentConfigOpenHRP(UShort inProblemId, const hpp::dofSeq& dofArray)
	throw (SystemException)
      {
	hpp::dofSeq dofArrayKineo(dofArray);

	// LARM
	for(unsigned int i=0; i<7; i++){
	  dofArrayKineo[LARM_JOINT0_KINEO+i] = dofArray[LARM_JOINT0_OPENHRP+i];
	}
	// RHAND
	for(unsigned int i=0; i<5; i++){
	  dofArrayKineo[RHAND_JOINT0_KINEO+i] = dofArray[RHAND_JOINT0_OPENHRP+i];
	}

	return setCurrentConfig(inProblemId, dofArrayKineo);
      }

      /// \brief Comment in interface hpp::Robot::getCurrentConfig
      hpp::dofSeq* Robot::getCurrentConfigOpenHRP(UShort inProblemId)
	throw (SystemException)
      {
	hpp::dofSeq *dofArray = getCurrentConfig(inProblemId);

	hpp::dofSeq dofArrayKineo(*dofArray);

	// LARM
	for(unsigned int i=0; i<7; i++){
	  (*dofArray)[LARM_JOINT0_OPENHRP+i] = dofArrayKineo[LARM_JOINT0_KINEO+i];
	}
	// RHAND
	for(unsigned int i=0; i<5; i++){
	  (*dofArray)[RHAND_JOINT0_OPENHRP+i] = dofArrayKineo[RHAND_JOINT0_KINEO+i];
	}

	return dofArray;
      }

#endif

      /// \brief Comment in interface hpp::Robot::getCurrentConfig
      hpp::dofSeq* Robot::getCurrentConfig(UShort inProblemId)
	throw (SystemException)
      {
	unsigned int hppProblemId = (unsigned int)inProblemId;
	hpp::dofSeq *dofArray;

	unsigned int nbProblems = planner_->getNbHppProblems();

	if (hppProblemId < nbProblems)
	  {
	    // Get robot in hppPlanner object.
	    CkppDeviceComponentShPtr hppRobot = planner_->robotIthProblem(hppProblemId);

	    std::vector<double> dofVector;
	    hppRobot->getCurrentDofValues(dofVector);

	    unsigned int deviceDim = hppRobot->countDofs ();

	    dofArray = new hpp::dofSeq();
	    dofArray->length(deviceDim);

	    for(unsigned int i=0; i<deviceDim; i++)
	      (*dofArray)[i] = dofVector[i];

	    return dofArray;
	  }
	else
	  {
	    hppDout (error, "wrong robot Id=" << hppProblemId
		     << ", nb problems=" << nbProblems);
	    dofArray = new hpp::dofSeq (1);

	    return dofArray;
	  }
	return new hpp::dofSeq (1);
      }

      hpp::nameSeq* Robot::getJointInnerObject(const char* inBodyName)
      {
	std::string bodyName(inBodyName);

	hpp::nameSeq *innerObjectSeq = NULL;
	// Find the body corresponding to the name in core::Planner object.
	CkwsKCDBodyAdvancedConstShPtr kcdBody
	  = planner_->findBodyByJointName(bodyName);;

	if (kcdBody) {
	  std::vector<CkcdObjectShPtr> innerObjectList = kcdBody->mobileObjects();
	  if (innerObjectList.size() > 0) {
	    unsigned int nbObjects = innerObjectList.size();
	    // Allocate result now that the size is known.
	    ULong size = (ULong)nbObjects;
	    char** nameList = hpp::nameSeq::allocbuf(size);
	    innerObjectSeq = new hpp::nameSeq(size, size, nameList);

	    for (unsigned int iObject=0; iObject < nbObjects; iObject++) {
	      CkcdObjectShPtr kcdObject = innerObjectList[iObject];
	      // Cast object into CkppKCDPolyhedron.
	      if (CkppGeometryComponentShPtr kppGeometry =
		  KIT_DYNAMIC_PTR_CAST(CkppGeometryComponent, kcdObject)) {
		// Get name of geometry and add it into the list.
		std::string geometryName = kppGeometry->name();
		nameList[iObject] =
		  (char*)malloc(sizeof(char)*(geometryName.length()+1));
		strcpy(nameList[iObject], geometryName.c_str());
	      }
	    }
	  }
	}
	else {
	  hppDout (error, "body of name " << " not found.");
	}

	if (!innerObjectSeq)
	  innerObjectSeq = new hpp::nameSeq (0);

	return innerObjectSeq;
      }

      hpp::nameSeq* Robot::getJointOuterObject(const char* inBodyName)
      {
	std::string bodyName(inBodyName);

	hpp::nameSeq *outerObjectSeq = NULL;
	// Find the body corresponding to the name in core::Planner object.
	CkwsKCDBodyAdvancedConstShPtr kcdBody
	  = planner_->findBodyByJointName(bodyName);

	if (kcdBody) {
	  std::vector<CkcdObjectShPtr> outerObjectList = kcdBody->obstacleObjects();
	  if (outerObjectList.size() > 0) {
	    unsigned int nbObjects = outerObjectList.size();
	    // Allocate result now that the size is known.
	    ULong size = (ULong)nbObjects;
	    char** nameList = hpp::nameSeq::allocbuf(size);
	    outerObjectSeq = new hpp::nameSeq(size, size, nameList);
	    for (unsigned int iObject=0; iObject < nbObjects; iObject++) {
	      CkcdObjectShPtr kcdObject = outerObjectList[iObject];
	      // Cast object into CkppKCDPolyhedron.
	      if (CkppGeometryComponentShPtr kppGeometry =
		  KIT_DYNAMIC_PTR_CAST(CkppGeometryComponent, kcdObject)) {
		// Get name of geometry and add it into the list.
		std::string geometryName = kppGeometry->name();
		nameList[iObject] = (char*)malloc(sizeof(char)*(geometryName.length()+1));
		strcpy(nameList[iObject], geometryName.c_str());
	      }
	    }
	  }
	} else {
	  hppDout (error, "body of name " << " not found.");
	}
	if (!outerObjectSeq) {
	  outerObjectSeq = new hpp::nameSeq(0);
	}
	return outerObjectSeq;
      }

      Short Robot::setPenetration(UShort inProblemId,
				  Double inPenetration)
      {
	unsigned int hppProblemId = (unsigned int)inProblemId;
	unsigned int nbProblems = planner_->getNbHppProblems();

	if (hppProblemId < nbProblems) {
	  planner_->penetration(inProblemId, inPenetration);
	}
	else{
	  hppDout (error, "wrong robot Id=" << hppProblemId
		   << ", nb problems=" << nbProblems);
	  return -1;
	}
	return 0;
      }

      Short Robot::getPenetration(UShort inProblemId,
				  Double& outPenetration)
      {
	unsigned int hppProblemId = (unsigned int)inProblemId;
	unsigned int nbProblems = planner_->getNbHppProblems();

	if (hppProblemId < nbProblems) {
	  outPenetration = planner_->penetration(inProblemId);
	}
	else{
	  hppDout (error, "wrong robot Id=" << hppProblemId
		   << ", nb problems=" << nbProblems);
	  return -1;
	}

	return 0;
      }

      Short Robot::collisionTest
      (UShort problemId, Boolean& validity) throw (SystemException)
      {
	validity = false;
	std::size_t rank = static_cast <std::size_t> (problemId);
	std::size_t nbProblems = static_cast <std::size_t>
	  (planner_->getNbHppProblems ());
	if (rank >= nbProblems) {
	  hppDout (error, "wrong robot Id=" << rank
		   << ", nb problems=" << nbProblems);
	  return -1;
	}
	CkppDeviceComponentShPtr robot = planner_->robotIthProblem (rank);
	validity = robot->collisionTest ();
	return 0;
      }

      Short Robot::isConfigValid (UShort problemId, const hpp::dofSeq& dofArray,
				  Boolean& validity)
	throw (SystemException)
      {
	validity = false;
	std::size_t rank = static_cast <std::size_t> (problemId);
	std::size_t nbProblems = static_cast <std::size_t>
	  (planner_->getNbHppProblems ());
	if (rank >= nbProblems) {
	  hppDout (error, "wrong robot Id=" << rank
		   << ", nb problems=" << nbProblems);
	  return -1;
	}
	CkppDeviceComponentShPtr robot = planner_->robotIthProblem (rank);
	std::size_t dim = robot->countDofs ();

	std::vector<double> dofVector (dim);
	for (std::size_t i=0; i<dim; ++i) {
	  dofVector [i] = dofArray [i];
	}
	CkwsConfig config (robot, dofVector);
	robot->setCurrentConfig (config);
	robot->configValidators()->validate(config);
	validity = config.isValid ();
	if (!validity) {
	  for (std::size_t i=0; i<config.countReports (true); ++i) {
	    std::string validatorName;
	    CkwsValidationReportConstShPtr report = config.report
	      (i, validatorName, true);
	    hppDout (info, "Validator " << validatorName
		   << " has unvalidated the configuration " << config);
	  }
	}
	return 0;
      }

      Short Robot::distancesToCollision (UShort problemId,
					 hpp::dofSeq_out distances,
					 hpp::nameSeq_out bodies,
					 hpp::dofSeqSeq_out bodyPoints,
					 hpp::dofSeqSeq_out obstaclePoints)
      {
	using hpp::model::BodyDistanceShPtr;
	std::size_t rank = static_cast <std::size_t> (problemId);
	std::size_t nbProblems = static_cast <std::size_t>
	  (planner_->getNbHppProblems ());
	if (rank >= nbProblems) {
	  hppDout (error, "wrong robot Id=" << rank
		   << ", nb problems=" << nbProblems);
	  distances = new hpp::dofSeq ();
	  bodies = new hpp::nameSeq ();
	  bodyPoints = new hpp::dofSeqSeq ();
	  obstaclePoints = new hpp::dofSeqSeq ();
	  return -1;
	}
	hpp::model::DeviceShPtr robot
	  (KIT_DYNAMIC_PTR_CAST (hpp::model::Device,
				 planner_->robotIthProblem (rank)));
	if (!robot) {
	  hppDout (error, "Robot is not of type hpp::model::Device");
	  distances = new hpp::dofSeq ();
	  bodies = new hpp::nameSeq ();
	  bodyPoints = new hpp::dofSeqSeq ();
	  obstaclePoints = new hpp::dofSeqSeq ();
	  return -1;
	}
	const std::vector< BodyDistanceShPtr >& bodyDistances
	  (robot->bodyDistances ());
	// count distance computatin pairs
	std::size_t nbDistPairs = 0;
	for (std::vector< BodyDistanceShPtr >::const_iterator it=
	       bodyDistances.begin (); it != bodyDistances.end (); it++) {
	  nbDistPairs += (*it)->nbDistPairs ();
	}
	hpp::dofSeq* distances_ptr = new hpp::dofSeq ();
	distances_ptr->length (nbDistPairs);
	hpp::nameSeq* bodies_ptr = new hpp::nameSeq ();
	bodies_ptr->length (nbDistPairs);
	hpp::dofSeqSeq* bodyPoints_ptr = new hpp::dofSeqSeq ();
	bodyPoints_ptr->length (nbDistPairs);
	hpp::dofSeqSeq* obstaclePoints_ptr = new hpp::dofSeqSeq ();
	obstaclePoints_ptr->length (nbDistPairs);
	// Loop over distance computation pairs
	std::size_t distPairId = 0;
	for (std::vector< BodyDistanceShPtr >::const_iterator it=
	       bodyDistances.begin (); it != bodyDistances.end (); it++) {
	  BodyDistanceShPtr bodyDistance (*it);
	  for (std::size_t i=0; i<bodyDistance->nbDistPairs (); ++i) {
	    double distance;
	    CkcdPoint pointBody;
	    CkcdPoint pointObstacle;
	    if (bodyDistance->distAndPairsOfPoints (i, distance, pointBody,
						    pointObstacle) != KD_OK) {
	      hppDout (error, "Failed to get distance and pairs of points for "
		       << bodyDistance->name () << ", id = " << i);
	      delete distances_ptr;
	      delete bodies_ptr;
	      delete bodyPoints_ptr;
	      delete obstaclePoints_ptr;
	      distances = new hpp::dofSeq ();
	      bodies = new hpp::nameSeq ();
	      bodyPoints = new hpp::dofSeqSeq ();
	      obstaclePoints = new hpp::dofSeqSeq ();
	      return -1;
	    }
	    distances_ptr->operator[] (distPairId) = distance;
	    bodies_ptr->operator[] (distPairId) =
	      bodyDistance->name ().c_str ();
	    hpp::dofSeq pointBody_seq;
	    pointBody_seq.length (3);
	    hpp::dofSeq pointObstacle_seq;
	    pointObstacle_seq.length (3);
	    for (std::size_t j=0; j<3; ++j) {
	      pointBody_seq [j] = pointBody [j];
	      pointObstacle_seq [j] = pointObstacle [j];
	    }
	    bodyPoints_ptr->operator[] (distPairId) = pointBody_seq;
	    obstaclePoints_ptr->operator[] (distPairId) = pointObstacle_seq;
	    ++distPairId;
	  }
	  distances = distances_ptr;
	  bodies = bodies_ptr;
	  bodyPoints = bodyPoints_ptr;
	  obstaclePoints = obstaclePoints_ptr;
	}
	
	
	return 0;
      }

      Short Robot::createPolyhedron(const char* inPolyhedronName)
	throw (SystemException)
      {
	std::string polyhedronName(inPolyhedronName);

	// Check that polyhedron does not already exist.
	if (polyhedronMap_.count(polyhedronName) != 0) {
	  hppDout (error, "polyhedron "	 << polyhedronName << " already exists.");
	  return -1;
	}
	CkppKCDPolyhedronShPtr kppPolyhedron = CkppKCDPolyhedron::create(polyhedronName);

	if (!kppPolyhedron) {
	  hppDout (error, "failed to create polyhedron "	 << polyhedronName);
	  return -1;
	}
	polyhedronMap_[polyhedronName] = kppPolyhedron;
	return 0;
      }


      Short Robot::createBox(const char* inBoxName,
			     Double x, Double y,
			     Double z)
	throw (SystemException)
      {
	//  server_->waitForMutex(std::string("hpp::Obstacle_impl::createBox"));
	std::string polyhedronName(inBoxName);
	// Check that polyhedron does not already exist.
	if (polyhedronMap_.count(polyhedronName) != 0) {
	  hppDout (info, "polyhedron "	 << polyhedronName << " already exists.");
	  return -1;
	}
	CkppKCDPolyhedronShPtr kppPolyhedron = CkppKCDBox::create(polyhedronName, x, y, z);

	if (!kppPolyhedron) {
	  hppDout (info, "failed to create polyhedron "	 << polyhedronName);
	  return -1;
	}
	polyhedronMap_[polyhedronName] = kppPolyhedron;

	return 0;
      }



      Short Robot::addPoint(const char* inPolyhedronName,
			    Double x, Double y,
			    Double z)
	throw (SystemException)
      {
	std::string polyhedronName(inPolyhedronName);

	// Check that polyhedron exists.
	if (polyhedronMap_.count(polyhedronName) != 1) {
	  hppDout (error, "polyhedron " << polyhedronName	 << " does not exist.");
	  return -1;
	}
	CkppKCDPolyhedronShPtr kppPolyhedron = polyhedronMap_[polyhedronName];
	unsigned int rank;
	kppPolyhedron->CkcdPolyhedron::addPoint((kcdReal)x, (kcdReal)y,
						(kcdReal)z, rank);

	return static_cast<Short> (rank);
      }



      Short Robot::addTriangle(const char* inPolyhedronName,
			       ULong pt1, ULong pt2, ULong pt3)
	throw (SystemException)
      {
	std::string polyhedronName(inPolyhedronName);

	// Check that polyhedron exists.
	if (polyhedronMap_.count(polyhedronName) != 1) {
	  hppDout (error, "polyhedron " << polyhedronName	 << " does not exist.");
	  return -1;
	}
	CkppKCDPolyhedronShPtr kppPolyhedron = polyhedronMap_[polyhedronName];
	unsigned int rank;
	/*
	  Test Kineo preconditions
	*/
	if (pt1==pt2 || pt1==pt3 || pt2==pt3 ||
	    pt1 >= kppPolyhedron->countPoints() ||
	    pt2 >= kppPolyhedron->countPoints() ||
	    pt3 >= kppPolyhedron->countPoints()) {
	  hppDout (error, "wrong triangle");
	  return -1;
	}

	kppPolyhedron->addTriangle(pt1, pt2, pt3, rank);

	return static_cast<Short> (rank);
      }



      Short Robot::setDofBounds(UShort inProblemId, UShort inDofId,
				Double inMinValue, Double inMaxValue)
	throw (SystemException)
      {
	unsigned int hppProblemId = (unsigned int)inProblemId;
	unsigned int nbProblems = planner_->getNbHppProblems();
	unsigned int dofId = (unsigned int) inDofId;

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr hppRobot = planner_->robotIthProblem(hppProblemId);

	  unsigned int deviceDim = hppRobot->countDofs();
	  if (dofId < deviceDim) {
	    if (inMinValue <= inMaxValue) {
	      hppRobot->dof(dofId)->isBounded(true);
	      hppRobot->dof(dofId)->bounds(inMinValue, inMaxValue);
	    }
	    else {
	      hppRobot->dof(dofId)->isBounded(false);
	      hppRobot->dof(dofId)->bounds(inMaxValue, inMinValue);
	    }
	  }
	  else {
	    hppDout (error,
		     "dofId = " << dofId
		     << "should be smaller than device dim="
		     << deviceDim);
	    return -1;
	  }
	}
	else{
	  hppDout (error, "wrong robot Id=" << hppProblemId
		   << ", nb problems=" << nbProblems);
	  return -1;
	}
	return 0;
      }



      Short Robot::setDofLocked(UShort inProblemId, UShort inDofId,
				Boolean locked, Double lockedValue)
	throw (SystemException)
      {
	unsigned int hppRobotId = (unsigned int)inProblemId;
	unsigned int nbRobots = planner_->getNbHppProblems();
	unsigned int dofId = (unsigned int) inDofId;

	// Test that rank is less than number of robots in vector.
	if (hppRobotId < nbRobots) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr hppRobot = planner_->robotIthProblem(hppRobotId);

	  // get joint
	  CkwsDevice::TDofVector dofList;
	  hppRobot->getDofVector(dofList);

	  if(dofId >= (unsigned short) dofList.size())
	    {
	      hppDout (error, "joint Id " << dofId
		       << " is larger than total size " << dofList.size());
	      return -1;
	    }

	  dofList[dofId]->isLocked(locked);

	  if(locked)
	    dofList[dofId]->lockedValue(lockedValue);
	}
	else {
	  hppDout (error, "wrong robot Id=" << hppRobotId
		   << ", nb problems=" << nbRobots);
	  return -1;
	}
	return 0;
      }



      Short Robot::getDeviceDim(UShort inProblemId, UShort& outDeviceDim)
	throw (SystemException)
      {
	unsigned int hppProblemId = (unsigned int)inProblemId;

	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr hppRobot = planner_->robotIthProblem(hppProblemId);


	  outDeviceDim = static_cast<UShort> (hppRobot->countDofs ());
	}
	else{
	  hppDout (error, "wrong robot Id=" << hppProblemId
		   << ", nb problems=" << nbProblems);
	  return -1;
	}
	return 0;
      }



      Short Robot::addPolyToBody(const char* inBodyName, const char* inPolyhedronName,
				 const hpp::Configuration& inConfig)
	throw (SystemException)
      {
	std::string bodyName(inBodyName);
	std::string polyhedronName(inPolyhedronName);

	// Check that body of this name exits.
	if (bodyMap_.count(bodyName) != 1) {
	  hppDout (error, "body " << bodyName << " does not exist.");
	  return -1;
	}
	// Check that polyhedron exists.
	if (polyhedronMap_.count(polyhedronName) != 1) {
	  hppDout (error, "polyhedron "	 << polyhedronName << " does not exist.");
	  return -1;
	}

	model::BodyDistanceShPtr hppBody = bodyMap_[bodyName];
	CkppKCDPolyhedronShPtr kppPolyhedron = polyhedronMap_[polyhedronName];

	// Set polyhedron in given configuration.
	CkitMat4 pos;
	ConfigurationToCkitMat4(inConfig, pos);

	if (!hppBody->addInnerObject(CkppSolidComponentRef::create(kppPolyhedron), pos)) {
	  return -1;
	}
	return 0;

      }

    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.
