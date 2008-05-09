/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#include <iostream>

#include "KineoModel/kppAnchorJointComponent.h"
#include "KineoModel/kppFreeFlyerJointComponent.h"
#include "KineoModel/kppTranslationJointComponent.h"
#include "KineoModel/kppRotationJointComponent.h"
#include "KineoModel/kppSolidComponentRef.h"
#include "KineoModel/kppExtraDofComponent.h"
#include "KineoKCDModel/kppKCDBox.h"

#include "hppCorbaServer/hppciServer.h"
#include "hppCorbaServer/hppciRobot.h"
#if WITH_OPENHRP
#include "hppCorbaServer/hppciOpenHrp.h"
#endif

#include "hppCorbaServer/hppciTools.h"

#if DEBUG==2
#define ODEBUG2(x) std::cout << "ChppciRobot:" << x << std::endl
#define ODEBUG1(x) std::cerr << "ChppciRobot:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "ChppciRobot:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

static ktStatus attachSolidComponentsToJoint(const CkppJointComponentShPtr& inKppJoint, 
					     const ChppBodyShPtr& inHppBody)
{
  std::vector<CkcdObjectShPtr> innerObjectVector;
  inHppBody->innerObjects(innerObjectVector);

  for (unsigned int iObj=0; iObj<innerObjectVector.size(); iObj++) {
    CkcdObjectShPtr object = innerObjectVector[iObj];

    CkppKCDPolyhedronShPtr kppPoly = KIT_DYNAMIC_PTR_CAST(CkppKCDPolyhedron, object);
    if (!kppPoly) {
      ODEBUG1(":attachSolidComponentsToJoint: Object is not a CkppKCDPolyhedron.");
      return KD_ERROR;
    }
    inKppJoint->addSolidComponentRef(CkppSolidComponentRef::create(kppPoly)); 
  }
  return KD_OK;
}

// ==========================================================================

ChppciRobot_impl::ChppciRobot_impl(ChppciServer *inHppciServer) : 
  attHppciServer(inHppciServer), attHppPlanner(inHppciServer->getHppPlanner())
{
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::createRobot(const char* inRobotName) 
  throw(CORBA::SystemException)
{
  std::string robotName(inRobotName);
  // Check that no robot of this name already exists.
  if (robotMap.count(robotName) != 0) {
    ODEBUG1(":createRobot: robot " << robotName << " already exists.");
    return -1;
  }
  // Try to create a robot.
  CkppDeviceComponentShPtr hppDevice=CkppDeviceComponent::create(robotName);
  if (!hppDevice) {
    ODEBUG1(":createRobot: failed to create a robot.");
    return -1;
  }
  // Store robot in map.
  robotMap[robotName] = hppDevice;
  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::addHppProblem(const char* inRobotName)
  throw(CORBA::SystemException)
{
  std::string robotName(inRobotName);
  // Check that robot of this name exists.
  if (robotMap.count(robotName) != 1) {
    ODEBUG1(":addHppProblem: robot " << robotName << " does not exist.");
    return -1;
  }
  CkppDeviceComponentShPtr hppDevice = robotMap[robotName];
  // Create a new problem with this robot.
  attHppPlanner->addHppProblem(hppDevice);
  
  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::setRobotRootJoint(const char* inRobotName, 
						 const char* inJointName)
  throw(CORBA::SystemException)
{
  std::string robotName(inRobotName);
  std::string jointName(inJointName);
    
  // Check that robot of this name exists.
  if (robotMap.count(robotName) != 1) {
    ODEBUG1(":setRobotRootJoint: robot " << robotName << " does not exist.");
    return -1;
  }
  // Check that joint of this name exists.
  if (jointMap.count(jointName) != 1) {
    ODEBUG1(":setRobotRootJoint: joint " << jointName << " does not exist.");
    return -1;
  }
  CkppDeviceComponentShPtr hppDevice = robotMap[robotName];
  CkppJointComponentShPtr kppJoint = jointMap[jointName];
  
  if (hppDevice->rootJointComponent(kppJoint)!=KD_OK) {
    ODEBUG1(":setRobotRootJoint: failed to set joint "
	    << jointName << " as root joint of robot " << robotName << ".");
    return -1;
  }
  return 0;
}

#if WITH_OPENHRP
CORBA::Short ChppciRobot_impl::loadHrp2Model()
{
  ChppciOpenHrpClient openHrpClient(attHppPlanner);
  if (openHrpClient.loadHrp2Model() != KD_OK) {
    return -1;
  }
  return 0;
}
#endif

// ==========================================================================

CORBA::Short ChppciRobot_impl::createExtraDof(const char* inDofName, CORBA::Boolean inRevolute, 
					      CORBA::Double inValueMin, CORBA::Double inValueMax)
  throw(CORBA::SystemException)
{
  std::string dofName(inDofName);
  // Check that extra dof of this name does not already exist.
  if (extraDofMap.count(dofName) != 0) {
    ODEBUG1(":createExtraDof: extra degree of freedom " << dofName << " already exists.");
    return -1;
  }
  CkppExtraDofComponentShPtr extraDof = CkppExtraDofComponent::create(inRevolute, dofName);
  // Check whether creation failed.
  if (!extraDof) {
    ODEBUG1(":createExtraDof: failed to create extra degree of freedom " << dofName);
    return -1;
  }
  if (inValueMin <= inValueMax) {
    extraDof->CkppDofComponent::isBounded(true);
    extraDof->CkppDofComponent::bounds(inValueMin, inValueMax);
  } else {
    extraDof->CkppDofComponent::isBounded(false);
  }
  // Store extra degree of freedom in map.
  extraDofMap[dofName] = extraDof;

  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::addExtraDofToRobot(const char* inRobotName, const char* inDofName)
  throw(CORBA::SystemException)
{
  std::string robotName(inRobotName);
  std::string dofName(inDofName);

  // Check that robot of this name exists.
  if (robotMap.count(robotName) != 1) {
    ODEBUG1(":addExtraDofToRobot: robot " << robotName << " does not exist.");
    return -1;
  }
  // Check that extra degree of freedom of this name exists.
  if (extraDofMap.count(dofName) != 1) {
    ODEBUG1(":addExtraDofToRobot: joint " << dofName << " does not exist.");
    return -1;
  }
  CkppDeviceComponentShPtr hppDevice = robotMap[robotName];
  CkppExtraDofComponentShPtr kwsExtraDof = extraDofMap[dofName];
  
  if (hppDevice->addExtraDof(kwsExtraDof)!=KD_OK) {
    ODEBUG1(":addExtraDofToRobot: failed add extra degree of freedom "	 
	    << dofName << " to robot " << robotName << ".");
    return -1;
  }
  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::createJoint(const char* inJointName, 
					   const char* inJointType, const hppCorbaServer::Configuration& pos, 
					   const hppCorbaServer::jointBoundSeq& inJointBound,
					   CORBA::Boolean inDisplay) 
  throw(CORBA::SystemException)
{
  std::string jointName(inJointName);
  std::string jointType(inJointType);

  // Check that joint of this name does not already exist.
  if (jointMap.count(jointName) != 0) {
    ODEBUG1(":createJoint: joint " << jointName  << " already exists.");
    return -1;
  }

  CkppJointComponentShPtr kppJoint;

  // Fill position matrix
  CkitMat4 posMatrix;
  ConfigurationToCkitMat4(pos, posMatrix);

  ODEBUG2("Position matrix = (( " << posMatrix(0, 0) << ",\t " << posMatrix(0, 1) << ",\t " << posMatrix(0, 2) << ",\t " << posMatrix(0, 3) << " )");
  ODEBUG2("                   ( " << posMatrix(1, 0) << ",\t " << posMatrix(1, 1) << ",\t " << posMatrix(1, 2) << ",\t " << posMatrix(1, 3) << " )");
  ODEBUG2("                   ( " << posMatrix(2, 0) << ",\t " << posMatrix(2, 1) << ",\t " << posMatrix(2, 2) << ",\t " << posMatrix(2, 3) << " )");
  ODEBUG2("                   ( " << posMatrix(3, 0) << ",\t " << posMatrix(3, 1) << ",\t " << posMatrix(3, 2) << ",\t " << posMatrix(3, 3) << " ))");

  // Determine type of joint.
  if (jointType == "anchor") {
    CkppAnchorJointComponentShPtr kppAnchorJoint = CkppAnchorJointComponent::create(std::string(inJointName));
    kppAnchorJoint->setCurrentPosition(posMatrix);
    kppJoint = kppAnchorJoint;
  }
  else if (jointType == "freeflyer") {
    CkppFreeFlyerJointComponentShPtr kppFreeFlyerJoint = CkppFreeFlyerJointComponent::create(std::string(inJointName));
    kppFreeFlyerJoint->setCurrentPosition(posMatrix);
    kppJoint = kppFreeFlyerJoint;
  }
  else if (jointType == "plan") {
    CkppFreeFlyerJointComponentShPtr kppFreeFlyerJoint = CkppFreeFlyerJointComponent::create(std::string(inJointName));
    kppFreeFlyerJoint->dof(2)->isLocked(true);
    kppFreeFlyerJoint->dof(3)->isLocked(true);
    kppFreeFlyerJoint->dof(4)->isLocked(true);

    kppFreeFlyerJoint->setCurrentPosition(posMatrix);
    kppJoint = kppFreeFlyerJoint;
  }
  else if (jointType == "rotation") {
    CkppRotationJointComponentShPtr kppRotationJoint = CkppRotationJointComponent::create(std::string(inJointName));
    kppRotationJoint->setCurrentPosition(posMatrix);
    kppJoint = kppRotationJoint;
  }
  else if (jointType == "translation") {
    CkppTranslationJointComponentShPtr kppTranslationJoint = CkppTranslationJointComponent::create(std::string(inJointName));
    kppTranslationJoint->setCurrentPosition(posMatrix);
    kppJoint = kppTranslationJoint;
  }
  else {
    ODEBUG1(":createJoint: joint type " << jointType  << " does not exist.");
    return -1;
  }
  // Check whether creation failed.
  if (!kppJoint) {
    ODEBUG1(":createJoint: failed to create joint " << jointName);
    return -1;
  } 
  // Bound joint if needed.
  CkwsJointShPtr kwsJoint = KIT_DYNAMIC_PTR_CAST(CkwsJoint, kppJoint);
  unsigned int nbJointBounds = (unsigned int)inJointBound.length();
  unsigned int kwsJointNbDofs = kwsJoint->countDofs(); 
  if (nbJointBounds == 2*kwsJointNbDofs) {
    for (unsigned int iDof=0; iDof<kwsJointNbDofs; iDof++) {
      double vMin = inJointBound[2*iDof];
      double vMax = inJointBound[2*iDof+1];
      CkwsJointDofShPtr dof = kwsJoint->dof(iDof);
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
  kppJoint->doesDisplayPath(inDisplay);
  kppJoint->isVisible(false);
  // Store joint in jointMap.
  jointMap[jointName] = kppJoint;

  return 0;
}

// ==========================================================================


CORBA::Short ChppciRobot_impl::addJoint(const char* inParentName, 
					const char* inChildName)
  throw(CORBA::SystemException)
{
  // Check that joint of this name exists.
  if (jointMap.count(inParentName) != 1) {
    ODEBUG1(":addJoint: joint " << inParentName  << " does not exist.");
    return -1;
  }
  // Check that joint of this name does not already exist.
  if (jointMap.count(inChildName) != 1) {
    ODEBUG1(":addJoint: joint " << inChildName  << " does not exist.");
    return -1;
  }
  CkppJointComponentShPtr parentJoint = jointMap[inParentName];
  CkppJointComponentShPtr childJoint = jointMap[inChildName];
  if (parentJoint->addChildJointComponent(childJoint) != KD_OK) {
    ODEBUG1(":addJoint: failed to attach joint " << inChildName << " to joint " << inParentName);
    return -1;
  }
  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::setJointBounds(CORBA::Short inProblemId, CORBA::Short inJointId, 
					      const hppCorbaServer::jointBoundSeq& inJointBound)
  throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int jointId = (unsigned int)inJointId;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

    // get joint
    CkwsDevice::TJointVector jointVector;
    hppRobot->getJointVector (jointVector);
    if (jointId < jointVector.size()) {
      CkwsJointShPtr kwsJoint = jointVector[jointId];
      
      // Bound dofs if needed.
      unsigned int nbJointBounds = (unsigned int)inJointBound.length();
      unsigned int kwsJointNbDofs = kwsJoint->countDofs(); 
      if (nbJointBounds == 2*kwsJointNbDofs) {
	for (unsigned int iDof=0; iDof<kwsJointNbDofs; iDof++) {
	  double vMin = inJointBound[2*iDof];
	  double vMax = inJointBound[2*iDof+1];
	  if (vMin <= vMax) {
	    kwsJoint->dof(iDof)->isBounded(true);
	    kwsJoint->dof(iDof)->bounds(vMin, vMax);
	  }
	  else {
	    kwsJoint->dof(iDof)->isBounded(false);
	  }
	}
      }
      else{
	ODEBUG1(":setJointBounds: nbJointBounds "<< nbJointBounds <<
		" != kwsJointNbDofs "<<kwsJointNbDofs<<" x 2");
	return -1;
      }

    }
    else {
      ODEBUG1(":setJointBounds: jointId="  << jointId  << 
	      " should be smaller than number of joints=" << jointVector.size());
      return -1;
    }
  }
  else {
    ODEBUG1(":setJointBounds: inProblemId="  << hppProblemId  << 
	    " should be smaller than number of problems="
	    << nbProblems);
    return -1;
  }
  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::setJointVisible(CORBA::Short inProblemId, CORBA::Short inJointId, 
					       CORBA::Boolean inVisible)
  throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int jointId = (unsigned int)inJointId;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);
    CkppJointComponentShPtr kppJoint = hppRobot->jointComponent(jointId);

    if (kppJoint) {
      kppJoint->isVisible(inVisible);
      return 0;
    }
  }
  return -1;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::setJointTransparent(CORBA::Short inProblemId, CORBA::Short inJointId, 
						   CORBA::Boolean inTransparent)
  throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int jointId = (unsigned int)inJointId;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);
    CkppJointComponentShPtr kppJoint = hppRobot->jointComponent(jointId);

    if (kppJoint) {
      kppJoint->isTransparent(inTransparent);
      return 0;
    }
  }
  return -1;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::setJointDisplayPath(CORBA::Short inProblemId, CORBA::Short inJointId, 
						   CORBA::Boolean inDisplayPath)
  throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int jointId = (unsigned int)inJointId;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);
    CkppJointComponentShPtr kppJoint = hppRobot->jointComponent(jointId);

    if (kppJoint) {
      kppJoint->doesDisplayPath(inDisplayPath);
      return 0;
    }
  }
  return -1;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::setDofLocked(CORBA::Short inProblemId, CORBA::Short dofId, 
					    CORBA::Boolean locked, CORBA::Double lockedValue)
  throw(CORBA::SystemException)
{
  unsigned int hppRobotId = (unsigned int)inProblemId;

  unsigned int nbRobots = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppRobotId < nbRobots) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppRobotId);

    // get joint
    CkwsDevice::TDofVector dofList;
    hppRobot->getDofVector(dofList);

    if(dofId >= (unsigned short) dofList.size()){
      ODEBUG1(":setJointBound: joint Id " << dofId << " is larger than total size " << dofList.size());
      return -1;
    }

    dofList[dofId]->isLocked(locked);

    if(locked)
      dofList[dofId]->lockedValue(lockedValue);
  }
  else {
    ODEBUG1("ChppciRobotConfig_impl::setJointBound: wrong robot Id");
    return -1;
  }
  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::setCurrentConfig(CORBA::Short inProblemId, 
						const hppCorbaServer::dofSeq& dofArray) 
  throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int configDim = (unsigned int)dofArray.length();

  std::vector<double> dofVector;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

    // by Yoshida 06/08/25
    unsigned int deviceDim = hppRobot->countDofs ();


    // Fill dof vector with dof array.
    for (unsigned int iDof=0; iDof<configDim; iDof++) {
      dofVector.push_back(dofArray[iDof]);
    }
    
    // by Yoshida 06/08/25
    // fill the vector by zero
    ODEBUG2("robot id "<<hppProblemId<<", configDim "<<configDim<<",  deviceDim "<<deviceDim);
    if(configDim != deviceDim){
      ODEBUG1(":setCurrentConfig: dofVector Does not match");
      return -1;

      // dofVector.resize(deviceDim, 0);
    }

    // Create a config for robot initialized with dof vector.
    CkwsConfig config(hppRobot, dofVector);
    
    return (short)attHppPlanner->robotCurrentConfIthProblem(hppProblemId, config);
  }
  else {
    ODEBUG1(":setCurrentConfig: wrong robot Id");
    return -1;
  }
  return 0;
}

// ==========================================================================

#if WITH_OPENHRP

#define LARM_JOINT0_OPENHRP 29
#define RHAND_JOINT0_OPENHRP 36
#define LARM_JOINT0_KINEO 34
#define RHAND_JOINT0_KINEO 29

/// \brief the config is in the order of OpenHRP Joints  RARM, LARM, RHAND, LHAND
CORBA::Short ChppciRobot_impl::setCurrentConfigOpenHRP(CORBA::Short inProblemId, const hppCorbaServer::dofSeq& dofArray)
    throw(CORBA::SystemException)
{
  hppCorbaServer::dofSeq dofArrayKineo(dofArray);

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

/// \brief Comment in interface ChppciRobot::getCurrentConfig
hppCorbaServer::dofSeq* ChppciRobot_impl::getCurrentConfigOpenHRP(CORBA::Short inProblemId)
    throw(CORBA::SystemException)
{
  hppCorbaServer::dofSeq *dofArray = getCurrentConfig(inProblemId);
  
  hppCorbaServer::dofSeq dofArrayKineo(*dofArray);

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

/// \brief Comment in interface ChppciRobot::getCurrentConfig
hppCorbaServer::dofSeq* ChppciRobot_impl::getCurrentConfig(CORBA::Short inProblemId)
    throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;

  hppCorbaServer::dofSeq *dofArray;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

    std::vector<double> dofVector;
    hppRobot->getCurrentDofValues(dofVector);

    // by Yoshida 06/08/25
    unsigned int deviceDim = hppRobot->countDofs ();

    // cerr<<"deviceDim "<<deviceDim<<" [ ";
    dofArray = new hppCorbaServer::dofSeq();
    dofArray->length(deviceDim);

    for(unsigned int i=0; i<deviceDim; i++){
      (*dofArray)[i] = dofVector[i];
      // cerr<<" "<<(*dofArray)[i];
    }
    // cerr<<" ] "<<endl;

    return dofArray;
  }

  else {
    ODEBUG1(":CurrentConfig: wrong robot Id");
    dofArray = new hppCorbaServer::dofSeq(1);

    return dofArray;
  }

  return new hppCorbaServer::dofSeq(1);
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::attachBodyToJoint(const char* inJointName, 
						 const char* inBodyName)
  throw(CORBA::SystemException)
{
  std::string jointName(inJointName);
  std::string bodyName(inBodyName);

  // Check that joint of this name exists.
  if (jointMap.count(jointName) != 1) {
    ODEBUG1(":addJoint: joint " << jointName 	 << " does not exist.");
    return -1;
  }
  // Check that body of this name exits.
  if (bodyMap.count(bodyName) != 1) {
    ODEBUG1(":attachBodyToJoint: body "	 << bodyName << " does not exist.");
    return -1;
  }
  CkppJointComponentShPtr kppJoint = jointMap[jointName];
  CkwsJointShPtr kwsJoint = KIT_DYNAMIC_PTR_CAST(CkwsJoint, kppJoint);
  ChppBodyShPtr hppBody = bodyMap[bodyName];

  if (kwsJoint->setAttachedBody(hppBody) != KD_OK) {
    ODEBUG1(":attachBodyToJoint: failed to attach body "	 << bodyName << " to joint " << jointName);
    return -1;
  }
  // If objects are attached to the body, the corresponding component need to
  // be attached to the joint component
  attachSolidComponentsToJoint(kppJoint, hppBody);

  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::createBody(const char* inBodyName)
  throw(CORBA::SystemException)
{
  std::string bodyName(inBodyName);

  // Check that body does not already exist.
  if (bodyMap.count(bodyName) != 0) {
    ODEBUG1(":createBody: body " << bodyName 	 << " already exists.");
    return -1;
  }
  ChppBodyShPtr hppBody = ChppBody::create(bodyName);
  
  if (!hppBody) {
    ODEBUG1(":createBody: failed to create body "	 << bodyName << ".");
    return -1;
  }
  // Attach an empty outer object list.
  std::vector<CkcdObjectShPtr> emptyObjectList ;
  hppBody->setOuterObjects(emptyObjectList);
  // Store body in map.
  bodyMap[bodyName] = hppBody;

  return 0;
}

// ==========================================================================

hppCorbaServer::nameSeq* ChppciRobot_impl::getBodyInnerObject(const char* inBodyName)
{
  std::string bodyName(inBodyName);

  hppCorbaServer::nameSeq *innerObjectSeq = NULL;
  // Find the body corresponding to the name in ChppPlanner object.
  ChppBodyConstShPtr hppBody = attHppPlanner->findBodyByName(bodyName);
  CkwsKCDBodyConstShPtr kcdBody;

  if (hppBody) {
    std::vector<CkcdObjectShPtr> innerObjectList = hppBody->innerObjects();
    if (innerObjectList.size() > 0) {
      unsigned int nbObjects = innerObjectList.size();
      // Allocate result now that the size is known.
      CORBA::ULong size = (CORBA::ULong)nbObjects;
      char** nameList = hppCorbaServer::nameSeq::allocbuf(size);
      innerObjectSeq = new hppCorbaServer::nameSeq(size, size, nameList);

      for (unsigned int iObject=0; iObject < nbObjects; iObject++) {
	CkcdObjectShPtr kcdObject = innerObjectList[iObject];
	// Cast object into CkppKCDPolyhedron.
	if (CkppKCDPolyhedronShPtr kppPolyhedron = boost::dynamic_pointer_cast<CkppKCDPolyhedron>(kcdObject)) {
	  // Get name of polyhedron and add it into the list.
	  std::string polyhedronName = kppPolyhedron->name();
	  nameList[iObject] = (char*)malloc(sizeof(char)*(polyhedronName.length()+1));
	  strcpy(nameList[iObject], polyhedronName.c_str());
	}
      }
    }
  } else {
    ODEBUG1(":getBodyInnerObject: body of name " << " not found.");
  }
  if (!innerObjectSeq) {
    innerObjectSeq = new hppCorbaServer::nameSeq(0);
  }

  return innerObjectSeq;
}

// ==========================================================================

hppCorbaServer::nameSeq* ChppciRobot_impl::getBodyOuterObject(const char* inBodyName)
{
  std::string bodyName(inBodyName);

  hppCorbaServer::nameSeq *outerObjectSeq = NULL;
  // Find the body corresponding to the name in ChppPlanner object.
  ChppBodyConstShPtr hppBody = attHppPlanner->findBodyByName(bodyName);
  CkwsKCDBodyConstShPtr kcdBody;

  if (hppBody) {
    std::vector<CkcdObjectShPtr> outerObjectList = hppBody->outerObjects();
    if (outerObjectList.size() > 0) {
      unsigned int nbObjects = outerObjectList.size();
      // Allocate result now that the size is known.
      CORBA::ULong size = (CORBA::ULong)nbObjects;
      char** nameList = hppCorbaServer::nameSeq::allocbuf(size);
      outerObjectSeq = new hppCorbaServer::nameSeq(size, size, nameList);
      for (unsigned int iObject=0; iObject < nbObjects; iObject++) {
	CkcdObjectShPtr kcdObject = outerObjectList[iObject];
	// Cast object into CkppKCDPolyhedron.
	if (CkppKCDPolyhedronShPtr kppPolyhedron = boost::dynamic_pointer_cast<CkppKCDPolyhedron>(kcdObject)) {
	  // Get name of polyhedron and add it into the list.
	  std::string polyhedronName = kppPolyhedron->name();
	  nameList[iObject] = (char*)malloc(sizeof(char)*(polyhedronName.length()+1));
	  strcpy(nameList[iObject], polyhedronName.c_str());
	}
      }
    }
  } else {
    ODEBUG1(":getBodyInnerObject: body of name " << " not found.");
  }
  if (!outerObjectSeq) {
    outerObjectSeq = new hppCorbaServer::nameSeq(0);
  }

  return outerObjectSeq;
}

#define DOF_SHIFT 7 // DOF shift. extraDof + 6 dof of the free-flying base.

// ==========================================================================

CORBA::Short 
ChppciRobot_impl::checkLinkCollision(CORBA::Short inProblemId, CORBA::Short jointId, 
				     CORBA::Short& result) 
throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int hppJointId = (unsigned int)jointId; // +DOF_SHIFT;
  
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();


  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

    // by Yoshida 06/08/25
    unsigned int deviceDim = hppRobot->countDofs ();

    if (hppJointId > deviceDim) {
      ODEBUG1(":checkLinkCollision: wrong joint Id");
      return -1;
    }

    // get joint
    CkwsDevice::TJointVector jointList;
    hppRobot->getJointVector(jointList);
    // get object
    ChppBodyShPtr hppBody = KIT_DYNAMIC_PTR_CAST(ChppBody, jointList[hppJointId]->attachedBody());

    // get result
    result = (CORBA::Short) hppBody->computeEstimatedDistance();

    // debug
    // cout<<"colliding bodies: ";
    ODEBUG2(=====================debugging collision detection ======================);

    for(unsigned int i=0; i<jointList.size(); i++){

      ChppBodyShPtr hppBody = KIT_DYNAMIC_PTR_CAST(ChppBody, jointList[i]->attachedBody());

      CkitMat4 mat = hppBody->absolutePosition();
      CkitMat4 matJoint = jointList[i]->currentPosition();
      CkitVect3 trans = mat.translation();
      CkitVect3 transJoint = matJoint.translation();
      double dist=0;
      if (hppBody->CkwsKCDBody::getEstimatedDistance(dist) == KD_ERROR) {
	ODEBUG1(":checkLinkCollision: failure in getting estimated distance");
	return -1;
      }
      ODEBUG2("for joint "<<hppBody->name()<<" body pos "<<trans[0]<<", "<<trans[1]<<", "<<trans[2]
	      <<" distance "<<dist<<);
      ODEBUG2("for joint "<<hppBody->name()<<" joint pos "<<transJoint[0]<<", "<<transJoint[1]<<", "<<transJoint[2]);
#if DEBUG==2
      std::cout << " joint config:";
      for(unsigned int j=0; j<3; j++){
	for(unsigned int k=0; k<3; k++)
	  cout<<matJoint(j,k)<<" ";
	cout<<" ";
      }
      cout<<endl;
#endif
      hppBody->printCollisionStatus();
      // hppBody->printCollisionStatusFast();
      short res = (CORBA::Short) hppBody->computeEstimatedDistance();

      if(res == 0){
	ODEBUG2("Collision");

      }
    }

    cout<<endl;


  }
  else{
    ODEBUG1(":checkLinkCollision: wrong robot Id");
    return -1;
  }

  return 0;
}
 
// ==========================================================================

CORBA::Short ChppciRobot_impl::createPolyhedron(const char* inPolyhedronName)
  throw(CORBA::SystemException)
{
  std::string polyhedronName(inPolyhedronName);

  // Check that polyhedron does not already exist.
  if (polyhedronMap.count(polyhedronName) != 0) {
    ODEBUG1(":createPolyhedron: polyhedron "	 << polyhedronName << " already exists.");
    return -1;
  }
  CkppKCDPolyhedronShPtr kppPolyhedron = CkppKCDPolyhedron::create(polyhedronName);

  if (!kppPolyhedron) {
    ODEBUG1(":createPolyhedron: failed to create polyhedron "	 << polyhedronName);
    return -1;
  }
  polyhedronMap[polyhedronName] = kppPolyhedron;

  return 0;
}

// ==========================================================================
 
CORBA::Short ChppciRobot_impl::createBox(const char* inBoxName, 
					 CORBA::Double x, CORBA::Double y, 
					 CORBA::Double z)
  throw(CORBA::SystemException)
{
  //  attHppciServer->waitForMutex(std::string("ChppciObstacle_impl::createBox"));
  std::string polyhedronName(inBoxName);
  // Check that polyhedron does not already exist.
  if (polyhedronMap.count(polyhedronName) != 0) {
    ODEBUG2("ChppciObstacle_impl::createPolyhedron: polyhedron "	 << polyhedronName << " already exists.");
    return -1;
  }
  CkppKCDPolyhedronShPtr kppPolyhedron = CkppKCDBox::create(polyhedronName, x, y, z);

  if (!kppPolyhedron) {
    ODEBUG2("ChppciObstacle_impl::createPolyhedron: failed to create polyhedron "	 << polyhedronName);
    return -1;
  }
  polyhedronMap[polyhedronName] = kppPolyhedron;

  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::addPoint(const char* inPolyhedronName, 
					CORBA::Double x, CORBA::Double y, 
					CORBA::Double z) 
  throw(CORBA::SystemException)
{
  std::string polyhedronName(inPolyhedronName);

  // Check that polyhedron exists.
  if (polyhedronMap.count(polyhedronName) != 1) {
    ODEBUG1(":addPoint: polyhedron " << polyhedronName	 << " does not exist.");
    return -1;
  }
  CkppKCDPolyhedronShPtr kppPolyhedron = polyhedronMap[polyhedronName];
  unsigned int rank;
  kppPolyhedron->CkcdPolyhedron::addPoint(x, y, z, rank);

  return rank;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::addTriangle(const char* inPolyhedronName, 
					   long pt1, long pt2, long pt3)
  throw(CORBA::SystemException)
{
  std::string polyhedronName(inPolyhedronName);

  // Check that polyhedron exists.
  if (polyhedronMap.count(polyhedronName) != 1) {
    ODEBUG1(":addTriangle: polyhedron " << polyhedronName	 << " does not exist.");
    return -1;
  }
  CkppKCDPolyhedronShPtr kppPolyhedron = polyhedronMap[polyhedronName];
  unsigned int rank;
  kppPolyhedron->addTriangle(pt1, pt2, pt3, rank);

  return rank;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::getDeviceDim(CORBA::Short inProblemId, CORBA::Short& deviceDim)
throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

   
    deviceDim = hppRobot->countDofs ();
  }
  else{
    ODEBUG1(":setCurrentConfig: wrong robot Id");
    return -1;
  }
  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::addPolyToBody(const char* inBodyName, const char* inPolyhedronName, 
					     const hppCorbaServer::Configuration& inConfig)
  throw(CORBA::SystemException)
{
  std::string bodyName(inBodyName);
  std::string polyhedronName(inPolyhedronName);

  // Check that body of this name exits.
  if (bodyMap.count(bodyName) != 1) {
    ODEBUG1(":setBodyInnerObject: body "	 << bodyName << " does not exist.");
    return -1;
  }
  // Check that polyhedron exists.
  if (polyhedronMap.count(polyhedronName) != 1) {
    ODEBUG1(":addPolyToCollList: polyhedron "	 << polyhedronName << " does not exist.");
    return -1;
  }

  ChppBodyShPtr hppBody = bodyMap[bodyName];
  CkppKCDPolyhedronShPtr kppPolyhedron = polyhedronMap[polyhedronName];

#if 0
  std::vector<CkcdObjectShPtr> innerObjectVector;
  hppBody->innerObjects(innerObjectVector);
  innerObjectVector.push_back(kppPolyhedron);
  hppBody->setInnerObjects(innerObjectVector);

  // If body is already attached to a joint, add polyhedron to the corresponding joint component.
  CkwsJointShPtr kwsJoint = hppBody->joint();
  if (CkppJointComponentShPtr kppJoint = KIT_DYNAMIC_PTR_CAST(CkppJointComponent, kwsJoint)) {
    kppJoint->addSolidComponentRef(CkppSolidComponentRef::create(kppPolyhedron)); 
  }  
#else
  // Set polyhedron in given configuration.
  CkitMat4 pos;
  ConfigurationToCkitMat4(inConfig, pos);
  
  if (!hppBody->addSolidComponent(CkppSolidComponentRef::create(kppPolyhedron), pos)) {
    return -1;
  }
#endif
  return 0;

}
