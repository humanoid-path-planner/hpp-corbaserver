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

static ktStatus attachSolidComponentsToJoint(const CkppJointComponentShPtr& inKppJoint, 
					     const ChppBodyShPtr& inHppBody)
{
  std::vector<CkcdObjectShPtr> innerObjectVector;
  inHppBody->innerObjects(innerObjectVector);

  for (unsigned int iObj=0; iObj<innerObjectVector.size(); iObj++) {
    CkcdObjectShPtr object = innerObjectVector[iObj];

    CkppKCDPolyhedronShPtr kppPoly = KIT_DYNAMIC_PTR_CAST(CkppKCDPolyhedron, object);
    if (!kppPoly) {
      std::cerr << "attachSolidComponentsToJoint: Object is not a CkppKCDPolyhedron." << std::endl;
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
    cerr << "ChppciRobot_impl::createRobot: robot " << robotName << " already exists." << endl;
    return -1;
  }
  // Try to create a robot.
  CkppDeviceComponentShPtr hppDevice=CkppDeviceComponent::create(robotName);
  if (!hppDevice) {
    cerr << "ChppciRobot_impl::createRobot: failed to create a robot.";
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
    cerr << "ChppciRobot_impl::addHppProblem: robot " << robotName << " does not exist." << endl;
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
    cerr << "ChppciRobot_impl::setRobotRootJoint: robot " << robotName << " does not exist." << endl;
    return -1;
  }
  // Check that joint of this name exists.
  if (jointMap.count(jointName) != 1) {
    cerr << "ChppciRobot_impl::setRobotRootJoint: joint " << jointName << " does not exist." << endl;
    return -1;
  }
  CkppDeviceComponentShPtr hppDevice = robotMap[robotName];
  CkppJointComponentShPtr kppJoint = jointMap[jointName];
  
  if (hppDevice->rootJointComponent(kppJoint)!=KD_OK) {
    cerr << "ChppciRobot_impl::setRobotRootJoint: failed to set joint "
	 << jointName << " as root joint of robot " << robotName << "." 
	 << endl;
    return -1;
  }
  return 0;
}

#if WITH_OPENHRP
CORBA::Short ChppciRobot_impl::loadHrp2Model()
{
  ChppciOpenHrpClient openHrpClient(attHppPlanner);
  if (openHrpClient.loadHrp2Model() != KD_OK) {
    return KD_ERROR;
  }
  return KD_OK;
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
    cerr << "ChppciRobot_impl::createExtraDof: extra degree of freedom " 
	 << dofName << " already exists." << endl;
    return -1;
  }
  CkppExtraDofComponentShPtr extraDof = CkppExtraDofComponent::create(inRevolute, "inDofName");
  // Check whether creation failed.
  if (!extraDof) {
    cerr << "ChppciRobot_impl::createExtraDof: failed to create extra degree of freedom "
	 << dofName << endl;
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
    cerr << "ChppciRobot_impl::addExtraDofToRobot: robot " << robotName << " does not exist." << endl;
    return -1;
  }
  // Check that extra degree of freedom of this name exists.
  if (extraDofMap.count(dofName) != 1) {
    cerr << "ChppciRobot_impl::addExtraDofToRobot: joint " << dofName << " does not exist." << endl;
    return -1;
  }
  CkppDeviceComponentShPtr hppDevice = robotMap[robotName];
  CkppExtraDofComponentShPtr kwsExtraDof = extraDofMap[dofName];
  
  if (hppDevice->addExtraDof(kwsExtraDof)!=KD_OK) {
    cerr << "ChppciRobot_impl::addExtraDofToRobot: failed add extra degree of freedom "
	 << dofName << " to robot " << robotName << "." 
	 << endl;
    return -1;
  }
  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::createJoint(const char* inJointName, 
					   const char* inJointType, const Configuration& pos, 
					   const jointBoundSeq& inJointBound,
					   CORBA::Boolean inDisplay) 
  throw(CORBA::SystemException)
{
  std::string jointName(inJointName);
  std::string jointType(inJointType);

  // Check that joint of this name does not already exist.
  if (jointMap.count(jointName) != 0) {
    cerr << "ChppciRobot_impl::createJoint: joint " << jointName 
	 << " already exists." << endl;
    return -1;
  }

  CkppJointComponentShPtr kppJoint;

  // Fill position matrix
  CkitMat4 posMatrix;
  ConfigurationToCkitMat4(pos, posMatrix);

  //  cout << "Position matrix = (( " << posMatrix(0, 0) << ",\t " << posMatrix(0, 1) << ",\t " << posMatrix(0, 2) << ",\t " << posMatrix(0, 3) << " )" << endl;
  //  cout << "                   ( " << posMatrix(1, 0) << ",\t " << posMatrix(1, 1) << ",\t " << posMatrix(1, 2) << ",\t " << posMatrix(1, 3) << " )" << endl;
  //  cout << "                   ( " << posMatrix(2, 0) << ",\t " << posMatrix(2, 1) << ",\t " << posMatrix(2, 2) << ",\t " << posMatrix(2, 3) << " )" << endl;
  //  cout << "                   ( " << posMatrix(3, 0) << ",\t " << posMatrix(3, 1) << ",\t " << posMatrix(3, 2) << ",\t " << posMatrix(3, 3) << " ))" << endl;

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
    cerr << "ChppciRobot_impl::createJoint: joint type " << jointType 
	 << " does not exist." << endl;
    return -1;
  }
  // Check whether creation failed.
  if (!kppJoint) {
    cerr << "ChppciRobot_impl::createJoint: failed to create joint "
	 << jointName << endl;
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
      if (vMin <= vMax) {
	kwsJoint->dof(iDof)->isBounded(true);
	kwsJoint->dof(iDof)->bounds(vMin, vMax);
      }
    }
  }
  kppJoint->doesDisplayPath(inDisplay);
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
    cerr << "ChppciRobot_impl::addJoint: joint " << inParentName 
	 << " does not exist." << endl;
    return -1;
  }
  // Check that joint of this name does not already exist.
  if (jointMap.count(inChildName) != 1) {
    cerr << "ChppciRobot_impl::addJoint: joint " << inChildName 
	 << " does not exist." << endl;
    return -1;
  }
  CkppJointComponentShPtr parentJoint = jointMap[inParentName];
  CkppJointComponentShPtr childJoint = jointMap[inChildName];
  if (parentJoint->addChildJointComponent(childJoint) != KD_OK) {
    cerr << "ChppciRobot_impl::addJoint: failed to attach joint "
	 << inChildName << " to joint " << inParentName << endl;
    return -1;
  }
  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::setJointBounds(CORBA::Short problemId, CORBA::Short inJointId, 
					      const jointBoundSeq& inJointBound)
  throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)problemId;
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
	cout<<"ChppciRobotConfig_impl::setJointBounds: nbJointBounds "<<nbJointBounds
	    <<" != kwsJointNbDofs "<<kwsJointNbDofs<<" x 2"<<endl;
	return KD_ERROR;
      }

    }
    else {
      cerr << "ChppciRobotConfig_impl::setJointBounds: jointId=" 
	   << jointId 
	   << " should be smaller than number of joints="
	   << jointVector.size() << endl;
      return KD_ERROR;
    }
  }
  else {
    cerr << "ChppciRobotConfig_impl::setJointBounds: problemId=" 
	 << hppProblemId 
	 << " should be smaller than number of problems="
	 << nbProblems << endl;
    return KD_ERROR;
  }
  return KD_OK;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::setDofLocked(CORBA::Short problemId, CORBA::Short jointId, 
					    CORBA::Boolean locked, CORBA::Double lockedValue)
  throw(CORBA::SystemException)
{
  unsigned int hppRobotId = (unsigned int)problemId;

  unsigned int nbRobots = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppRobotId < nbRobots) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppRobotId);

    // get joint
    CkwsDevice::TDofVector dofList;
    hppRobot->getDofVector(dofList);

    if(jointId >= (unsigned short) dofList.size()){
      cerr <<"ChppciRobot_impl::setJointBound: joint Id "<<jointId<<" is larger than total size"
	   <<dofList.size();
      return KD_ERROR;
    }

    dofList[jointId]->isLocked(locked);

    if(locked)
      dofList[jointId]->lockedValue(lockedValue);
  }
  else {
    cerr << "ChppciRobotConfig_impl::setJointBound: wrong robot Id" << endl;
    return KD_ERROR;
  }
  return KD_OK;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::setCurrentConfig(CORBA::Short inProblemId, 
						const dofSeq& dofArray) 
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
    cout<<"robot id "<<hppProblemId<<", configDim "<<configDim<<",  deviceDim "<<deviceDim<<endl;
    if(configDim != deviceDim){
      cerr << "ChppciRobotConfig_impl::setCurrentConfig: dofVector Does not match" << endl;
      return KD_ERROR;

      // dofVector.resize(deviceDim, 0);
    }

    // Create a config for robot initialized with dof vector.
    CkwsConfig config(hppRobot, dofVector);
    
    return (short)attHppPlanner->robotCurrentConfIthProblem(hppProblemId, config);
  }
  else {
    cerr << "ChppciRobotConfig_impl::setCurrentConfig: wrong robot Id" << endl;
    return KD_ERROR;
  }
  return KD_OK;
}

// ==========================================================================

#if WITH_OPENHRP

#define LARM_JOINT0_OPENHRP 29
#define RHAND_JOINT0_OPENHRP 36
#define LARM_JOINT0_KINEO 34
#define RHAND_JOINT0_KINEO 29

/// \brief the config is in the order of OpenHRP Joints  RARM, LARM, RHAND, LHAND
CORBA::Short ChppciRobot_impl::setCurrentConfigOpenHRP(CORBA::Short problemId, const dofSeq& dofArray)
    throw(CORBA::SystemException)
{
  dofSeq dofArrayKineo(dofArray);

  // LARM
  for(unsigned int i=0; i<7; i++){
    dofArrayKineo[LARM_JOINT0_KINEO+i] = dofArray[LARM_JOINT0_OPENHRP+i];
  }
  // RHAND
  for(unsigned int i=0; i<5; i++){
    dofArrayKineo[RHAND_JOINT0_KINEO+i] = dofArray[RHAND_JOINT0_OPENHRP+i];
  }

  return setCurrentConfig(problemId, dofArrayKineo);
}

/// \brief Comment in interface ChppciRobot::getCurrentConfig
dofSeq* ChppciRobot_impl::getCurrentConfigOpenHRP(CORBA::Short inProblemId)
    throw(CORBA::SystemException)
{
  dofSeq *dofArray = getCurrentConfig(inProblemId);
  
  dofSeq dofArrayKineo(*dofArray);

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
dofSeq* ChppciRobot_impl::getCurrentConfig(CORBA::Short inProblemId)
    throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;

  dofSeq *dofArray;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

    std::vector<double> dofVector;
    hppRobot->getCurrentDofValues(dofVector);

    // by Yoshida 06/08/25
    unsigned int deviceDim = hppRobot->countDofs ();

    // cerr<<"deviceDim "<<deviceDim<<" [ ";
    dofArray = new dofSeq();
    dofArray->length(deviceDim);

    for(unsigned int i=0; i<deviceDim; i++){
      (*dofArray)[i] = dofVector[i];
      // cerr<<" "<<(*dofArray)[i];
    }
    // cerr<<" ] "<<endl;

    return dofArray;
  }

  else {
    cerr << "ChppciRobotConfig_impl::CurrentConfig: wrong robot Id" << endl;
    dofArray = new dofSeq(1);

    return dofArray;
  }

  return new dofSeq(1);
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
    cerr << "ChppciRobot_impl::addJoint: joint " << jointName 
	 << " does not exist." << endl;
    return -1;
  }
  // Check that body of this name exits.
  if (bodyMap.count(bodyName) != 1) {
    cerr << "ChppciRobot_impl::attachBodyToJoint: body "
	 << bodyName << " does not exist." << endl;
    return -1;
  }
  CkppJointComponentShPtr kppJoint = jointMap[jointName];
  CkwsJointShPtr kwsJoint = KIT_DYNAMIC_PTR_CAST(CkwsJoint, kppJoint);
  ChppBodyShPtr hppBody = bodyMap[bodyName];

  if (kwsJoint->setAttachedBody(hppBody) != KD_OK) {
    cerr << "ChppciRobot_impl::attachBodyToJoint: failed to attach body "
	 << bodyName << " to joint " << jointName << endl;
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
    cerr << "ChppciRobot_impl::createBody: body " << bodyName 
	 << " already exists." << endl;
    return -1;
  }
  ChppBodyShPtr hppBody = ChppBody::create(bodyName);
  
  if (!hppBody) {
    cerr << "ChppciRobot_impl::createBody: failed to create body "
	 << bodyName << "." << endl;
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

nameSeq* ChppciRobot_impl::getBodyInnerObject(const char* inBodyName)
{
  std::string bodyName(inBodyName);

  nameSeq *innerObjectSeq = NULL;
  // Find the body corresponding to the name in ChppPlanner object.
  ChppBodyConstShPtr hppBody = attHppPlanner->findBodyByName(bodyName);
  CkwsKCDBodyConstShPtr kcdBody;

  if (hppBody) {
    std::vector<CkcdObjectShPtr> innerObjectList = hppBody->innerObjects();
    if (innerObjectList.size() > 0) {
      unsigned int nbObjects = innerObjectList.size();
      // Allocate result now that the size is known.
      CORBA::ULong size = (CORBA::ULong)nbObjects;
      char** nameList = nameSeq::allocbuf(size);
      innerObjectSeq = new nameSeq(size, size, nameList);

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
    cerr << "ChppciRobot_impl::getBodyInnerObject: body of name " 
	 << " not found." << endl;
  }
  if (!innerObjectSeq) {
    innerObjectSeq = new nameSeq(0);
  }

  return innerObjectSeq;
}

// ==========================================================================

nameSeq* ChppciRobot_impl::getBodyOuterObject(const char* inBodyName)
{
  std::string bodyName(inBodyName);

  nameSeq *outerObjectSeq = NULL;
  // Find the body corresponding to the name in ChppPlanner object.
  ChppBodyConstShPtr hppBody = attHppPlanner->findBodyByName(bodyName);
  CkwsKCDBodyConstShPtr kcdBody;

  if (hppBody) {
    std::vector<CkcdObjectShPtr> outerObjectList = hppBody->outerObjects();
    if (outerObjectList.size() > 0) {
      unsigned int nbObjects = outerObjectList.size();
      // Allocate result now that the size is known.
      CORBA::ULong size = (CORBA::ULong)nbObjects;
      char** nameList = nameSeq::allocbuf(size);
      outerObjectSeq = new nameSeq(size, size, nameList);
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
    cerr << "ChppciRobot_impl::getBodyInnerObject: body of name " 
	 << " not found." << endl;
  }
  if (!outerObjectSeq) {
    outerObjectSeq = new nameSeq(0);
  }

  return outerObjectSeq;
}

#define DOF_SHIFT 7 // DOF shift. extraDof + 6 dof of the free-flying base.

// ==========================================================================

CORBA::Short 
ChppciRobot_impl::checkLinkCollision(CORBA::Short problemId, CORBA::Short jointId, 
				     CORBA::Short& result) 
throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)problemId;
  unsigned int hppJointId = (unsigned int)jointId; // +DOF_SHIFT;
  
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();


  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

    // by Yoshida 06/08/25
    unsigned int deviceDim = hppRobot->countDofs ();

    if (hppJointId > deviceDim) {
      cerr << "ChppciRobot_impl::checkLinkCollision: wrong joint Id" << endl;
      return KD_ERROR;
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
    std::cout<<" =====================debugging collision detection ====================== "<<endl;

    for(unsigned int i=0; i<jointList.size(); i++){

      ChppBodyShPtr hppBody = KIT_DYNAMIC_PTR_CAST(ChppBody, jointList[i]->attachedBody());

      CkitMat4 mat = hppBody->absolutePosition();
      CkitMat4 matJoint = jointList[i]->currentPosition();
      CkitVect3 trans = mat.translation();
      CkitVect3 transJoint = matJoint.translation();
      double dist =  hppBody->estimatedDistance();
      cout<<"for joint "<<hppBody->name()<<" body pos "<<trans[0]<<", "<<trans[1]<<", "<<trans[2]
	  <<" distance "<<dist<<endl; 
      cout<<"for joint "<<hppBody->name()<<" joint pos "<<transJoint[0]<<", "<<transJoint[1]<<", "<<transJoint[2]<<endl;
      cout<<" joint config:";
      for(unsigned int j=0; j<3; j++){
	for(unsigned int k=0; k<3; k++)
	  cout<<matJoint(j,k)<<" ";
	cout<<" ";
      }
      cout<<endl;
      hppBody->printCollisionStatus();
      // hppBody->printCollisionStatusFast();
      short res = (CORBA::Short) hppBody->computeEstimatedDistance();

      if(res == 0){
	cout<<"collision "<<endl;

      }
    }

    cout<<endl;


  }
  else{
    cerr << "ChppciRobot_impl::checkLinkCollision: wrong robot Id" << endl;
    return KD_ERROR;
  }

  return KD_OK;
}
 
// ==========================================================================

CORBA::Short ChppciRobot_impl::createPolyhedron(const char* inPolyhedronName)
  throw(CORBA::SystemException)
{
  std::string polyhedronName(inPolyhedronName);

  // Check that polyhedron does not already exist.
  if (polyhedronMap.count(polyhedronName) != 0) {
    cerr << "ChppciRobot_impl::createPolyhedron: polyhedron "
	 << polyhedronName << " already exists." << endl;
    return -1;
  }
  CkppKCDPolyhedronShPtr kppPolyhedron = CkppKCDPolyhedron::create(polyhedronName);

  if (!kppPolyhedron) {
    cerr << "ChppciRobot_impl::createPolyhedron: failed to create polyhedron "
	 << polyhedronName << endl;
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
    cerr << "ChppciObstacle_impl::createPolyhedron: polyhedron "
	 << polyhedronName << " already exists." << endl;
    return -1;
  }
  CkppKCDPolyhedronShPtr kppPolyhedron = CkppKCDBox::create(polyhedronName, x, y, z);

  if (!kppPolyhedron) {
    cerr << "ChppciObstacle_impl::createPolyhedron: failed to create polyhedron "
	 << polyhedronName << endl;
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
    cerr << "ChppciRobot_impl::addPoint: polyhedron " << polyhedronName
	 << " does not exist." << endl;
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
    cerr << "ChppciRobot_impl::addTriangle: polyhedron " << polyhedronName
	 << " does not exist." << endl;
    return -1;
  }
  CkppKCDPolyhedronShPtr kppPolyhedron = polyhedronMap[polyhedronName];
  unsigned int rank;
  kppPolyhedron->addTriangle(pt1, pt2, pt3, rank);

  return rank;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::getDeviceDim(CORBA::Short problemId, CORBA::Short& deviceDim)
throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)problemId;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

   
    deviceDim = hppRobot->countDofs ();
  }
  else{
    cerr << "ChppciRobot_impl::setCurrentConfig: wrong robot Id" << endl;
    return KD_ERROR;
  }
  return KD_OK;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::addPolyToBody(const char* inBodyName, const char* inPolyhedronName, 
					     const Configuration& inConfig)
  throw(CORBA::SystemException)
{
  std::string bodyName(inBodyName);
  std::string polyhedronName(inPolyhedronName);

  // Check that body of this name exits.
  if (bodyMap.count(bodyName) != 1) {
    cerr << "ChppciRobot_impl::setBodyInnerObject: body "
	 << bodyName << " does not exist." << endl;
    return -1;
  }
  // Check that polyhedron exists.
  if (polyhedronMap.count(polyhedronName) != 1) {
    cerr << "ChppciRobot_impl::addPolyToCollList: polyhedron "
	 << polyhedronName << " does not exist." << endl;
    return -1;
  }

  ChppBodyShPtr hppBody = bodyMap[bodyName];
  CkppKCDPolyhedronShPtr kppPolyhedron = polyhedronMap[polyhedronName];

  // Set polyhedron in given configuration.
  CkitMat4 pos;
  ConfigurationToCkitMat4(inConfig, pos);
  kppPolyhedron->setAbsolutePosition(pos);
  
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
  if (!hppBody->addSolidComponent(CkppSolidComponentRef::create(kppPolyhedron))) {
    return -1;
  }
#endif
  return 0;

}
