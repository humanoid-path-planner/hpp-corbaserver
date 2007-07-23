/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#include <iostream>
#include "hppciServer.h"
#include "hppciRobot.h"
#if WITH_OPENHRP
#include "hppciOpenHrp.h"
#endif

// ==========================================================================

ChppciRobot_impl::ChppciRobot_impl(ChppPlanner *inHppPlanner)
{
  hppPlanner = inHppPlanner;
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
  ChppDeviceShPtr hppDevice=ChppDevice::create(robotName);
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
  ChppDeviceShPtr hppDevice = robotMap[robotName];
  // Create a new problem with this robot.
  hppPlanner->addHppProblem(hppDevice);
  
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
  ChppDeviceShPtr hppDevice = robotMap[robotName];
  CkwsJointShPtr kwsJoint = jointMap[jointName];
  
  if (hppDevice->setRootJoint(kwsJoint)!=KD_OK) {
    cerr << "ChppciRobot_impl::setRobotRootJoint: failed to set joint "
	 << jointName << " as root joint of robot " << robotName << "." 
	 << endl;
    return -1;
  }
  return 0;
}

#if WITH_OPENHRP
CORBA::Short ChppciRobot_impl::loadHrp2Model(const char* inHrp2Name, const char* inCorbaNameService)
{
  ChppciOpenHrpClient openHrpClient(hppPlanner);
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
  CkwsDofShPtr extraDof = CkwsDof::create(inRevolute);
  // Check whether creation failed.
  if (!extraDof) {
    cerr << "ChppciRobot_impl::createExtraDof: failed to create extra degree of freedom "
	 << dofName << endl;
    return -1;
  }
  if (inValueMin <= inValueMax) {
    extraDof->isBounded(true);
    extraDof->bounds(inValueMin, inValueMax);
  } else {
    extraDof->isBounded(false);
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
  ChppDeviceShPtr hppDevice = robotMap[robotName];
  CkwsDofShPtr kwsExtraDof = extraDofMap[dofName];
  
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
					   const char* inJointType, const matrix4 pos, 
					   const jointBoundSeq& inJointBound) 
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

  CkwsJointShPtr kwsJoint;

	// Fill position matrix
  CkitMat4 posMatrix;
  for (unsigned int iRow=0; iRow < 4; iRow++) {
  	for (unsigned int iCol=0; iCol < 4; iCol++) {
  		posMatrix(iRow, iCol) = pos[iRow][iCol];
  	}
  } 
//  cout << "Position matrix = (( " << posMatrix(0, 0) << ",\t " << posMatrix(0, 1) << ",\t " << posMatrix(0, 2) << ",\t " << posMatrix(0, 3) << " )" << endl;
//  cout << "                   ( " << posMatrix(1, 0) << ",\t " << posMatrix(1, 1) << ",\t " << posMatrix(1, 2) << ",\t " << posMatrix(1, 3) << " )" << endl;
//  cout << "                   ( " << posMatrix(2, 0) << ",\t " << posMatrix(2, 1) << ",\t " << posMatrix(2, 2) << ",\t " << posMatrix(2, 3) << " )" << endl;
//  cout << "                   ( " << posMatrix(3, 0) << ",\t " << posMatrix(3, 1) << ",\t " << posMatrix(3, 2) << ",\t " << posMatrix(3, 3) << " ))" << endl;

  // Determine type of joint.
  if (jointType == "anchor") {
    kwsJoint = CkwsJointAnchor::create();
  }
  else if (jointType == "freeflyer") {
    kwsJoint = CkwsJointFreeFlyer::create(posMatrix);
  }
  else if (jointType == "plan") {
    kwsJoint = CkwsJointPlan::create(posMatrix);
  }
  else if (jointType == "rotation") {
    kwsJoint = CkwsJointRotation::create(posMatrix);
  }
  else if (jointType == "translation") {
    kwsJoint = CkwsJointTranslation::create(posMatrix);
  }
  else {
    cerr << "ChppciRobot_impl::createJoint: joint type " << jointType 
	 << " does not exist." << endl;
    return -1;
  }
  // Check whether creation failed.
  if (!kwsJoint) {
    cerr << "ChppciRobot_impl::createJoint: failed to create joint "
	 << jointName << endl;
    return -1;
  } 
  // Bound joint if needed.
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
  // Store joint in jointMap.
  jointMap[jointName] = kwsJoint;

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
  CkwsJointShPtr parentJoint = jointMap[inParentName];
  CkwsJointShPtr childJoint = jointMap[inChildName];
  if (parentJoint->addJoint(childJoint) != KD_OK) {
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

  // get object hppPlanner of Corba server.
  ChppPlanner *hppPlanner = ChppciServer::getInstance()->getHppPlanner();
  unsigned int nbProblems = hppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    ChppDeviceShPtr hppRobot = hppPlanner->robotIthProblem(hppProblemId);

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

CORBA::Short ChppciRobot_impl::setJointLocked(CORBA::Short problemId, CORBA::Short jointId, 
					      CORBA::Boolean locked, CORBA::Double lockedValue)
  throw(CORBA::SystemException)
{
  unsigned int hppRobotId = (unsigned int)problemId;

  // get object hppPlanner of Corba server.
  ChppPlanner *hppPlanner = ChppciServer::getInstance()->getHppPlanner();

  unsigned int nbRobots = hppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppRobotId < nbRobots) {
    // Get robot in hppPlanner object.
    ChppDeviceShPtr hppRobot = hppPlanner->robotIthProblem(hppRobotId);

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

  // get object hppPlanner of Corba server.
  ChppPlanner *hppPlanner = ChppciServer::getInstance()->getHppPlanner();

  unsigned int nbProblems = hppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    ChppDeviceShPtr hppRobot = hppPlanner->robotIthProblem(hppProblemId);

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
    
    return (short)hppPlanner->robotCurrentConfIthProblem(hppProblemId, config);
  }
  else {
    cerr << "ChppciRobotConfig_impl::setCurrentConfig: wrong robot Id" << endl;
    return KD_ERROR;
  }
  return KD_OK;
}

// ==========================================================================


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
dofSeq* ChppciRobot_impl::getCurrentConfig(CORBA::Short inProblemId)
    throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;

  dofSeq *dofArray;

  // get object hppPlanner of Corba server.
  ChppPlanner *hppPlanner = ChppciServer::getInstance()->getHppPlanner();

  unsigned int nbProblems = hppPlanner->getNbHppProblems();

  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    ChppDeviceShPtr hppRobot = hppPlanner->robotIthProblem(hppProblemId);

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

/*
/// \brief Comment in interface ChppciRobot::getCurrentConfig
CORBA::Short ChppciRobot_impl::Test(CORBA::Short inProblemId)
    throw(CORBA::SystemException)
{

}
*/

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
  CkwsJointShPtr kwsJoint = jointMap[jointName];
  ChppBodyShPtr hppBody = bodyMap[bodyName];

  if (kwsJoint->setAttachedBody(hppBody) != KD_OK) {
    cerr << "ChppciRobot_impl::attachBodyToJoint: failed to attach body "
	 << bodyName << " to joint " << jointName << endl;
    return -1;
  }
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

CORBA::Short ChppciRobot_impl::setBodyInnerObject(const char* inBodyName, 
						  const char* inListName)
  throw(CORBA::SystemException)
{
  std::string bodyName(inBodyName);
  std::string listName(inListName);

  // Check that body of this name exits.
  if (bodyMap.count(bodyName) != 1) {
    cerr << "ChppciRobot_impl::setBodyInnerObject: body "
	 << bodyName << " does not exist." << endl;
    return -1;
  }
  // Check that collision list of this name exists.
  if (collisionListMap.count(listName) != 1) {
    cerr << "ChppciRobot_impl::setBodyInnerObject: collision list " 
	 << listName << " does not exist." << endl;
    return -1;
  }
  ChppBodyShPtr hppBody = bodyMap[bodyName];
  std::vector<CkcdObjectShPtr> kcdCollisionList = collisionListMap[listName];

  hppBody->setInnerObjects(kcdCollisionList);

  return 0;
}

// ==========================================================================

nameSeq* ChppciRobot_impl::getBodyInnerObject(const char* inBodyName)
{
  std::string bodyName(inBodyName);

  nameSeq *innerObjectSeq = NULL;
  // Find the body corresponding to the name in ChppPlanner object.
  ChppBodyConstShPtr hppBody = hppPlanner->findBodyByName(bodyName);
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
	// Cast object into ChppPolyhedron.
	if (ChppPolyhedronShPtr hppPolyhedron = boost::dynamic_pointer_cast<ChppPolyhedron>(kcdObject)) {
	  // Get name of polyhedron and add it into the list.
	  std::string polyhedronName = hppPolyhedron->name();
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
  ChppBodyConstShPtr hppBody = hppPlanner->findBodyByName(bodyName);
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
	// Cast object into ChppPolyhedron.
	if (ChppPolyhedronShPtr hppPolyhedron = boost::dynamic_pointer_cast<ChppPolyhedron>(kcdObject)) {
	  // Get name of polyhedron and add it into the list.
	  std::string polyhedronName = hppPolyhedron->name();
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

// ==========================================================================

CORBA::Short ChppciRobot_impl::createCollisionList(const char* inListName)
  throw(CORBA::SystemException)
{
  std::string listName(inListName);

  // Check that collision list does not already exist.
  if (collisionListMap.count(listName) != 0) {
    cerr << "ChppciRobot_impl::createCollisionList: collision list "
	 << listName << " already exists." << endl;
    return -1;
  }
  std::vector<CkcdObjectShPtr> collisionList ;
  collisionListMap[listName] = collisionList;
  return 0;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::addPolyToCollList(const char* inListName, 
						 const char* inPolyhedronName)
  throw(CORBA::SystemException)
{
  std::string listName(inListName);
  std::string polyhedronName(inPolyhedronName);

  // Check that collision list exists.
  if (collisionListMap.count(listName) != 1) {
    cerr << "ChppciRobot_impl::addPolyToCollList: collision list "
	 << listName << " does not exist." << endl;
    return -1;
  }
  // Check that polyhedron exists.
  if (polyhedronMap.count(polyhedronName) != 1) {
    cerr << "ChppciRobot_impl::addPolyToCollList: polyhedron "
	 << polyhedronName << " does not exist." << endl;
    return -1;
  }
  std::vector< CkcdObjectShPtr > kcdCollisionList = collisionListMap[listName];
  ChppPolyhedronShPtr hppPolyhedron = polyhedronMap[polyhedronName];

  // Build collision entity for KCD.
  hppPolyhedron->makeCollisionEntity();
  kcdCollisionList.push_back(hppPolyhedron);

  return 0;
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
  
  // get object hppPlanner of Corba server.
  ChppPlanner *hppPlanner = ChppciServer::getInstance()->getHppPlanner();

  unsigned int nbProblems = hppPlanner->getNbHppProblems();


  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    ChppDeviceShPtr hppRobot = hppPlanner->robotIthProblem(hppProblemId);

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
    CkitPoint3 CoM =  hppRobot->computeCenterOfMass();
    cout<<"com: "<<CoM[0]<<", "<<CoM[1]<<", "<<CoM[2]<<endl;

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
  ChppPolyhedronShPtr hppPolyhedron = ChppPolyhedron::create(polyhedronName);

  if (!hppPolyhedron) {
    cerr << "ChppciRobot_impl::createPolyhedron: failed to create polyhedron "
	 << polyhedronName << endl;
    return -1;
  }
  polyhedronMap[polyhedronName] = hppPolyhedron;

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
  ChppPolyhedronShPtr hppPolyhedron = polyhedronMap[polyhedronName];
  unsigned int rank;
  hppPolyhedron->addPoint(x, y, z, rank);

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
  ChppPolyhedronShPtr hppPolyhedron = polyhedronMap[polyhedronName];
  unsigned int rank;
  hppPolyhedron->addTriangle(pt1, pt2, pt3, rank);

  return rank;
}

// ==========================================================================

CORBA::Short ChppciRobot_impl::getDeviceDim(CORBA::Short problemId, CORBA::Short& deviceDim)
throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)problemId;

  // get object hppPlanner of Corba server.
  ChppPlanner *hppPlanner = ChppciServer::getInstance()->getHppPlanner();

  unsigned int nbProblems = hppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    ChppDeviceShPtr hppRobot = hppPlanner->robotIthProblem(hppProblemId);

   
    deviceDim = hppRobot->countDofs ();
  }
  else{
    cerr << "ChppciRobot_impl::setCurrentConfig: wrong robot Id" << endl;
    return KD_ERROR;
  }
  return KD_OK;
}

