
/*************************************
INCLUDE
**************************************/
#include <iostream>
#include <string>

#include "hppCorbaServer/hppciOpenHrp.h"

#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION
#include "common-modelloader.hh"
#include "jrl-modelloader.hh"

#include "hppOpenHRP/parserOpenHRPKineoDevice.h"
#include "hppOpenHRP/parserOpenHRPKineoObstacle.h"

#include "hppModel/hppJoint.h"
#include "hppModel/hppBody.h"
#include "hppModel/hppSpecificHumanoidRobot.h"

#include "hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h"

#include "hrp2_14/hrp2_14.h"

/*************************************
DEFINE
**************************************/


  
// ==============================================================================
//
//  INTERNAL CLASS : Internal Corba Object
//
//  Note : this class has been created because "common.hh" and "modelloader.hh"
//  should not appear in hppciOpenHrp.h  
//
// ==============================================================================


class CinternalCorbaObject {
    
public :

  /// Corba elements.
  CORBA::ORB_var orb;
  /// Corba elements.
  CORBA::Object_var nameService;

  // HRP2 Corba Information 
  ModelInfo_var HRP2info;

  // Obstacle Corba Information
  std::vector<ModelInfo_var> obstInfoVector;

  ChppciOpenHrpClient *attOpenHrpClient ;


  // Constructor
  CinternalCorbaObject(ChppciOpenHrpClient *openHrpClientPtr) {
    attOpenHrpClient = openHrpClientPtr ;
  } ; 

  ktStatus getModelLoader();

  ModelLoader_var attLoader;

} ; 


//-------------------------------------------------------------------------------
//
//  
//  METHODS FOR ChppciOpenHrpClient
//
//
//-------------------------------------------------------------------------------
  
// ==============================================================================

ChppciOpenHrpClient::ChppciOpenHrpClient(ChppPlanner *hpp) 
{
  hppPlanner = hpp;
  privateCorbaObject = new CinternalCorbaObject(this) ;
}
  
// ==============================================================================

ChppciOpenHrpClient::~ChppciOpenHrpClient() {
  // nothing to do
}
  
// ==============================================================================
enum {
  BODY = 0 ,

  RLEG_LINK0 = 1 ,
  RLEG_LINK1 ,
  RLEG_LINK2 ,
  RLEG_LINK3 ,
  RLEG_LINK4 ,
  RLEG_LINK5 ,

  LLEG_LINK0 = 7 ,
  LLEG_LINK1 ,	
  LLEG_LINK2 ,
  LLEG_LINK3 ,
  LLEG_LINK4 ,
  LLEG_LINK5 ,

  CHEST_LINK0 = 13 ,
  CHEST_LINK1 ,

  HEAD_LINK0 = 15 ,
  HEAD_LINK1 ,

  RARM_LINK0 = 17 ,
  RARM_LINK1 ,
  RARM_LINK2 ,
  RARM_LINK3 ,
  RARM_LINK4 ,
  RARM_LINK5 ,
  RARM_LINK6 ,

  RHAND_LINK0 =	24 ,
  RHAND_LINK1 ,	
  RHAND_LINK2 ,
  RHAND_LINK3 ,
  RHAND_LINK4 ,

  LARM_LINK0 = 29 ,
  LARM_LINK1 ,
  LARM_LINK2 ,
  LARM_LINK3 ,
  LARM_LINK4 ,
  LARM_LINK5 ,
  LARM_LINK6 ,

  LHAND_LINK0 =	36 ,
  LHAND_LINK1 ,
  LHAND_LINK2 ,
  LHAND_LINK3 ,
  LHAND_LINK4 ,	

  NO_LINK = 41

} ;

void makeColPair(ChppColPair &cp)
{
  // BODY:  +ARM2  +LEG2 
  cp.addColPairRange(BODY, RARM_LINK2, RARM_LINK6);
  cp.addColPairRange(BODY, LARM_LINK2, LARM_LINK6);
  cp.addColPairRange(BODY, RHAND_LINK0, RHAND_LINK4);
  cp.addColPairRange(BODY, LHAND_LINK0, LHAND_LINK4);
  cp.addColPairRange(BODY, RLEG_LINK2, RLEG_LINK5);
  cp.addColPairRange(BODY, LLEG_LINK2, LLEG_LINK5);
  cp.addColPairRange(CHEST_LINK0, RARM_LINK3, RARM_LINK6);
  // BODY + CHEST_JOINT1
  cp.addColPair(BODY, CHEST_LINK1);

  // CHEST0  +ARM3  +LEG3 
  cp.addColPairRange(CHEST_LINK0, RARM_LINK3, RARM_LINK6);
  cp.addColPairRange(CHEST_LINK0, LARM_LINK3, LARM_LINK6);
  cp.addColPairRange(CHEST_LINK0, RHAND_LINK0, RHAND_LINK4);
  cp.addColPairRange(CHEST_LINK0, LHAND_LINK0, LHAND_LINK4);
  cp.addColPairRange(CHEST_LINK0, RLEG_LINK3, RLEG_LINK5);
  cp.addColPairRange(CHEST_LINK0, LLEG_LINK3, LLEG_LINK5);

  // CHEST1  +ARM2  +LEG3
  cp.addColPairRange(CHEST_LINK1, RARM_LINK2, RARM_LINK6);
  cp.addColPairRange(CHEST_LINK1, LARM_LINK2, LARM_LINK6);
  cp.addColPairRange(CHEST_LINK1, RHAND_LINK0, RHAND_LINK4);
  cp.addColPairRange(CHEST_LINK1, LHAND_LINK0, LHAND_LINK4);
  cp.addColPairRange(CHEST_LINK1, RLEG_LINK3, RLEG_LINK5);
  cp.addColPairRange(CHEST_LINK1, LLEG_LINK3, LLEG_LINK5);

  // HEAD0  +ARM4
  cp.addColPairRange(HEAD_LINK0, RARM_LINK4, RARM_LINK6);
  cp.addColPairRange(HEAD_LINK0, LARM_LINK4, LARM_LINK6);
  cp.addColPairRange(HEAD_LINK0, RHAND_LINK0, RHAND_LINK4);
  cp.addColPairRange(HEAD_LINK0, LHAND_LINK0, LHAND_LINK4);

  // HEAD1  +ARM4
  cp.addColPairRange(HEAD_LINK1, RARM_LINK4, RARM_LINK6);
  cp.addColPairRange(HEAD_LINK1, LARM_LINK4, LARM_LINK6);
  cp.addColPairRange(HEAD_LINK1, RHAND_LINK0, RHAND_LINK4);
  cp.addColPairRange(HEAD_LINK1, LHAND_LINK0, LHAND_LINK4);

  // LEGS
  //        Same side leg       other side          others
  // LEG0    +LINK4             LINK0, LINK2-5      +ARM3
  // LEG1    Not exposed to outside. no collision check
  // LEG2     none              +LINK2              +ARM2 (same) +ARM3 (other)
  // LEG3    LINK5*             LINK0, LINK2-5      +ARM2 (same) +ARM3 (other)
  //          * can go very close.
  // LEG4     none              +LEG2               +ARM2 (same) +ARM3 (other)
  // LEG5     none              +LEG2               +ARM2 (same) +ARM3 (other)


  // Right legs
  // LEG0
  cp.addColPairRange(RLEG_LINK0, RLEG_LINK4, RLEG_LINK5);
  cp.addColPair(RLEG_LINK0, LLEG_LINK0);
  cp.addColPairRange(RLEG_LINK0, LLEG_LINK2, LLEG_LINK5);
  cp.addColPairRange(RLEG_LINK0, RARM_LINK3, RARM_LINK6);
  cp.addColPairRange(RLEG_LINK0, LARM_LINK3, LARM_LINK6);
  cp.addColPairRange(RLEG_LINK0, RHAND_LINK0, RHAND_LINK4);
  cp.addColPairRange(RLEG_LINK0, LHAND_LINK0, LHAND_LINK4);

  // LEG2
  cp.addColPairRange(RLEG_LINK2, LLEG_LINK2, LLEG_LINK5);

  // LEG3
  // pair leg3 and leg5 excluded.
  // addColPair(RLEG_LINK3, RLEG_LINK5, cp);
  cp.addColPair(RLEG_LINK3, LLEG_LINK0);
  cp.addColPairRange(RLEG_LINK3, LLEG_LINK2, LLEG_LINK3);
  // leg4 no need
  cp.addColPair(RLEG_LINK3, LLEG_LINK5);

  // LEG4 no need.
  // addColPair(RLEG_LINK4, LLEG_LINK0, cp);

  // LEG5
  cp.addColPair(RLEG_LINK5, LLEG_LINK0);
  // addColPairRange(RLEG_LINK5, LLEG_LINK2, LLEG_LINK5, cp);
  cp.addColPairRange(RLEG_LINK5, LLEG_LINK2, LLEG_LINK3);
  cp.addColPair(RLEG_LINK5, LLEG_LINK5);

  // for arm/hands
  for(unsigned int j=RLEG_LINK2; j<=RLEG_LINK5; j++){
    cp.addColPairRange(j, RARM_LINK2, RARM_LINK6);
    cp.addColPairRange(j, LARM_LINK3, LARM_LINK6);
    cp.addColPairRange(j, RHAND_LINK0, RHAND_LINK4);
    cp.addColPairRange(j, LHAND_LINK0, LHAND_LINK4);
  }


  // Left legs
  // LEG0
  cp.addColPairRange(LLEG_LINK0, LLEG_LINK4, LLEG_LINK5);
  cp.addColPair(LLEG_LINK0, RLEG_LINK0);
  cp.addColPairRange(LLEG_LINK0, RLEG_LINK2, RLEG_LINK5);
  cp.addColPairRange(LLEG_LINK0, RARM_LINK3, RARM_LINK6);
  cp.addColPairRange(LLEG_LINK0, LARM_LINK3, LARM_LINK6);
  cp.addColPairRange(LLEG_LINK0, RHAND_LINK0, RHAND_LINK4);
  cp.addColPairRange(LLEG_LINK0, LHAND_LINK0, LHAND_LINK4);

  // LEG2
  cp.addColPairRange(LLEG_LINK2, RLEG_LINK2, RLEG_LINK5);

  // LEG3
  // pair leg3 and leg5 excluded.
  // addColPair(LLEG_LINK3, LLEG_LINK5, cp);
  cp.addColPair(LLEG_LINK3, RLEG_LINK0);
  // addColPairRange(LLEG_LINK3, RLEG_LINK2, RLEG_LINK5, cp);
  cp.addColPairRange(LLEG_LINK3, RLEG_LINK2, RLEG_LINK3);
  cp.addColPair(LLEG_LINK3, RLEG_LINK5);

  // LEG4 no need.
  // addColPair(LLEG_LINK4, RLEG_LINK0, cp);

  // LEG5 leg4 exluded.
  cp.addColPair(LLEG_LINK5, RLEG_LINK0);
  // addColPairRange(LLEG_LINK5, RLEG_LINK2, RLEG_LINK5, cp);
  cp.addColPairRange(LLEG_LINK5, RLEG_LINK2, RLEG_LINK3);
  cp.addColPair(LLEG_LINK5, RLEG_LINK5);

  // for arm/hands
  for(unsigned int j=LLEG_LINK2; j<=LLEG_LINK5; j++){
    cp.addColPairRange(j, RARM_LINK2, RARM_LINK6);
    cp.addColPairRange(j, LARM_LINK2, LARM_LINK6);
    cp.addColPairRange(j, RHAND_LINK0, RHAND_LINK4);
    cp.addColPairRange(j, LHAND_LINK0, LHAND_LINK4);
  }

  // ARM/HANDS
  //        Same side arm/hand     other side arm/hand
  // ARM0      +ARM4                  +ARM4
  // ARM1      none
  // ARM2      none                   +ARM3
  // ARM3      none                   +ARM3
  // ARM4      none                   +ARM2
  // ARM5      none                   +ARM2
  // ARM6      none                   +ARM2
  // HANDS     none                   +ARM2

  // Right arm/hands

  // LINK0
  cp.addColPairRange(RARM_LINK0, RARM_LINK4, RARM_LINK6);
  cp.addColPairRange(RARM_LINK0, RHAND_LINK0, RHAND_LINK4);
  cp.addColPairRange(RARM_LINK0, LARM_LINK4, LARM_LINK6);
  cp.addColPairRange(RARM_LINK0, LHAND_LINK0, LHAND_LINK4);

  // LINK1
  cp.addColPairRange(RARM_LINK1, LARM_LINK3, LARM_LINK6);
  cp.addColPairRange(RARM_LINK1, LHAND_LINK0, LHAND_LINK4);

  // LINK2
  cp.addColPairRange(RARM_LINK2, LARM_LINK3, LARM_LINK6);
  cp.addColPairRange(RARM_LINK2, LHAND_LINK0, LHAND_LINK4);

  // LINK3 - LINK6, HAND
  for(unsigned int j=RARM_LINK3; j<=RARM_LINK6; j++){
    cp.addColPairRange(j, LARM_LINK2, LARM_LINK6);
    cp.addColPairRange(j, LHAND_LINK0, LHAND_LINK4);
  }
  for(unsigned int j=RHAND_LINK0; j<=RHAND_LINK4; j++){
    cp.addColPairRange(j, LARM_LINK2, LARM_LINK6);
    cp.addColPairRange(j, LHAND_LINK0, LHAND_LINK4);
  }

  // Left arm/hands

  // LINK0
  cp.addColPairRange(LARM_LINK0, LARM_LINK4, LARM_LINK6);
  cp.addColPairRange(LARM_LINK0, LHAND_LINK0, LHAND_LINK4);
  cp.addColPairRange(LARM_LINK0, RARM_LINK4, RARM_LINK6);
  cp.addColPairRange(LARM_LINK0, RHAND_LINK0, RHAND_LINK4);

  // LINK1
  cp.addColPairRange(LARM_LINK1, RARM_LINK3, RARM_LINK6);
  cp.addColPairRange(LARM_LINK1, RHAND_LINK0, RHAND_LINK4);

  // LINK2
  cp.addColPairRange(LARM_LINK2, RARM_LINK3, RARM_LINK6);
  cp.addColPairRange(LARM_LINK2, RHAND_LINK0, RHAND_LINK4);

  // LINK3 - LINK6, HAND
  for(unsigned int j=LARM_LINK3; j<=LARM_LINK6; j++){
    cp.addColPairRange(j, RARM_LINK2, RARM_LINK6);
    cp.addColPairRange(j, RHAND_LINK0, RHAND_LINK4);
  }
  for(unsigned int j=LHAND_LINK0; j<=LHAND_LINK4; j++){
    cp.addColPairRange(j, RARM_LINK2, RARM_LINK6);
    cp.addColPairRange(j, RHAND_LINK0, RHAND_LINK4);
  }
}

void setHRP2Specificities(ChppHumanoidRobotShPtr i_humanoid, 
			  ChppJoint *i_joint)
{
    const std::string& name = i_joint->kppJoint()->name();
    if (name == "HEAD_JOINT1"){
	i_humanoid->gazeJoint(i_joint->jrlJoint());
	vector3d dir,pos;
	dir[0]=0; dir[1]=1; dir[2]=0;
	pos[0]=0; pos[1]=0;  pos[2]=-0.118;
	i_humanoid->gaze(dir, pos);

    }else if (name == "RLEG_JOINT5"){
	i_humanoid->rightFoot(i_joint->jrlJoint());
    }else if (name == "LLEG_JOINT5"){
	i_humanoid->leftFoot(i_joint->jrlJoint());
    }else if (name == "RARM_JOINT5"){
	i_humanoid->rightWrist(i_joint->jrlJoint());
	vector3d center,okayAxis,showingAxis,palmAxis;
	center[0] = 0;
	center[1] = 0;
	center[2] = 0.17;
	okayAxis[0] = 0;
	okayAxis[1] = 1;
	okayAxis[2] = 0;
	showingAxis[0] = 0;
	showingAxis[1] = 0;
	showingAxis[2] = 1;
	palmAxis[0] = 1;
	palmAxis[1] = 0;
	palmAxis[2] = 0;
	i_humanoid->rightHand(new CimplHand(i_humanoid->rightWrist(), center, 
					    okayAxis, showingAxis, palmAxis));
    }else if (name == "LARM_JOINT5"){
	i_humanoid->leftWrist(i_joint->jrlJoint());
	vector3d center,okayAxis,showingAxis,palmAxis;
	center[0] = 0;
	center[1] = 0;
	center[2] = 0.17;
	okayAxis[0] = 0;
	okayAxis[1] = 1;
	okayAxis[2] = 0;
	showingAxis[0] = 0;
	showingAxis[1] = 0;
	showingAxis[2] = 1;
	palmAxis[0] = -1;
	palmAxis[1] = 0;
	palmAxis[2] = 0;
	i_humanoid->leftHand(new CimplHand(i_humanoid->leftWrist(), center, 
					   okayAxis, showingAxis, palmAxis));
    }
    for (unsigned int i=0; i<i_joint->countChildJoints(); i++){
	setHRP2Specificities(i_humanoid, i_joint->childJoint(i));
    }
}

void setHRP2OuterLists(ChppHumanoidRobotShPtr i_hrp2)
{
    ChppColPair hrpCP;
    makeColPair(hrpCP);

    std::vector<CkwsJointShPtr> jv;
    i_hrp2->getJointVector(jv);

    // adding outer objects
    for(unsigned int iJoint=0; iJoint<jv.size(); iJoint++){
	ChppBodyShPtr hppBody;
	hppBody = KIT_DYNAMIC_PTR_CAST(ChppBody, jv[iJoint]->attachedBody());
	
	std::vector<CkcdObjectShPtr> mergedList ;
	
	//debug
	//cout<<"joint "<<iJoint<<": adding in outerObjects: ";
	
	for(unsigned int jJoint=0; jJoint<jv.size(); jJoint++){
	    ChppBodyShPtr hppOtherBody = KIT_DYNAMIC_PTR_CAST(ChppBody, jv[jJoint]->attachedBody());
	    
	    if(jJoint != iJoint && hrpCP.existPairNarrow(iJoint, jJoint) ){
		
		//debug
		//cout<<jJoint<<" ";
		
		std::vector<CkcdObjectShPtr> clist;
		clist = hppOtherBody->innerObjects();
		
		for(unsigned int k=0; k< clist.size(); k++){
		    CkppKCDPolyhedronShPtr kppPolyhedron = KIT_DYNAMIC_PTR_CAST(CkppKCDPolyhedron, clist[k]);
		    mergedList.push_back(kppPolyhedron);
		}
	    }
	}
	
	//debug
	//cout<<endl;
	
	// add to outerObject. 
	hppBody->setOuterObjects(mergedList);      
	
	// debug
	//cout<<"mergedlist "<<mergedList.size()<<" objects. "<<endl;
	
	// debug
	//cout<<"has "<<hppBody->outerObjects().size()<<" outer objects "<<endl;
    }
}

// ==============================================================================

ktStatus ChppciOpenHrpClient::loadHrp2Model(ChppDeviceShPtr &HRP2Device)
{
  // 
  // Get Corba objects containing model of HRP2 and Obstacles.
  //
  if (getRobotURL()!= KD_OK) {
    privateCorbaObject->orb->destroy();
    cerr << "ERROR : ChppciOpenHrpClient::loadHrp2Model::Failed to load Hrp2 model" 
	 << endl;
    return KD_ERROR;
  }

  //
  // Build KineoHRP2 from OpenHRPModel
  //
  CparserOpenHRPKineoDevice parser(privateCorbaObject->HRP2info) ;
  ChppHumanoidRobotShPtr humanoid;
  humanoid = ChppSpecificHumanoidRobot<Chrp2OptHumanoidDynamicRobot>::create(privateCorbaObject->HRP2info->getCharObject()->name());
  parser.parser(humanoid);
  Chrp2OptHumanoidDynamicRobot *hrp2;
  hrp2 = dynamic_cast<Chrp2OptHumanoidDynamicRobot *>(humanoid.get());
  hrp2->isKineoOrder(true);
  HRP2Device = humanoid;
  if (!HRP2Device) {
    cerr << " ERROR : ChppciOpenHrpClient::loadHrp2Model : Failed building Kineo HRP2 Model" << endl ;
    return KD_ERROR;
  }
  
  //
  // set HRP2 joints in invisible mode by default
  //
  HRP2Device->isVisible(false) ;

  // set HRP2 specifities
  setHRP2Specificities(humanoid, humanoid->getRootJoint());

  // set Collision Check Pairs
  setHRP2OuterLists(humanoid);

  //
  // set HRP2 in a default configuration (HALFSITTING) ;
  //
  CkwsConfig halfSittingConfig(HRP2Device);
  double dInitPos[] = HALFSITTINGPOSITION_RAD_KINEO ;
  std::vector<double>  halfSittingVector (dInitPos  , dInitPos + sizeof(dInitPos) / sizeof(double) );
  halfSittingConfig.setDofValues( halfSittingVector ) ; 
  HRP2Device->hppSetCurrentConfig(halfSittingConfig) ;

  return KD_OK;
}




ktStatus ChppciOpenHrpClient::loadRobotModel(std::string inFilename, std::string inDeviceName, 
					     ChppDeviceShPtr &outDevice, 
					     std::string inOpenHrpPrefix)
{
  ModelLoader_var loader;
  if (privateCorbaObject->getModelLoader() != KD_OK){
    cerr << "ERROR : ChppciOpenHrpClient::loadRobotModel::Failed to load robot model" 
	 << endl;
    return KD_ERROR;
  }
	
  //
  // Build hppDevice from OpenHRPModel
  //
  std::string url("file://");
  url += inOpenHrpPrefix;
  url += std::string("/etc/");
  url += inFilename;
    
  std:: cout << "ChppciOpenHrpClient::loadRobotModel: reading " << url << std::endl;

  try{
    CparserOpenHRPKineoDevice parser(privateCorbaObject->attLoader->loadURL(url.c_str())) ;
    ChppHumanoidRobotShPtr humanoid;
    parser.parser(humanoid) ;
    outDevice = humanoid;
  } catch (CORBA::SystemException &ex) {
    cerr << "System exception( " << ex._rep_id() << ") in ChppciOpenHrpClient::loadRobotModel()" << endl;
    return KD_ERROR;
  } catch (...){
    cerr << "Unknown exception" << endl;
    return KD_ERROR;
  }
  if (!outDevice) {
    cerr << " ERROR : ChppciOpenHrpClient::loadRobotModel : Failed building Kineo Robot Model" << endl ;
    return KD_ERROR;
  }
  
  //
  // set joints in invisible mode by default
  //
  outDevice->isVisible(false) ;
  outDevice->name(inDeviceName);

  // privateCorbaObject->orb->destroy();

  return KD_OK;
}




ktStatus  ChppciOpenHrpClient::loadHrp2Model()
{
  ChppDeviceShPtr HRP2Device;
  if (loadHrp2Model(HRP2Device) != KD_OK){
    return KD_ERROR;
  }
  
  //
  // ADD HRP2 to the planner
  // 
  if (hppPlanner->addHppProblem(HRP2Device) != KD_OK) {
    cerr << "ChppciOpenHrpClient::loadHrp2Model: Failed to add robot" << endl;
    return KD_ERROR;
  }

  return KD_OK;
 
}

// ==============================================================================

ktStatus  ChppciOpenHrpClient::loadObstacleModel(std::string inFilename, std::string inObstacleName, 
						 CkppKCDPolyhedronShPtr& outPolyhedron,
						 std::string inOpenHrpPrefix)
{
  // 
  // Get Corba objects containing model of Obstacles.
  //
  std::string url("file://");
  url += inOpenHrpPrefix;
  url += std::string("/etc/");
  url += inFilename;

  if (getObstacleURL(url)!= KD_OK) {
    privateCorbaObject->orb->destroy();
    cerr << "ERROR : ChppciOpenHrpClient::loadObstacleModel: failed to load Obstacle model" 
	 << endl;
    return KD_ERROR;
  }

  //
  // build obstacles (if there is some obstacle)
  //
  if (privateCorbaObject->obstInfoVector.size() > 0){  


    //
    // Build KineoObstacle from OpenHRPModel
    //
    CparserOpenHRPKineoObstacle parserObstacle(privateCorbaObject->obstInfoVector);

    std::vector<CkppKCDPolyhedronShPtr> obstacleVector = parserObstacle.parser() ;

    if (obstacleVector.size() == 0) {
      cerr << "ERROR ChppciOpenHrpClient::loadObstacleModel : Failed to parse the obstacle model " <<  endl;
      return KD_ERROR;
    }
    outPolyhedron = obstacleVector[0];
    outPolyhedron->name(inObstacleName);
  }

  // privateCorbaObject->orb->destroy();

  return KD_OK;
}


  
ktStatus CinternalCorbaObject::getModelLoader()
{
  // Services
  string modelLoaderString="ModelLoaderJRL" ;
  string nameServiceString="NameService" ;

  try {
    int argc=1;
    char *argv[1]={"unused"};
    // ORB Creation.
    orb = CORBA::ORB_init(argc, argv);

    // Getting reference of name server
    try {
      nameService = orb->resolve_initial_references(nameServiceString.c_str());
    } catch (const CORBA::ORB::InvalidName&) {
      cerr << "ERROR :ChppciOpenHrpClient::getModelLoader : cannot resolve " << nameServiceString << endl;
      return KD_ERROR;
    }
    if(CORBA::is_nil(nameService)) {
      cerr << " ERROR :ChppciOpenHrpClient::getModelLoader "<< nameServiceString << "is a nil object reference"
	   << endl;
      return KD_ERROR;
    }

    // Getting root naming context
    CosNaming::NamingContext_var cxt = 
      CosNaming::NamingContext::_narrow(nameService);
    if(CORBA::is_nil(cxt)) {
      cerr << " ERROR :ChppciOpenHrpClient::getModelLoader "<< nameServiceString << " is not a NamingContext object reference"
	   << endl;
      return KD_ERROR;
    }

    CosNaming::Name ncFactory;
    ncFactory.length(1);

    // modelloaderJRL
    CosNaming::Name mFactory;
    mFactory.length(1);
    mFactory[0].id = CORBA::string_dup(modelLoaderString.c_str());
    mFactory[0].kind = CORBA::string_dup("");

    try {
      attLoader = ModelLoader::_narrow(cxt -> resolve(mFactory));
    } catch (const CosNaming::NamingContext::NotFound&){
      cerr << "ERROR :  CinternalCorbaObject::getModelLoader : "<< modelLoaderString <<  "NOT FOUND" << endl;
      return KD_ERROR;
    }
  } catch (CORBA::SystemException &ex) {
    cerr << "System exception(" << ex._rep_id() 
	 << ") in CinternalCorbaObject::getModelLoader()" <<endl;
    return KD_ERROR;
  } catch (...){
    cerr << "Unknown exception" << endl;
    return KD_ERROR;
  }
  return KD_OK;
}


ktStatus ChppciOpenHrpClient::getRobotURL()
{
  if (privateCorbaObject->getModelLoader() != KD_OK){
      return KD_ERROR;
  }
  
  if(privateCorbaObject->attLoader) {
    //Get HRP2 Information 
    std::string url("file://");
    url += std::string(STRING_OPENHRP_PREFIX);
    url += std::string("/etc/HRP2JRL/HRP2JRLmain.wrl");

    privateCorbaObject->HRP2info = privateCorbaObject->attLoader->loadURL(url.c_str());
    
    // TO CHECK ON SCREEN
    cout << endl ;
    cout << "Loaded URL        : " << privateCorbaObject->HRP2info->getUrl()<<endl;
    cout << "Loaded Model Name : " << privateCorbaObject->HRP2info->getCharObject()->name() << endl ; 
    cout << "Loaded Model Size : " << privateCorbaObject->HRP2info->getCharObject()->modelObjectSeq()->length() << endl;
  }
  return KD_OK;
}

// ==============================================================================

ktStatus ChppciOpenHrpClient::getObstacleURL(std::string inFilename)
{
  if (privateCorbaObject->getModelLoader() != KD_OK){
      return KD_ERROR;
  }
  
  if(privateCorbaObject->attLoader) {   
    try {
      privateCorbaObject->obstInfoVector.push_back(privateCorbaObject->attLoader->loadURL(inFilename.c_str()));
      int sz_obst = privateCorbaObject->obstInfoVector.size();
	                 
      // TO CHECK ON SCREEN
      cout << endl ;
      cout << "Vector Obstacle " << 0 << endl ; 
      cout << "Loaded URL        : " << privateCorbaObject->obstInfoVector[0]->getUrl() << endl;
      cout << "Loaded Model Name : " << privateCorbaObject->obstInfoVector[0]->getCharObject()->name() << endl ; 
      cout << "Loaded Model Size : " << privateCorbaObject->obstInfoVector[0]->getCharObject()->modelObjectSeq()->length() << endl ;
    } catch (ModelLoader::ModelLoaderException& exception) {
      std::cerr << "ChppciOpenHrpClient::getObstacleURL: could not open file" << std::endl;
      return KD_ERROR;
    }
  } 
  return KD_OK;
}
  

