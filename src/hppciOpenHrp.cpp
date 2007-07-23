
/*************************************
INCLUDE
**************************************/
#include <iostream>

#include "hppciOpenHrp.h"

#include "common-modelloader.hh"
#include "jrl-modelloader.hh"

#include "hppOpenHRP/parserOpenHRPKineoHRP2.h"
#include "hppOpenHRP/parserOpenHRPKineoObstacle.h"


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

  ktStatus getModelLoader(int argc, char **argv, ModelLoader_var& outLoader);

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

ktStatus ChppciOpenHrpClient::loadHrp2Model(int argc, char **argv, 
					    ChppDeviceShPtr &HRP2Device)
{
  // 
  // Get Corba objects containing model of HRP2 and Obstacles.
  //
  if (getInfoFromCorba(argc, argv)!= KD_OK) {
    privateCorbaObject->orb->destroy();
    cerr << "ERROR : ChppciOpenHrpClient::loadHrp2Model::Failed to load Hrp2 model" 
	 << endl;
    return KD_ERROR;
  }

  //
  // Build KineoHRP2 from OpenHRPModel
  //
  CparserOpenHRPKineoHRP2 parserHRP2(privateCorbaObject->HRP2info) ;
  HRP2Device=parserHRP2.parser() ;
  if (!HRP2Device) {
    cerr << " ERROR : ChppciOpenHrpClient::loadHrp2Model : Failed building Kineo HRP2 Model" << endl ;
    return KD_ERROR;
  }
  
  //
  // set HRP2 joints in invisible mode by default
  //
  HRP2Device->isVisible(false) ;

  //
  // set HRP2 in a default configuration (HALFSITTING) ;
  //
  CkwsConfigShPtr halfSittingConfig = CkwsConfig::create(HRP2Device) ;
  double dInitPos[46] = HALFSITTINGPOSITION_RAD_KINEO ;
  std::vector<double>  halfSittingVector (dInitPos  , dInitPos + sizeof(dInitPos) / sizeof(double) );
  halfSittingConfig->setDofValues( halfSittingVector ) ; 
  HRP2Device->applyCurrentConfig(halfSittingConfig) ;

  return KD_OK;
}




ktStatus ChppciOpenHrpClient::loadRobotModel(int argc, char **argv, 
					     ChppDeviceShPtr &o_device)
{
  ModelLoader_var loader;
  if (privateCorbaObject->getModelLoader(argc, argv, loader) != KD_OK){
    cerr << "ERROR : ChppciOpenHrpClient::loadRobotModel::Failed to load robot model" 
	 << endl;
    return KD_ERROR;
  }
	
  int url_idx=0;
  for (unsigned int i=0; (int)i<argc; i++){
    if (strcmp("-vrml", argv[i])==0){
      url_idx = ++i;
      break;
    }
  }
  url_idx += 1; // skip name  

  //
  // Build hppDevice from OpenHRPModel
  //
  try{
    CparserOpenHRPKineoDevice parser(loader->loadURL(argv[url_idx])) ;
    o_device=parser.parser() ;
  } catch (CORBA::SystemException &ex) {
    cerr << "System exception: " << ex._rep_id() << endl;
    return KD_ERROR;
  } catch (...){
    cerr << "Unknown exception" << endl;
    return KD_ERROR;
  }
  if (!o_device) {
    cerr << " ERROR : ChppciOpenHrpClient::loadRobotModel : Failed building Kineo Robot Model" << endl ;
    return KD_ERROR;
  }
  
  //
  // set joints in invisible mode by default
  //
  o_device->isVisible(false) ;

  privateCorbaObject->orb->destroy();

  return KD_OK;
}




ktStatus  ChppciOpenHrpClient::loadHrp2Model(int argc, char **argv)
{
  ChppDeviceShPtr HRP2Device;
  if (loadHrp2Model(argc, argv, HRP2Device) != KD_OK){
    return KD_ERROR;
  }
  
  //
  // ADD HRP2 to the planner
  // 
  if (hppPlanner->addHppProblem(HRP2Device) != KD_OK) {
    cerr << "ChppciOpenHrpClient::loadHrp2Model: Failed to add robot" << endl;
    return KD_ERROR;
  }

  //
  // build obstacles (if there is some obstacle)
  //
  if (privateCorbaObject->obstInfoVector.size() > 0){  


    //
    // Build KineoObstacle from OpenHRPModel
    //
    CparserOpenHRPKineoObstacle parserObstacle(privateCorbaObject->obstInfoVector) ;
    vector<ChppPolyhedronShPtr> obstacleVector = parserObstacle.parser() ;
    if (obstacleVector.size() == 0) {
      cerr << "ERROR ChppciOpenHrpClient::loadHrp2Model : Failed to parse the obstacle model " <<  endl;
      return KD_ERROR;
    }

    //
    // ADD Obstacle to the planner
    //
    for(unsigned int i=0; i < obstacleVector.size(); i++){
      if (hppPlanner->addObstacle(obstacleVector[i]) != KD_OK) {
	cerr << "ChppciOpenHrpClient::loadHrp2Model: Failed ta add obstacle in hppPlanner" << endl;
	return KD_ERROR;
      }
    }
  }

  privateCorbaObject->orb->destroy();

  return KD_OK;
 
}

// ==============================================================================

ktStatus  ChppciOpenHrpClient::loadObstacleModel(int argc, char **argv, 
						 std::vector<ChppPolyhedronShPtr> &obstacleVector)
{
 
  // 
  // Get Corba objects containing model of HRP2 and Obstacles.
  //
  if (getInfoFromCorba(argc, argv)!= KD_OK) {
    privateCorbaObject->orb->destroy();
    cerr << "ERROR : ChppciOpenHrpClient::loadHrp2Model::Failed to load Hrp2 model" 
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
    CparserOpenHRPKineoObstacle parserObstacle(privateCorbaObject->obstInfoVector) ;
    obstacleVector = parserObstacle.parser() ;
    if (obstacleVector.size() == 0) {
      cerr << "ERROR ChppciOpenHrpClient::loadHrp2Model : Failed to parse the obstacle model " <<  endl;
      return KD_ERROR;
    }

  }

  privateCorbaObject->orb->destroy();


  return KD_OK;
}


  
ktStatus CinternalCorbaObject::getModelLoader(int argc, char **argv, 
					      ModelLoader_var& outLoader)
{
  // Services
  string modelLoaderString="ModelLoaderJRL" ;
  string nameServiceString="NameService" ;

  try {
    // ORB Creation.
    orb = CORBA::ORB_init(argc, argv);

    // Getting reference of name server
    try {
      nameService = orb->resolve_initial_references(nameServiceString.c_str());
    } catch (const CORBA::ORB::InvalidName&) {
      cerr << "ERROR :ChppciOpenHrpClient::getModelLoader : " << argv[0] << ": can't resolve " << nameServiceString << endl;
      return KD_ERROR;
    }
    if(CORBA::is_nil(nameService)) {
      cerr << argv[0]
	   << " ERROR :ChppciOpenHrpClient::getModelLoader "<< nameServiceString << "is a nil object reference"
	   << endl;
      return KD_ERROR;
    }

    // Getting root naming context
    CosNaming::NamingContext_var cxt = 
      CosNaming::NamingContext::_narrow(nameService);
    if(CORBA::is_nil(cxt)) {
      cerr << argv[0]
	   << " ERROR :ChppciOpenHrpClient::getModelLoader "<< nameServiceString << " is not a NamingContext object reference"
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
      outLoader = ModelLoader::_narrow(cxt -> resolve(mFactory));
    } catch (const CosNaming::NamingContext::NotFound&){
      cerr << "ERROR :  ChppciOpenHrpClient::getModelLoader : "<< modelLoaderString <<  "NOT FOUND" << endl;
      return KD_ERROR;
    }
  } catch (CORBA::SystemException &ex) {
    cerr << "System exception: " << ex._rep_id() << endl;
    return KD_ERROR;
  } catch (...){
    cerr << "Unknown exception" << endl;
    return KD_ERROR;
  }
  return KD_OK;
}

// ==============================================================================

ktStatus ChppciOpenHrpClient::getInfoFromCorba(int argc, char **argv)
{
  int vrml_idx=0;
  for (unsigned int i=0; (int)i<argc; i++){
    if (strcmp("-vrml", argv[i])==0){
      vrml_idx = ++i;
    }
  }

  ModelLoader_var loader;
  if (privateCorbaObject->getModelLoader(argc, argv, loader) != KD_OK){
      return KD_ERROR;
  }

  try{
    if (vrml_idx>0){
      for (unsigned int i=vrml_idx;(int)i<argc-1;){ 
	
	cout << " ---------------------------------------- " << endl ;
	cout << "Character argv    : " << argv[i] << endl;
	cout << "Model     argv    : " << argv[i+1] << endl;
	

	//
	// GET INFORMATIONS
	//
	if(loader) {
	  if(strstr(argv[i],"HRP2") != NULL){ 
	    
	    //Get HRP2 Information 
	    privateCorbaObject->HRP2info = loader->loadURL(argv[i+1]);
	    
	    // TO CHECK ON SCREEN
            cout << endl ;
	    cout << "Loaded URL        : " << privateCorbaObject->HRP2info->getUrl()<<endl;
	    cout << "Loaded Model Name : " << privateCorbaObject->HRP2info->getCharObject()->name() << endl ; 
	    cout << "Loaded Model Size : " << privateCorbaObject->HRP2info->getCharObject()->modelObjectSeq()->length() << endl;
	  }
	  else {  
	   	    
	    // Get Obstacle Information
	    privateCorbaObject->obstInfoVector.push_back(loader->loadURL(argv[i+1]));
	    int sz_obst = privateCorbaObject->obstInfoVector.size();
	    int idLastObst = sz_obst - 1 ;

	                 
	    // TO CHECK ON SCREEN
	    cout << endl ;
	    cout << "Vector Obstacle " << idLastObst << endl ; 
	    cout << "Loaded URL        : " << privateCorbaObject->obstInfoVector[idLastObst]->getUrl() << endl;
	    cout << "Loaded Model Name : " << privateCorbaObject->obstInfoVector[idLastObst]->getCharObject()->name() << endl ; 
	    cout << "Loaded Model Size : " << privateCorbaObject->obstInfoVector[idLastObst]->getCharObject()->modelObjectSeq()->length() << endl ;
	  }
	}
	else
	  cout<<"ERROR :  ChppciOpenHrpClient::getInfoFromCorba : LOADER NULL"<<endl;
	i+=2;
      }
    }
    
    cout << " ---------------------------------------- " << endl ;

  } catch (CORBA::SystemException &ex) {
    cerr << "System exception: " << ex._rep_id() << endl;
    return KD_ERROR;
  } catch (...){
    cerr << "Unknown exception" << endl;
    return KD_ERROR;
  }
  
  return KD_OK;
}
  
