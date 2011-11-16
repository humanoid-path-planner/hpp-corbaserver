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
#include <string>

#include <robotbuilder/robotbuilder.hh>

#include <hpp/corbaserver/openhrp.hh>

#include <hpp/util/debug.hh>
#include <hpp/core/collision-pair.hh>
#include <hpp/model/body.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/specific-humanoid-robot.hh>
#include <hppOpenHRP/parserOpenHRPKineoDevice.h>
#include <hppOpenHRP/parserOpenHRPKineoObstacle.h>

#include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>

#include <hrp2-14/hrp2_14.h>

// CORBA stubs.
#include "common-modelloader.hh"
#include "jrl-modelloader.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      static void makeColPair(ChppColPair &cp);
      static void setHRP2OuterLists(model::HumanoidRobotShPtr i_hrp2);

      // Note : this class has been created because "common.hh" and "modelloader.hh"
      // should not appear in openhrp.hh
      class InternalCorbaObject
      {
      public:
	/// Corba elements.
	CORBA::ORB_var orb;
	/// Corba elements.
	CORBA::Object_var nameService;

	// HRP2 Corba Information
	ModelInfo_var HRP2info;

	// Obstacle Corba Information
	std::vector<ModelInfo_var> obstInfoVector;

	ChppciOpenHrpClient* attOpenHrpClient;


	// Constructor
	InternalCorbaObject (ChppciOpenHrpClient* openHrpClientPtr)
	{
	  attOpenHrpClient = openHrpClientPtr;
	}

	ktStatus getModelLoader ();

	ModelLoader_var attLoader;
      };



      ChppciOpenHrpClient::ChppciOpenHrpClient (core::Planner *hpp)
      {
	hppPlanner = hpp;
	privateCorbaObject = new InternalCorbaObject(this) ;
      }



      ChppciOpenHrpClient::~ChppciOpenHrpClient()
      {}


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

      void makeColPair (ChppColPair &cp)
      {
	// BODY:  +ARM2  +LEG2
	cp.addColPairRange (BODY, RARM_LINK2, RARM_LINK6);
	cp.addColPairRange (BODY, LARM_LINK2, LARM_LINK6);
	cp.addColPairRange (BODY, RHAND_LINK0, RHAND_LINK4);
	cp.addColPairRange (BODY, LHAND_LINK0, LHAND_LINK4);
	cp.addColPairRange (BODY, RLEG_LINK2, RLEG_LINK5);
	cp.addColPairRange (BODY, LLEG_LINK2, LLEG_LINK5);
	cp.addColPairRange (CHEST_LINK0, RARM_LINK3, RARM_LINK6);
	// BODY + CHEST_JOINT1
	cp.addColPair(BODY, CHEST_LINK1);

	// CHEST0  +ARM3  +LEG3
	cp.addColPairRange (CHEST_LINK0, RARM_LINK3, RARM_LINK6);
	cp.addColPairRange (CHEST_LINK0, LARM_LINK3, LARM_LINK6);
	cp.addColPairRange (CHEST_LINK0, RHAND_LINK0, RHAND_LINK4);
	cp.addColPairRange (CHEST_LINK0, LHAND_LINK0, LHAND_LINK4);
	cp.addColPairRange (CHEST_LINK0, RLEG_LINK3, RLEG_LINK5);
	cp.addColPairRange (CHEST_LINK0, LLEG_LINK3, LLEG_LINK5);

	// CHEST1  +ARM2  +LEG3
	cp.addColPairRange (CHEST_LINK1, RARM_LINK2, RARM_LINK6);
	cp.addColPairRange (CHEST_LINK1, LARM_LINK2, LARM_LINK6);
	cp.addColPairRange (CHEST_LINK1, RHAND_LINK0, RHAND_LINK4);
	cp.addColPairRange (CHEST_LINK1, LHAND_LINK0, LHAND_LINK4);
	cp.addColPairRange (CHEST_LINK1, RLEG_LINK3, RLEG_LINK5);
	cp.addColPairRange (CHEST_LINK1, LLEG_LINK3, LLEG_LINK5);

	// HEAD0  +ARM4
	cp.addColPairRange (HEAD_LINK0, RARM_LINK4, RARM_LINK6);
	cp.addColPairRange (HEAD_LINK0, LARM_LINK4, LARM_LINK6);
	cp.addColPairRange (HEAD_LINK0, RHAND_LINK0, RHAND_LINK4);
	cp.addColPairRange (HEAD_LINK0, LHAND_LINK0, LHAND_LINK4);

	// HEAD1  +ARM4
	cp.addColPairRange (HEAD_LINK1, RARM_LINK4, RARM_LINK6);
	cp.addColPairRange (HEAD_LINK1, LARM_LINK4, LARM_LINK6);
	cp.addColPairRange (HEAD_LINK1, RHAND_LINK0, RHAND_LINK4);
	cp.addColPairRange (HEAD_LINK1, LHAND_LINK0, LHAND_LINK4);

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
	cp.addColPairRange (RLEG_LINK0, RLEG_LINK4, RLEG_LINK5);
	cp.addColPair(RLEG_LINK0, LLEG_LINK0);
	cp.addColPairRange (RLEG_LINK0, LLEG_LINK2, LLEG_LINK5);
	cp.addColPairRange (RLEG_LINK0, RARM_LINK3, RARM_LINK6);
	cp.addColPairRange (RLEG_LINK0, LARM_LINK3, LARM_LINK6);
	cp.addColPairRange (RLEG_LINK0, RHAND_LINK0, RHAND_LINK4);
	cp.addColPairRange (RLEG_LINK0, LHAND_LINK0, LHAND_LINK4);

	// LEG2
	cp.addColPairRange (RLEG_LINK2, LLEG_LINK2, LLEG_LINK5);

	// LEG3
	// pair leg3 and leg5 excluded.
	// addColPair(RLEG_LINK3, RLEG_LINK5, cp);
	cp.addColPair(RLEG_LINK3, LLEG_LINK0);
	cp.addColPairRange (RLEG_LINK3, LLEG_LINK2, LLEG_LINK3);
	// leg4 no need
	cp.addColPair(RLEG_LINK3, LLEG_LINK5);

	// LEG4 no need.
	// addColPair(RLEG_LINK4, LLEG_LINK0, cp);

	// LEG5
	cp.addColPair(RLEG_LINK5, LLEG_LINK0);
	// addColPairRange (RLEG_LINK5, LLEG_LINK2, LLEG_LINK5, cp);
	cp.addColPairRange (RLEG_LINK5, LLEG_LINK2, LLEG_LINK3);
	cp.addColPair(RLEG_LINK5, LLEG_LINK5);

	// for arm/hands
	for(unsigned int j=RLEG_LINK2; j<=RLEG_LINK5; j++){
	  cp.addColPairRange (j, RARM_LINK2, RARM_LINK6);
	  cp.addColPairRange (j, LARM_LINK3, LARM_LINK6);
	  cp.addColPairRange (j, RHAND_LINK0, RHAND_LINK4);
	  cp.addColPairRange (j, LHAND_LINK0, LHAND_LINK4);
	}


	// Left legs
	// LEG0
	cp.addColPairRange (LLEG_LINK0, LLEG_LINK4, LLEG_LINK5);
	cp.addColPair(LLEG_LINK0, RLEG_LINK0);
	cp.addColPairRange (LLEG_LINK0, RLEG_LINK2, RLEG_LINK5);
	cp.addColPairRange (LLEG_LINK0, RARM_LINK3, RARM_LINK6);
	cp.addColPairRange (LLEG_LINK0, LARM_LINK3, LARM_LINK6);
	cp.addColPairRange (LLEG_LINK0, RHAND_LINK0, RHAND_LINK4);
	cp.addColPairRange (LLEG_LINK0, LHAND_LINK0, LHAND_LINK4);

	// LEG2
	cp.addColPairRange (LLEG_LINK2, RLEG_LINK2, RLEG_LINK5);

	// LEG3
	// pair leg3 and leg5 excluded.
	// addColPair(LLEG_LINK3, LLEG_LINK5, cp);
	cp.addColPair(LLEG_LINK3, RLEG_LINK0);
	// addColPairRange (LLEG_LINK3, RLEG_LINK2, RLEG_LINK5, cp);
	cp.addColPairRange (LLEG_LINK3, RLEG_LINK2, RLEG_LINK3);
	cp.addColPair(LLEG_LINK3, RLEG_LINK5);

	// LEG4 no need.
	// addColPair(LLEG_LINK4, RLEG_LINK0, cp);

	// LEG5 leg4 exluded.
	cp.addColPair(LLEG_LINK5, RLEG_LINK0);
	// addColPairRange (LLEG_LINK5, RLEG_LINK2, RLEG_LINK5, cp);
	cp.addColPairRange (LLEG_LINK5, RLEG_LINK2, RLEG_LINK3);
	cp.addColPair(LLEG_LINK5, RLEG_LINK5);

	// for arm/hands
	for(unsigned int j=LLEG_LINK2; j<=LLEG_LINK5; j++){
	  cp.addColPairRange (j, RARM_LINK2, RARM_LINK6);
	  cp.addColPairRange (j, LARM_LINK2, LARM_LINK6);
	  cp.addColPairRange (j, RHAND_LINK0, RHAND_LINK4);
	  cp.addColPairRange (j, LHAND_LINK0, LHAND_LINK4);
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
	cp.addColPairRange (RARM_LINK0, RARM_LINK4, RARM_LINK6);
	cp.addColPairRange (RARM_LINK0, RHAND_LINK0, RHAND_LINK4);
	cp.addColPairRange (RARM_LINK0, LARM_LINK4, LARM_LINK6);
	cp.addColPairRange (RARM_LINK0, LHAND_LINK0, LHAND_LINK4);

	// LINK1
	cp.addColPairRange (RARM_LINK1, LARM_LINK3, LARM_LINK6);
	cp.addColPairRange (RARM_LINK1, LHAND_LINK0, LHAND_LINK4);

	// LINK2
	cp.addColPairRange (RARM_LINK2, LARM_LINK3, LARM_LINK6);
	cp.addColPairRange (RARM_LINK2, LHAND_LINK0, LHAND_LINK4);

	// LINK3 - LINK6, HAND
	for(unsigned int j=RARM_LINK3; j<=RARM_LINK6; j++){
	  cp.addColPairRange (j, LARM_LINK2, LARM_LINK6);
	  cp.addColPairRange (j, LHAND_LINK0, LHAND_LINK4);
	}
	for(unsigned int j=RHAND_LINK0; j<=RHAND_LINK4; j++){
	  cp.addColPairRange (j, LARM_LINK2, LARM_LINK6);
	  cp.addColPairRange (j, LHAND_LINK0, LHAND_LINK4);
	}

	// Left arm/hands

	// LINK0
	cp.addColPairRange (LARM_LINK0, LARM_LINK4, LARM_LINK6);
	cp.addColPairRange (LARM_LINK0, LHAND_LINK0, LHAND_LINK4);
	cp.addColPairRange (LARM_LINK0, RARM_LINK4, RARM_LINK6);
	cp.addColPairRange (LARM_LINK0, RHAND_LINK0, RHAND_LINK4);

	// LINK1
	cp.addColPairRange (LARM_LINK1, RARM_LINK3, RARM_LINK6);
	cp.addColPairRange (LARM_LINK1, RHAND_LINK0, RHAND_LINK4);

	// LINK2
	cp.addColPairRange (LARM_LINK2, RARM_LINK3, RARM_LINK6);
	cp.addColPairRange (LARM_LINK2, RHAND_LINK0, RHAND_LINK4);

	// LINK3 - LINK6, HAND
	for(unsigned int j=LARM_LINK3; j<=LARM_LINK6; j++){
	  cp.addColPairRange (j, RARM_LINK2, RARM_LINK6);
	  cp.addColPairRange (j, RHAND_LINK0, RHAND_LINK4);
	}
	for(unsigned int j=LHAND_LINK0; j<=LHAND_LINK4; j++){
	  cp.addColPairRange (j, RARM_LINK2, RARM_LINK6);
	  cp.addColPairRange (j, RHAND_LINK0, RHAND_LINK4);
	}
      }

      void setHRP2OuterLists (model::HumanoidRobotShPtr i_hrp2)
      {
	ChppColPair hrpCP;
	makeColPair (hrpCP);

	std::vector<CkwsJointShPtr> jv;
	i_hrp2->getJointVector (jv);

	// adding outer objects
	for(unsigned int iJoint = 0; iJoint < jv.size (); ++iJoint)
	  {
	    model::BodyShPtr hppBody;
	    hppBody = KIT_DYNAMIC_PTR_CAST(model::Body, jv[iJoint]->attachedBody());

	    std::vector<CkcdObjectShPtr> mergedList;

	    for (unsigned int jJoint=0; jJoint < jv.size (); ++jJoint)
	      {
		model::BodyShPtr hppOtherBody =
		  KIT_DYNAMIC_PTR_CAST(model::Body, jv[jJoint]->attachedBody());

		if (jJoint != iJoint && hrpCP.existPairNarrow(iJoint, jJoint) )
		  {
		    std::vector<CkcdObjectShPtr> clist;
		    clist = hppOtherBody->innerObjects();
		    for (unsigned int k = 0; k < clist.size (); ++k)
		      {
			CkppKCDPolyhedronShPtr kppPolyhedron = KIT_DYNAMIC_PTR_CAST(CkppKCDPolyhedron, clist[k]);
			mergedList.push_back(kppPolyhedron);
		      }
		  }
	      }

	  // add to outerObject.
	  for (std::vector<CkcdObjectShPtr>::iterator it = mergedList.begin();
	       it < mergedList.end ();
	       ++it)
	    hppBody->addOuterObject(*it);
	  }
      }

      ktStatus
      ChppciOpenHrpClient::loadHrp2Model (double inPenetration,
			      const std::string& inModel)
      {
	model::HumanoidRobotShPtr HRP2Device;
	if (loadHrp2Model(HRP2Device, inModel) != KD_OK){
	  return KD_ERROR;
	}

	//
	// ADD HRP2 to the planner
	//
	if (hppPlanner->addHppProblem (HRP2Device, inPenetration) != KD_OK)
	  {
	    hppDout (error, "failed to add robot");
	    return KD_ERROR;
	  }

	return KD_OK;
      }

      ktStatus
      ChppciOpenHrpClient::loadHrp2Model
      (model::HumanoidRobotShPtr& HRP2Device, const std::string& inModel)
      {
	model::HumanoidRobotShPtr humanoid;
	Robotbuilder::Robotbuilder robotBuilder;
	humanoid = robotBuilder.makeRobot();
	Chrp2OptHumanoidDynamicRobot *hrp2;
	hrp2 = dynamic_cast<Chrp2OptHumanoidDynamicRobot *>(humanoid.get());
	hrp2->isKineoOrder(true);
	HRP2Device = humanoid;
	if (!HRP2Device)
	  {
	    hppDout (error, "failed to build Kineo HRP2 Model");
	    return KD_ERROR;
	  }

	//
	// set HRP2 joints in invisible mode by default
	//
	HRP2Device->isVisible (false);

	// set Collision Check Pairs
	setHRP2OuterLists (humanoid);

	//
	// set HRP2 in a default configuration (HALFSITTING) ;
	//
	CkwsConfig halfSittingConfig (HRP2Device);
	double dInitPos[] = HALFSITTINGPOSITION_RAD_KINEO;
	std::vector<double>  halfSittingVector (dInitPos, dInitPos + sizeof(dInitPos) / sizeof(double));
	halfSittingConfig.setDofValues (halfSittingVector);
	HRP2Device->hppSetCurrentConfig (halfSittingConfig);

	return KD_OK;
      }



      ktStatus
      ChppciOpenHrpClient::loadRobotModel(const std::string& inFilename,
			      const std::string& inDeviceName,
			      model::DeviceShPtr &outDevice,
			      const std::string& inDirectory)
      {
	ModelLoader_var loader;
	if (privateCorbaObject->getModelLoader() != KD_OK)
	  {
	    hppDout (error, "failed to load robot model");
	    return KD_ERROR;
	  }

	//
	// Build hppDevice from ChppciOpenHrpClientModel
	//
	std::string url ("file://");
	url += inDirectory;
	url += std::string ("/");
	url += inFilename;

	hppDout (info, "reading " << url);

	try
	  {
	    CparserChppciOpenHrpClientKineoDevice parser(privateCorbaObject->attLoader->loadURL(url.c_str())) ;
	    model::HumanoidRobotShPtr humanoid;
	    parser.parser(humanoid) ;
	    outDevice = humanoid;
	  }
	catch (CORBA::SystemException &ex)
	  {
	    hppDout (error, "system exception (" << ex._rep_id () << ")");
	    return KD_ERROR;
	  }
	catch (...)
	  {
	    hppDout (error, "unknown exception");
	    return KD_ERROR;
	  }

	if (!outDevice)
	  {
	    hppDout (error, "failed to build Kineo Robot Model");
	    return KD_ERROR;
	  }

	//
	// set joints in invisible mode by default
	//
	outDevice->isVisible (false);
	outDevice->name (inDeviceName);
	return KD_OK;
      }



      ktStatus
      ChppciOpenHrpClient::loadObstacleModel (const std::string& inFilename,
				  const std::string& inObstacleName,
				  CkppKCDPolyhedronShPtr& outPolyhedron,
				  const std::string& inDirectory)
      {
	//
	// Get Corba objects containing model of Obstacles.
	//
	std::string url ("file://");
	url += inDirectory;
	url += std::string ("/");
	url += inFilename;

	if (getObstacleURL(url)!= KD_OK)
	  {
	    privateCorbaObject->orb->destroy();
	    hppDout (error, "failed to load Obstacle model");
	    return KD_ERROR;
	  }

	//
	// build obstacles (if there is some obstacle)
	//
	if (!privateCorbaObject->obstInfoVector.empty ())
	  {
	    //
	    // Build KineoObstacle from ChppciOpenHrpClientModel
	    //
	    CparserChppciOpenHrpClientKineoObstacle parserObstacle (privateCorbaObject->obstInfoVector);

	    std::vector<CkppKCDPolyhedronShPtr> obstacleVector = parserObstacle.parser ();

	    if (obstacleVector.empty ())
	      {
		hppDout (error, "failed to parse the obstacle model");
		return KD_ERROR;
	      }
	    outPolyhedron = obstacleVector[0];
	    outPolyhedron->name (inObstacleName);
	  }
	return KD_OK;
      }

      ktStatus
      InternalCorbaObject::getModelLoader ()
      {
	// Services
	std::string modelLoaderString ("ModelLoaderJRL");
	std::string nameServiceString ("NameService");

	try
	  {
	  int argc = 1;
	  char *argv[1] = {"unused"};
	  // ORB Creation.
	  orb = CORBA::ORB_init (argc, argv);

	  // Getting reference of name server
	  try
	    {
	      nameService = orb->resolve_initial_references(nameServiceString.c_str());
	    }
	  catch (const CORBA::ORB::InvalidName&)
	    {
	      hppDout (error, "cannot resolve " << nameServiceString);
	      return KD_ERROR;
	    }
	  if (CORBA::is_nil (nameService))
	    {
	      hppDout (info, nameServiceString << "is a nil object reference");
	      return KD_ERROR;
	    }

	  // Getting root naming context
	  CosNaming::NamingContext_var cxt =
	    CosNaming::NamingContext::_narrow(nameService);
	  if (CORBA::is_nil(cxt))
	    {
	      hppDout (error, nameServiceString
		       << " is not a NamingContext object reference");
	      return KD_ERROR;
	    }

	  CosNaming::Name ncFactory;
	  ncFactory.length (1);

	  // modelloaderJRL
	  CosNaming::Name mFactory;
	  mFactory.length (1);
	  mFactory[0].id = CORBA::string_dup (modelLoaderString.c_str ());
	  mFactory[0].kind = CORBA::string_dup ("");

	  try
	    {
	      attLoader = ModelLoader::_narrow (cxt -> resolve (mFactory));
	    }
	  catch (const CosNaming::NamingContext::NotFound&)
	    {
	      hppDout (error, modelLoaderString <<  " not found");
	      return KD_ERROR;
	    }
	  }
	catch (CORBA::SystemException &ex)
	  {
	    hppDout (error, "system exception (" << ex._rep_id() << ")");
	    return KD_ERROR;
	  }
	catch (...)
	  {
	    hppDout (error, "unknown exception");
	    return KD_ERROR;
	  }
	return KD_OK;
      }

      ktStatus ChppciOpenHrpClient::getRobotURL (const std::string& inModel)
      {
	if (privateCorbaObject->getModelLoader() != KD_OK)
	  return KD_ERROR;

	if(privateCorbaObject->attLoader)
	  {
	    //Get HRP2 Information
	    std::string url ("file://");
	    url += inModel;
	    privateCorbaObject->HRP2info = privateCorbaObject->attLoader->loadURL (url.c_str ());

	    hppDout (info,
		     "loaded URL: "
		     << privateCorbaObject->HRP2info->getUrl ());
	    hppDout
	      (info,
	       "loaded Model Name: "
	       << privateCorbaObject->HRP2info->getCharObject ()->name ());
	    hppDout
	      (info, "loaded Model Size: "
	       << privateCorbaObject->HRP2info->getCharObject ()->modelObjectSeq ()->length ());
	}
	return KD_OK;
      }

      ktStatus ChppciOpenHrpClient::getObstacleURL (const std::string& inFilename)
      {
	if (privateCorbaObject->getModelLoader() != KD_OK)
	  return KD_ERROR;
	if(privateCorbaObject->attLoader)
	  {
	    try
	      {
		privateCorbaObject->obstInfoVector.push_back
		  (privateCorbaObject->attLoader->loadURL (inFilename.c_str ()));

		hppDout (info, "vector Obstacle " << 0);
		hppDout (info, "loaded URL: "
			 << privateCorbaObject->obstInfoVector[0]->getUrl ());
		hppDout
		  (info, "loaded Model Name: "
		   << privateCorbaObject->obstInfoVector[0]->getCharObject ()->name ());
		hppDout
		  (info, "loaded Model Size: "
		   << privateCorbaObject->obstInfoVector[0]->getCharObject ()->modelObjectSeq ()->length ());
	      }
	    catch (ModelLoader::ModelLoaderException& exception)
	      {
		hppDout (error, "failed to open obstacle file");
		return KD_ERROR;
	      }
	  }
	return KD_OK;
      }

    } // end of namespace implementation.
  } // end of namespace corbaServer.
} // end of namespace hpp.
