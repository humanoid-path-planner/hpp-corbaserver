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

#include <hpp/corbaserver/openhrp.hh>

#include <hpp/util/debug.hh>
#include <hpp/model/body.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/humanoid-robot.hh>

#include <hppOpenHRP/parserOpenHRPKineoDevice.h>
#include <hppOpenHRP/parserOpenHRPKineoObstacle.h>

#include <hpp/model/urdf/parser.hh>
#include <hpp/model/srdf/parser.hh>
#include <hpp/model/rcpdf/parser.hh>

// CORBA stubs.
#include "common-modelloader.hh"
#include "jrl-modelloader.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
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

	OpenHRP* attOpenHrpClient;


	// Constructor
	InternalCorbaObject (OpenHRP* openHrpClientPtr)
	{
	  attOpenHrpClient = openHrpClientPtr;
	}

	ktStatus getModelLoader ();

	ModelLoader_var attLoader;
      };



      OpenHRP::OpenHRP (core::Planner *hpp)
      {
	hppPlanner = hpp;
	privateCorbaObject = new InternalCorbaObject(this) ;
      }



      OpenHRP::~OpenHRP()
      {}

      ktStatus
      OpenHRP::loadHrp2Model (double inPenetration,
			      const std::string& inModel)
      {
	model::HumanoidRobotShPtr HRP2Device;
	if (loadHrp2Model(HRP2Device, inModel) != KD_OK){
	  return KD_ERROR;
	}

	//
	// ADD HRP2 to the planner
	//
      }

      ktStatus
      OpenHRP::loadHrp2Model
      (model::HumanoidRobotShPtr& HRP2Device, const std::string& inModel)
      {
	model::HumanoidRobotShPtr hrp2;
	hpp::model::urdf::Parser parser;
	hpp::model::srdf::Parser srdfParser;
	hpp::model::rcpdf::Parser rcpdfParser;
	hrp2 = parser.parse ("package://hrp2_14_description/urdf/hrp2_capsule.urdf");

	HRP2Device = hrp2;
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
	srdfParser.parse ("package://hrp2_14_description/urdf/hrp2_capsule.urdf",
			  "package://hrp2_14_description/srdf/hrp2_capsule.srdf",
			  hrp2);

	//
	// set HRP2 in a default configuration (HALFSITTING) ;
	//
	hpp::model::srdf::Parser::HppConfigurationType halfSittingConfig
	  = srdfParser.getHppReferenceConfig ("all", "half_sitting");

	HRP2Device->hppSetCurrentConfig (halfSittingConfig);

	// Parse contact points to set foot size.
	rcpdfParser.parse ("package://hrp2_14_description/rcpdf/hrp2.rcpdf",
			   hrp2);

	return KD_OK;
      }

      ktStatus
      OpenHRP::loadRobotModel(const std::string& inFilename,
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
	// Build hppDevice from OpenHRPModel
	//
	std::string url ("file://");
	url += inDirectory;
	url += std::string ("/");
	url += inFilename;

	hppDout (info, "reading " << url);

	try
	  {
	    CparserOpenHRPKineoDevice parser(privateCorbaObject->attLoader->loadURL(url.c_str())) ;
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
      OpenHRP::loadObstacleModel (const std::string& inFilename,
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
	    // Build KineoObstacle from OpenHRPModel
	    //
	    CparserOpenHRPKineoObstacle parserObstacle (privateCorbaObject->obstInfoVector);

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

      ktStatus OpenHRP::getRobotURL (const std::string& inModel)
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

      ktStatus OpenHRP::getObstacleURL (const std::string& inFilename)
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
