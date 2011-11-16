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

#include <KineoWorks2/kwsIPPRdmBuilder.h>

#include <kwsPlus/directPath/flicSteeringMethod.h>
#include <kwsPlus/directPath/reedsSheppSteeringMethod.h>
#include <kwsPlus/roadmap/hppVisRdmBuilder.h>
#include <kwsPlus/roadmap/kwsPlusPCARdmBuilder.h>

#include <hpp/util/debug.hh>
#include <hpp/util/portability.hh>

#include "hpp/corbaserver/server.hh"

#include "problem.impl.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      namespace
      {
	static bool
	checkChppciProblem_implId (const core::Planner& planner, unsigned id);

	static bool
	checkChppciProblem_implId (const core::Planner& planner, unsigned id)
	{
	  if (id >= planner.getNbHppChppciProblem_impls ())
	    {
	      hppDout (error, "invalid problem id");
	      return false;
	    }
	  return true;
	}

      } // end of namespace.


      ChppciProblem_impl::ChppciProblem_impl (corbaServer::Server* server)
	: server_ (server),
	  planner_ (server->planner ())
      {}

      Short
      ChppciProblem_impl::setSteeringMethod
      (UShort problemId, const char* steeringMethodName, Boolean oriented)
      {
	assert (planner_);
	if (!checkChppciProblem_implId (*planner_, problemId))
	  return -1;

	// Get robot in hppPlanner object.
	CkppDeviceComponentShPtr robot = planner_->robotIthChppciProblem_impl (problemId);

	/* Check that name correspond to a steering method factory */
	if (!server_->steeringMethodFactoryAlreadySet (steeringMethodName))
	  {
	    hppDout (error, "invalid steering method");
	    return -1;
	  }

	// Create steering method
	CkwsSteeringMethodShPtr steeringMethod =
	  server_->createSteeringMethod (steeringMethodName, oriented);

	robot->steeringMethod (steeringMethod);
	return 0;
      }

      Short ChppciProblem_impl::setRoadmapbuilder(UShort problemId, const char* inRoadmapBuilderName,
				       Boolean inDisplay)
      {
	std::string roadmapBuilderName(inRoadmapBuilderName);
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	ktStatus status;

	unsigned int nbChppciProblem_impls = planner_->getNbHppChppciProblem_impls();

	// Test that rank is less than number of robots in vector.
	if (hppChppciProblem_implId < nbChppciProblem_impls) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr robot = planner_->robotIthChppciProblem_impl(hppChppciProblem_implId);

	  // Create an empty roadmap for the robot.
	  CkwsRoadmapShPtr roadmap = CkwsRoadmap::create(robot->kwsDevice());
	  // Set arbitrary penetration.
	  double penetration = 0.05;

	  // Create roadmapBuilder
	  CkwsRoadmapBuilderShPtr roadmapBuilder;
	  if (roadmapBuilderName == "basic") {
	    roadmapBuilder = CkwsBasicRdmBuilder::create(roadmap, penetration);
	  } else if (roadmapBuilderName == "diffusing") {
	    roadmapBuilder = CkwsDiffusingRdmBuilder::create(roadmap, penetration);
	  } else if (roadmapBuilderName == "IPP") {
	    roadmapBuilder = CkwsIPPRdmBuilder::create(roadmap, penetration);
	  } else if (roadmapBuilderName == "visibility") {
	    roadmapBuilder = ChppVisRdmBuilder::create(roadmap, penetration);
	  } else if (roadmapBuilderName == "PCA<diffusing>") {
	    roadmapBuilder =
	      CkwsPlusPCARdmBuilder<CkwsDiffusingRdmBuilder>::create(roadmap, penetration);
	  } else {
	    hppDout (error, ":setRoadmapbuilder: unknown roadmap builder");
	    return -1;
	  }
	  status = planner_->roadmapBuilderIthChppciProblem_impl(hppChppciProblem_implId, roadmapBuilder, inDisplay);
	  return status;

	}
	else {
	  hppDout (error, ":setRoadmapbuilder: wrong problem Id");
	  return -1;
	}

	return 0;
      }

      Short ChppciProblem_impl::setDiffusingNode(UShort problemId,
				      const char* inDiffusingNode)
      {
	std::string diffusingNode(inDiffusingNode);
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int nbChppciProblem_impls = planner_->getNbHppChppciProblem_impls();

	// Test that rank is less than number of robots in vector.
	if (hppChppciProblem_implId < nbChppciProblem_impls) {
	  // Get roadmap builder in hppPlanner object.
	  CkwsDiffusingRdmBuilderShPtr roadmapBuilder =
	    KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder,
				 planner_->roadmapBuilderIthChppciProblem_impl(hppChppciProblem_implId));
	  // Check that diffusion roadmap builder is set
	  if (!roadmapBuilder) {
	    hppDout (error, ":setDiffusingNode: roadmap builder is not set or not of type diffusion");
	    return -1;
	  }
	  if (diffusingNode == "start") {
	    roadmapBuilder->diffuseFromChppciProblem_implStart(true);
	    roadmapBuilder->diffuseFromChppciProblem_implGoal(false);
	    hppDout (info, ":setDiffusingNode: diffusing from start");
	  }
	  else if (diffusingNode == "goal") {
	    roadmapBuilder->diffuseFromChppciProblem_implStart(false);
	    roadmapBuilder->diffuseFromChppciProblem_implGoal(true);
	    hppDout (info, ":setDiffusingNode: diffusing from goal");
	  }
	  else if (diffusingNode == "start and goal") {
	    roadmapBuilder->diffuseFromChppciProblem_implStart(true);
	    roadmapBuilder->diffuseFromChppciProblem_implGoal(true);
	    hppDout (info, ":setDiffusingNode: diffusing from start and goal");
	  }
	  else  {
	    hppDout (info, ":setDiffusingNode: unknown command " << diffusingNode);
	  }
	}
	else {
	  hppDout (error, ":setDiffusingNode: wrong problem Id");
	  return -1;
	}

	return 0;
      }

      Short ChppciProblem_impl::setPathOptimizer(UShort problemId,
				      const char* inPathOptimizerName,
				      UShort inMaxNumberLoop)
      {
	std::string pathOptimizerName(inPathOptimizerName);

	ktStatus status;
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int nbChppciProblem_impls = planner_->getNbHppChppciProblem_impls();

	hppDout (info, ":setPathOptimizer: nbChppciProblem_impls " << nbChppciProblem_impls << "problem id "
		 << problemId << ", pathOptimizerName " <<pathOptimizerName);
	// Test that rank is less than number of robots in vector.
	if (hppChppciProblem_implId < nbChppciProblem_impls) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr robot = planner_->robotIthChppciProblem_impl(hppChppciProblem_implId);
	  CkwsDistanceShPtr distance = CkwsDistance::create();

	  // Get distance function associated to robot if any
	  if (robot) {
	    if (robot->specificDistance()) {
	      distance = robot->specificDistance();
	    }
	  }

	  if (pathOptimizerName == "clear") {
	    CkwsPathOptimizerShPtr pathOptimizer = CkwsClearOptimizer::create();
	    pathOptimizer->distance(distance);
	    pathOptimizer->shortcutMethod(CkwsShortcutDirect::create());

	    status = planner_->pathOptimizerIthChppciProblem_impl(hppChppciProblem_implId, pathOptimizer);
	    hppDout (info, ":setPathOptimizer: clear path optimizer set.");
	    return (Short)status;
	  }
	  else if (pathOptimizerName == "adaptiveShortcut") {
	    CkwsLoopOptimizerShPtr pathOptimizer = CkwsAdaptiveShortcutOptimizer::create();
	    pathOptimizer->distance(distance);
	    pathOptimizer->shortcutMethod(CkwsShortcutDirect::create());
	    pathOptimizer->maxNbLoop(inMaxNumberLoop);

	    status = planner_->pathOptimizerIthChppciProblem_impl(hppChppciProblem_implId, pathOptimizer);
	    hppDout (info, ":setPathOptimizer: adaptive shortcut path optimizer set");
	    return (Short)status;
	  }
	  else if (pathOptimizerName == "random") {
	    CkwsLoopOptimizerShPtr pathOptimizer = CkwsRandomOptimizer::create();
	    pathOptimizer->distance(distance);
	    pathOptimizer->shortcutMethod(CkwsShortcutDirect::create());
	    pathOptimizer->maxNbLoop(inMaxNumberLoop);

	    status = planner_->pathOptimizerIthChppciProblem_impl(hppChppciProblem_implId, pathOptimizer);
	    hppDout (info, ":setPathOptimizer: random path optimizer set");
	    return (Short)status;
	  }
	  else if (pathOptimizerName == "none") {
	    CkwsPathOptimizerShPtr pathOptimizer;
	    status = planner_->pathOptimizerIthChppciProblem_impl(hppChppciProblem_implId, pathOptimizer);
	    hppDout (info, ":setPathOptimizer: no path optimizer set");
	    return (Short)status;
	  }
	  else {
	    hppDout (info, ":setPathOptimizer: unknown path optimizer");
	    return -1;
	  }
	}
	else {
	  hppDout (error, ":setPathOptimizer: wrong problem Id");
	  return -1;
	}

	return 0;
      }

      Short ChppciProblem_impl::setConfigExtractor(UShort problemId, Double inMinRadius,
					Double inMaxRadius, Double inScaleFactor)
      {
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int nbChppciProblem_impls = planner_->getNbHppChppciProblem_impls();

	// Test that rank is less than number of robots in vector.
	if (hppChppciProblem_implId < nbChppciProblem_impls) {
	  /*
	    If inMinRadius = 0 remove configuration extractor from problem.
	  */
	  CkwsConfigExtractorShPtr configExtractor;
	  if (inMinRadius == 0) {
	    if (planner_->configExtractorIthChppciProblem_impl(hppChppciProblem_implId, configExtractor) == KD_ERROR) {
	      hppDout (error, ":setConfigExtractor: failed to delete configuration extractor.");
	      return -1;
	    }
	  } else {
	    configExtractor = CkwsConfigExtractor::create(inMinRadius, inMaxRadius, inScaleFactor);
	    if (!configExtractor) {
	      hppDout (error, ":setConfigExtractor: failed at creating a configuration extractor.");
	      return -1;
	    }
	    if (planner_->configExtractorIthChppciProblem_impl(hppChppciProblem_implId, configExtractor) != KD_OK) {
	      hppDout (error, ":setConfigExtractor: failed at setting configuration extractor.");
	      return -1;
	    }
	  }
	}
	else {
	  hppDout (error, ":setConfigExtractor: wrong problem Id");
	  return -1;
	}

	return 0;
      }

      Short ChppciProblem_impl::setDistanceFunction(UShort problemId, const char* inDistanceName, Boolean oriented)
      {
	std::string distanceName(inDistanceName);
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int nbChppciProblem_impls = planner_->getNbHppChppciProblem_impls();

	// Test that rank is less than number of robots in vector.
	if (hppChppciProblem_implId < nbChppciProblem_impls) {

	  /* Check that name corresponds to a distance function factory */
	  if (!server_->distanceFactoryAlreadySet(distanceName)) {
	    hppDout (error, " unknown distance function.");
	    return -1;
	  }

	  // Create distance function
	  CkwsDistanceShPtr distance =
	    server_->createDistanceFunction(distanceName, oriented);

	  hppDout (info, ":setDistanceFunction: set roadmap builder distance function to " << distanceName);
	  //
	  // Set distance function in
	  //   - device
	  //   - roadmap builder if any
	  //   - path optimizer if any

	  // Device
	  CkppDeviceComponentShPtr robot = planner_->robotIthChppciProblem_impl(hppChppciProblem_implId);
	  if (!robot) {
	    hppDout (error, ":setDistanceFunction: no robot in problem " << hppChppciProblem_implId);
	    return -1;
	  }
	  robot->distance(distance);

	  // Roadmap builder
	  CkwsRoadmapBuilderShPtr roadmapBuilder = planner_->roadmapBuilderIthChppciProblem_impl(hppChppciProblem_implId);
	  if (roadmapBuilder) {
	    roadmapBuilder->distance(distance);
	  }

	  // path optimizer
	  CkwsPathOptimizerShPtr pathOptimizer =
	    planner_->pathOptimizerIthChppciProblem_impl(hppChppciProblem_implId);
	  if (pathOptimizer) {
	    pathOptimizer->distance(distance);
	  }
	}
	else {
	  hppDout (error, ":setDistanceFunction: wrong problem Id");
	  return -1;
	}

	return 0;
      }

      Short ChppciProblem_impl::setDiffusionNodePicker(UShort problemId,
					    const char* inDiffusionNodePickerName)
      {
	std::string diffusionNodePickerName(inDiffusionNodePickerName);
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int nbChppciProblem_impls = planner_->getNbHppChppciProblem_impls();

	// Test that rank is less than number of robots in vector.
	if (hppChppciProblem_implId < nbChppciProblem_impls) {
	  // Get roadmap builder in hppPlanner object.
	  CkwsDiffusingRdmBuilderShPtr roadmapBuilder =
	    KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder,
				 planner_->roadmapBuilderIthChppciProblem_impl(hppChppciProblem_implId));
	  // Check that diffusion roadmap builder is set
	  if (!roadmapBuilder) {
	    hppDout (error, ":setDiffusionNodePicker: roadmap builder is not set or not of type diffusion");
	    return -1;
	  }
	  /* Check that name corresponds to a diffusion node picker factory */
	  if (!server_->diffusionNodePickerFactoryAlreadySet(diffusionNodePickerName)) {
	    hppDout (error, ":setDiffusionNodePicker: unknown diffusion node picker.");
	    return -1;
	  }

	  // Create diffusion node picker
	  CkwsDiffusionNodePickerShPtr diffusionNodePicker =
	    server_->createDiffusionNodePicker(diffusionNodePickerName);

	  hppDout (info, ":setDiffusionNodePicker: set roadmap builder diffusion node picker to " << diffusionNodePickerName);
	  roadmapBuilder->diffusionNodePicker(diffusionNodePicker);
	}
	else {
	  hppDout (error, ":setDiffusionNodePicker: wrong problem Id");
	  return -1;
	}

	return 0;
      }

      Short ChppciProblem_impl::setDiffusionShooter(UShort problemId,
					 const char* inDiffusionShooterName,
					 Double inStandardDeviation)
      {
	std::string diffusionShooterName(inDiffusionShooterName);
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int nbChppciProblem_impls = planner_->getNbHppChppciProblem_impls();

	// Test that rank is less than number of robots in vector.
	if (hppChppciProblem_implId < nbChppciProblem_impls) {
	  // Get roadmap builder in hppPlanner object.
	  CkwsDiffusingRdmBuilderShPtr roadmapBuilder =
	    KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder,
				 planner_->roadmapBuilderIthChppciProblem_impl(hppChppciProblem_implId));
	  // Check that diffusion roadmap builder is set
	  if (!roadmapBuilder) {
	    hppDout (error, ":setDiffusionShooter: roadmap builder is not set or not of type diffusion");
	    return -1;
	  }
	  /* Check that name corresponds to a diffusion shooter factory */
	  if (!server_->diffusionShooterFactoryAlreadySet(diffusionShooterName)) {
	    hppDout (error, ":setDiffusionShooter: unknown diffusion shooter: " << diffusionShooterName);
	    return -1;
	  }

	  // Create diffusion shooter
	  CkwsDiffusionShooterShPtr diffusionShooter =
	    server_->createDiffusionShooter(diffusionShooterName,
					    inStandardDeviation);

	  hppDout (info, ":setDiffusionShooter: set roadmap builder diffusion shooter to " << diffusionShooterName);
	  roadmapBuilder->diffusionShooter(diffusionShooter);
	}
	else {
	  hppDout (error, ":setDiffusionShooter: wrong problem Id");
	  return -1;
	}

	return 0;
      }

      Short ChppciProblem_impl::setInitialConfig(UShort problemId, const hpp::dofSeq& dofArray)
      {
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int configDim = (unsigned int)dofArray.length();
	std::vector<double> dofVector;
	unsigned int nbChppciProblem_impls = planner_->getNbHppChppciProblem_impls();

	// Test that rank is less than nulber of robots in vector.
	if (hppChppciProblem_implId < nbChppciProblem_impls) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr robot = planner_->robotIthChppciProblem_impl(hppChppciProblem_implId);

	  // Compare size of input array with number of degrees of freedom of robot.
	  if (configDim != robot->countDofs()) {
	    hppDout (error, ":setInitialConfig: robot nb dof is different from config size");
	    return -1;
	  }
	  // Fill dof vector with dof array.
	  for (unsigned int iDof=0; iDof<configDim; iDof++) {
	    dofVector.push_back(dofArray[iDof]);
	  }
	  // Create a config for robot initialized with dof vector.
	  CkwsConfigShPtr config = CkwsConfig::create(robot, dofVector);
	  if (!config) {
	    hppDout (error, ":setInitialConfig: cannot create config. Check that robot nb dof is equal to config size");
	    return -1;
	  }

	  return (short)planner_->initConfIthChppciProblem_impl(hppChppciProblem_implId, config);
	}
	else {
	  hppDout (error, ":setInitialConfig: wrong problem Id");
	  return -1;
	}
	return 0;
      }

      Short ChppciProblem_impl::setGoalConfig(UShort problemId, const hpp::dofSeq& dofArray)
      {
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int configDim = (unsigned int)dofArray.length();
	std::vector<double> dofVector;
	unsigned int nbChppciProblem_impls = planner_->getNbHppChppciProblem_impls();

	// Test that rank is less than nulber of robots in vector.
	if (hppChppciProblem_implId < nbChppciProblem_impls) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr robot = planner_->robotIthChppciProblem_impl(hppChppciProblem_implId);

	  // Compare size of input array with number of degrees of freedom of robot.
	  if (configDim != robot->countDofs()) {
	    hppDout (error, ":setInitialConfig: robot nb dof is different from config size");
	    return -1;
	  }

	  // Fill dof vector with dof array.
	  for (unsigned int iDof=0; iDof<configDim; iDof++) {
	    dofVector.push_back(dofArray[iDof]);
	  }
	  // Create a config for robot initialized with dof vector.
	  CkwsConfigShPtr config = CkwsConfig::create(robot, dofVector);
	  if (!config) {
	    hppDout (error, ":setGoalConfig: cannot create config. Check that robot nb dof is equal to config size");
	    return -1;
	  }
	  return (short)planner_->goalConfIthChppciProblem_impl(hppChppciProblem_implId, config);
	}
	else {
	  hppDout (error, ":setGoalConfig: wrong problem Id");
	  return -1;
	}
	return 0;
      }

      hpp::dofSeq* ChppciProblem_impl::getInitialConfig(UShort problemId)
	throw(SystemException)
      {
	hpp::dofSeq *dofArray;
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int nbChppciProblem_impls = planner_->getNbHppChppciProblem_impls();

	if (hppChppciProblem_implId < nbChppciProblem_impls) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr robot = planner_->robotIthChppciProblem_impl(hppChppciProblem_implId);

	  std::vector<double> dofVector;
	  CkwsConfigShPtr config = planner_->initConfIthChppciProblem_impl(hppChppciProblem_implId);

	  if (config) {
	    // by Yoshida 06/08/25
	    unsigned int deviceDim = config->size();

	    dofArray = new hpp::dofSeq();
	    dofArray->length(deviceDim);

	    for(unsigned int i=0; i<deviceDim; i++){
	      (*dofArray)[i] = config->dofValue(i);
	    }
	    return dofArray;
	  }
	  else {
	    hppDout (error, ":getInitialConfig: no initial configuration defined");
	    dofArray = new hpp::dofSeq(1);
	    return dofArray;
	  }
	}

	else {
	  hppDout (error, ":getInitialConfig: wrong problem Id");
	  dofArray = new hpp::dofSeq(1);
	  return dofArray;
	}

	return new hpp::dofSeq(1);
      }


      hpp::dofSeq* ChppciProblem_impl::getGoalConfig(UShort problemId)
	throw(SystemException)
      {
	hpp::dofSeq *dofArray;
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int nbChppciProblem_impls = planner_->getNbHppChppciProblem_impls();

	if (hppChppciProblem_implId < nbChppciProblem_impls) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr robot = planner_->robotIthChppciProblem_impl(hppChppciProblem_implId);

	  std::vector<double> dofVector;
	  CkwsConfigShPtr config = planner_->goalConfIthChppciProblem_impl(hppChppciProblem_implId);

	  if (config)
	    {
	      unsigned int deviceDim = config->size ();

	      dofArray = new hpp::dofSeq ();
	      dofArray->length (deviceDim);

	      for (unsigned int i=0; i<deviceDim; i++)
		(*dofArray)[i] = config->dofValue(i);
	      return dofArray;
	  }
	  else {
	    hppDout (error, ":getInitialConfig: no initial configuration defined");
	    dofArray = new hpp::dofSeq(1);
	    return dofArray;
	  }
	}

	else {
	  hppDout (error, ":getInitialConfig: wrong problem Id");
	  dofArray = new hpp::dofSeq(1);
	  return dofArray;
	}

	return new hpp::dofSeq(1);
      }


      Short ChppciProblem_impl::initializeChppciProblem_impl()
      {
	ktStatus status  = planner_->initializeChppciProblem_impl();
	return (Short)status;
      }


      Short ChppciProblem_impl::solveOneChppciProblem_impl(UShort problemId, Short& inLastPathId, Double& pathLength)
      {

	ktStatus status = KD_ERROR;
	inLastPathId = 0 ;
	pathLength = 0;

	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int nbChppciProblem_impls = planner_->getNbHppChppciProblem_impls();

	// Test that rank is less than number of robots in vector.
	if (hppChppciProblem_implId < nbChppciProblem_impls) {
	  status = planner_->solveOneChppciProblem_impl(hppChppciProblem_implId);
	}


	inLastPathId = planner_->getNbPaths(hppChppciProblem_implId) - 1;
	if (inLastPathId > -1) {
	  pathLength = planner_->getPath(hppChppciProblem_implId, inLastPathId)->length();
	}
	else {
	  hppDout (error, ":solveOneChppciProblem_impl: no path in hppChppciProblem_impl " << hppChppciProblem_implId);
	}

	return (Short)status;
      }


      Short ChppciProblem_impl::solve()
      {
	ktStatus status  = planner_->solve();
	return (Short)status;
      }

      Short ChppciProblem_impl::interruptPathPlanning()
      {
	planner_->interruptPathPlanning();
	return 0;
      }

      Short ChppciProblem_impl::optimizePath(UShort problemId, UShort inPathId)
      {
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int pathId = (unsigned int)inPathId;

	if (hppChppciProblem_implId > planner_->getNbHppChppciProblem_impls() - 1) {
	  hppDout (error, ":optimizePath: wrong problem id");
	  return -1;
	}

	if (pathId > planner_->getNbPaths(hppChppciProblem_implId) - 1) {
	  hppDout (error, ":optimizePath: wrong path id");
	  return -1;
	}

	ktStatus status  = planner_->optimizePath((unsigned int) problemId, (unsigned int) inPathId);
	return (Short)status;
      }

      Double ChppciProblem_impl::pathLength(UShort problemId, UShort inPathId)
      {
	double length = -1.0;

	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int pathId = (unsigned int)inPathId;

	if (hppChppciProblem_implId > planner_->getNbHppChppciProblem_impls() - 1) {
	  hppDout (error, ":pathLength: wrong problem id");
	  return -1.0;
	}

	if (pathId > planner_->getNbPaths(hppChppciProblem_implId) - 1) {
	  hppDout (error, ":pathLength: wrong path id");
	  return -1.0;
	}

	length = planner_->getPath(hppChppciProblem_implId, pathId)->length();
	return length;
      }

      hpp::dofSeq* ChppciProblem_impl::configAtDistance(UShort problemId, UShort inPathId, Double atDistance)
      {
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	unsigned int pathId = (unsigned int)inPathId;

	if (hppChppciProblem_implId > planner_->getNbHppChppciProblem_impls() - 1) {
	  hppDout (error, ":configAtDistance: wrong problem id");
	  return new hpp::dofSeq(0, 0, NULL, true);
	}

	if (pathId > planner_->getNbPaths(hppChppciProblem_implId) - 1) {
	  hppDout (error, ":configAtDistance: wrong path id");
	  return new hpp::dofSeq(0, 0, NULL, true);
	}

	double length = planner_->getPath(hppChppciProblem_implId, pathId)->length();

	//get the nb of dof of the robot in the problem
	unsigned int nbDofRobot = planner_->robotIthChppciProblem_impl(problemId)->countDofs();

	//init the seqdof
	// Allocate result now that the size is known.
	ULong size = (ULong)nbDofRobot;
	double* DofList = hpp::dofSeq::allocbuf(size);
	hpp::dofSeq* dofSeq = new hpp::dofSeq(size, size, DofList, true);

	//get the config of the robot on the path
	if (atDistance > length) {
	  hppDout (error, ":configAtParam: param out of range (longer than path Length) " << "Param : "<< atDistance << " length : " << length);
	}
	else {
	  CkwsConfigShPtr inConfig ;
	  inConfig = planner_->getPath(hppChppciProblem_implId, pathId)->configAtDistance(atDistance) ;
	  //convert the config in dofseq
	  for ( unsigned int i = 0 ; i < inConfig->size() ; i++){
	    DofList[i] =  inConfig->dofValue(i) ;
	  }
	}

	return dofSeq;
      }

      /// \brief set tolerance to the objects in the planner
      Short ChppciProblem_impl::setObstacleTolerance(UShort problemId, Double tolerance)
	throw(SystemException)
      {
	// get the planner
	unsigned int hppChppciProblem_implId = (unsigned int)problemId;
	// get object hppPlanner of Corba server.
	if(!planner_)
	  {
	    hppDout (error, ":setObstacleTolerance: problem " << hppChppciProblem_implId << " not found");
	    return -1;
	  }

	std::vector<CkcdObjectShPtr> oList = planner_->obstacleList ();

	if(oList.size() == 0)
	  hppDout (warning, "there are no obstacle in problem " << hppChppciProblem_implId);

	for (unsigned int i = 0; i < oList.size(); ++i)
	  {
	    oList[i]->tolerance(tolerance);
	    hppDout (error, ":setObstacleTolerance: tolerance " << tolerance << " set to obstacle " << i);
	  }
	return 0;
      }
      Short ChppciProblem_impl::parseFile
	(const char* inFilename) throw (SystemException)
      {
	if(!planner_)
	  {
	    hppDout (error, ":parseFile: no planner.");
	    return -1;
	  }
	if (planner_->parseFile(inFilename) != KD_OK) return -1;
	return 0;
      }
    }
  }
}
