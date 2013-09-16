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
      Problem::Problem (corbaServer::Server* server)
	: server_ (server),
	  planner_ (server->planner ())
      {}

      Short Problem::setRoadmapbuilder(UShort problemId, const char* inRoadmapBuilderName,
				       Boolean inDisplay)
      {
	std::string roadmapBuilderName(inRoadmapBuilderName);
	unsigned int hppProblemId = (unsigned int)problemId;
	ktStatus status;

	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr robot = planner_->robotIthProblem(hppProblemId);

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
	      CkwsPlusPCARdmBuilder<CkwsDiffusingRdmBuilder>::create(roadmap);
	  } else {
	    hppDout (error, ":setRoadmapbuilder: unknown roadmap builder");
	    return -1;
	  }
	  status = planner_->roadmapBuilderIthProblem(hppProblemId, roadmapBuilder, inDisplay);
	  return status;

	}
	else {
	  hppDout (error, ":setRoadmapbuilder: wrong problem Id");
	  return -1;
	}

	return 0;
      }

      Short Problem::setDiffusingNode(UShort problemId,
				      const char* inDiffusingNode)
      {
	std::string diffusingNode(inDiffusingNode);
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get roadmap builder in hppPlanner object.
	  CkwsDiffusingRdmBuilderShPtr roadmapBuilder =
	    KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder,
				 planner_->roadmapBuilderIthProblem(hppProblemId));
	  // Check that diffusion roadmap builder is set
	  if (!roadmapBuilder) {
	    hppDout (error, ":setDiffusingNode: roadmap builder is not set or not of type diffusion");
	    return -1;
	  }
	  if (diffusingNode == "start") {
	    roadmapBuilder->diffuseFromProblemStart(true);
	    roadmapBuilder->diffuseFromProblemGoal(false);
	    hppDout (info, ":setDiffusingNode: diffusing from start");
	  }
	  else if (diffusingNode == "goal") {
	    roadmapBuilder->diffuseFromProblemStart(false);
	    roadmapBuilder->diffuseFromProblemGoal(true);
	    hppDout (info, ":setDiffusingNode: diffusing from goal");
	  }
	  else if (diffusingNode == "start and goal") {
	    roadmapBuilder->diffuseFromProblemStart(true);
	    roadmapBuilder->diffuseFromProblemGoal(true);
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

      Short Problem::setPathOptimizer(UShort problemId,
				      const char* inPathOptimizerName,
				      UShort inMaxNumberLoop)
      {
	std::string pathOptimizerName(inPathOptimizerName);

	ktStatus status;
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int nbProblems = planner_->getNbHppProblems();

	hppDout (info, ":setPathOptimizer: nbProblems " << nbProblems << "problem id "
		 << problemId << ", pathOptimizerName " <<pathOptimizerName);
	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr robot = planner_->robotIthProblem(hppProblemId);

	  if (pathOptimizerName == "clear") {
	    CkwsPathOptimizerShPtr pathOptimizer = CkwsClearOptimizer::create();
	    pathOptimizer->pathEvaluator (CkwsPathEvaluatorLength::create ());
	    pathOptimizer->shortcutMethod(CkwsShortcutDirect::create());

	    status = planner_->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
	    hppDout (info, ":setPathOptimizer: clear path optimizer set.");
	    return (Short)status;
	  }
	  else if (pathOptimizerName == "adaptiveShortcut") {
	    CkwsLoopOptimizerShPtr pathOptimizer = CkwsAdaptiveShortcutOptimizer::create();
	    pathOptimizer->pathEvaluator (CkwsPathEvaluatorLength::create ());
	    pathOptimizer->shortcutMethod(CkwsShortcutDirect::create());
	    pathOptimizer->maxNbLoop(inMaxNumberLoop);

	    status = planner_->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
	    hppDout (info, ":setPathOptimizer: adaptive shortcut path optimizer set");
	    return (Short)status;
	  }
	  else if (pathOptimizerName == "random") {
	    CkwsLoopOptimizerShPtr pathOptimizer = CkwsRandomOptimizer::create();
	    pathOptimizer->pathEvaluator (CkwsPathEvaluatorLength::create ());
	    pathOptimizer->shortcutMethod(CkwsShortcutDirect::create());
	    pathOptimizer->maxNbLoop(inMaxNumberLoop);

	    status = planner_->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
	    hppDout (info, ":setPathOptimizer: random path optimizer set");
	    return (Short)status;
	  }
	  else if (pathOptimizerName == "none") {
	    CkwsPathPlannerShPtr pathOptimizer;
	    status = planner_->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
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

      Short Problem::setConfigExtractor(UShort problemId, Double inMinRadius,
					Double inMaxRadius, Double inScaleFactor)
      {
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  /*
	    If inMinRadius = 0 remove configuration extractor from problem.
	  */
	  CkwsConfigExtractorShPtr configExtractor;
	  if (inMinRadius == 0) {
	    if (planner_->configExtractorIthProblem(hppProblemId, configExtractor) == KD_ERROR) {
	      hppDout (error, ":setConfigExtractor: failed to delete configuration extractor.");
	      return -1;
	    }
	  } else {
	    configExtractor = CkwsConfigExtractor::create(inMinRadius, inMaxRadius, inScaleFactor);
	    if (!configExtractor) {
	      hppDout (error, ":setConfigExtractor: failed at creating a configuration extractor.");
	      return -1;
	    }
	    if (planner_->configExtractorIthProblem(hppProblemId, configExtractor) != KD_OK) {
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

      Short Problem::setDistanceFunction(UShort problemId, const char* inDistanceName, Boolean oriented)
      {
	std::string distanceName(inDistanceName);
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {

	  /* Check that name corresponds to a distance function factory */
	  if (!server_->distanceFactoryAlreadySet(distanceName)) {
	    hppDout (error, " unknown distance function.");
	    return -1;
	  }

	  // Create distance function
	  CkwsMetricShPtr distance =
	    server_->createDistanceFunction(distanceName, oriented);

	  hppDout (info, ":setDistanceFunction: set roadmap builder distance function to " << distanceName);
	  //
	  // Set distance function in
	  //   - device
	  //   - roadmap builder if any
	  //   - path optimizer if any

	  // Device
	  CkppDeviceComponentShPtr robot = planner_->robotIthProblem(hppProblemId);
	  if (!robot) {
	    hppDout (error, ":setDistanceFunction: no robot in problem " << hppProblemId);
	    return -1;
	  }
	  robot->configSpace ()->metric (distance);

	  // Roadmap builder
	  CkwsRoadmapBuilderShPtr roadmapBuilder = planner_->roadmapBuilderIthProblem(hppProblemId);
	  if (roadmapBuilder) {
	    roadmapBuilder->roadmap ()->configSpace ()-> metric (distance);
	  }

	  // path optimizer
	  CkwsPathPlannerShPtr pathOptimizer =
	    planner_->pathOptimizerIthProblem(hppProblemId);
	  if (pathOptimizer) {
	    pathOptimizer->configSpace ()->metric (distance);
	  }
	}
	else {
	  hppDout (error, ":setDistanceFunction: wrong problem Id");
	  return -1;
	}

	return 0;
      }

      Short Problem::setDiffusionNodePicker(UShort problemId,
					    const char* inDiffusionNodePickerName)
      {
	std::string diffusionNodePickerName(inDiffusionNodePickerName);
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get roadmap builder in hppPlanner object.
	  CkwsDiffusingRdmBuilderShPtr roadmapBuilder =
	    KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder,
				 planner_->roadmapBuilderIthProblem(hppProblemId));
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

      Short Problem::setDiffusionShooter(UShort problemId,
					 const char* inDiffusionShooterName)
      {
	std::string diffusionShooterName(inDiffusionShooterName);
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get roadmap builder in hppPlanner object.
	  CkwsDiffusingRdmBuilderShPtr roadmapBuilder =
	    KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder,
				 planner_->roadmapBuilderIthProblem(hppProblemId));
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
	    server_->createDiffusionShooter(diffusionShooterName);

	  hppDout (info, ":setDiffusionShooter: set roadmap builder diffusion shooter to " << diffusionShooterName);
	  roadmapBuilder->diffusionShooter(diffusionShooter);
	}
	else {
	  hppDout (error, ":setDiffusionShooter: wrong problem Id");
	  return -1;
	}

	return 0;
      }

      Short Problem::setInitialConfig(UShort problemId, const hpp::dofSeq& dofArray)
      {
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int configDim = (unsigned int)dofArray.length();
	std::vector<double> dofVector;
	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than nulber of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr robot = planner_->robotIthProblem(hppProblemId);

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

	  planner_->initConfIthProblem(hppProblemId, config);
	  return 0;
	}
	else {
	  hppDout (error, ":setInitialConfig: wrong problem Id");
	  return -1;
	}
	return 0;
      }

      hpp::dofSeq* Problem::getInitialConfig(UShort problemId)
	throw(SystemException)
      {
	hpp::dofSeq *dofArray;
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int nbProblems = planner_->getNbHppProblems();

	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr robot = planner_->robotIthProblem(hppProblemId);

	  std::vector<double> dofVector;
	  CkwsConfigShPtr config = planner_->initConfIthProblem(hppProblemId);

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


      Short Problem::addGoalConfig(UShort problemId,
				   const hpp::dofSeq& dofArray)
      {
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int configDim = (unsigned int)dofArray.length();
	std::vector<double> dofVector;
	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than nulber of robots in vector.
	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr robot =
	    planner_->robotIthProblem(hppProblemId);

	  // Compare size of input array with number of degrees of freedom of
	  // robot.
	  if (configDim != robot->countDofs()) {
	    hppDout (error, ":setInitialConfig: robot nb dof is different from"
		     " config size");
	    return -1;
	  }

	  // Fill dof vector with dof array.
	  for (unsigned int iDof=0; iDof<configDim; iDof++) {
	    dofVector.push_back(dofArray[iDof]);
	  }
	  // Create a config for robot initialized with dof vector.
	  CkwsConfigShPtr config = CkwsConfig::create(robot, dofVector);
	  if (!config) {
	    hppDout (error, ":setGoalConfig: cannot create config. Check that"
		     " robot nb dof is equal to config size");
	    return -1;
	  }
	  planner_->addGoalConfIthProblem (hppProblemId, config);
	  return 0;
	}
	else {
	  hppDout (error, ":setGoalConfig: wrong problem Id");
	  return -1;
	}
	return 0;
      }

      hpp::dofSeqSeq* Problem::getGoalConfig(UShort problemId)
	throw(SystemException)
      {
	typedef hpp::core::Problem::goalConfigConstIterator_t
	  goalConfigConstIterator_t;
	hpp::dofSeqSeq *configSequence;
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int nbProblems = planner_->getNbHppProblems();

	if (hppProblemId < nbProblems) {
	  // Get robot in hppPlanner object.
	  CkppDeviceComponentShPtr robot =
	    planner_->robotIthProblem(hppProblemId);

	  std::vector<double> dofVector;
	  const std::vector< CkwsConfigShPtr >& goalConfs =
	    planner_->goalConfIthProblem(hppProblemId);
	  configSequence = new hpp::dofSeqSeq ();
	  configSequence->length (goalConfs.size ());
	  for (std::size_t i=0; i<goalConfs.size () ;i++) {
	    CkwsConfigShPtr config = goalConfs [i];
	    unsigned int deviceDim = config->size ();

	    hpp::dofSeq dofArray;
	    dofArray.length (deviceDim);

	    for (unsigned int j=0; j<deviceDim; j++)
	      dofArray[j] = config->dofValue(j);
	    (*configSequence) [i] = dofArray;
	  }
	  return configSequence;
	}
	else {
	  hppDout (error, ":getInitialConfig: wrong problem Id");
	  configSequence = new hpp::dofSeqSeq(1);
	  return configSequence;
	}
	return new hpp::dofSeqSeq (1);
      }

      Short Problem::resetGoalConfig (UShort problemId)  throw (SystemException)
      {
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int nbProblems = planner_->getNbHppProblems();

	if (hppProblemId >= nbProblems) return -1;
	planner_->resetGoalConfIthProblem (hppProblemId);
	return 0;
      }

      Short Problem::initializeProblem()
      {
	ktStatus status  = planner_->initializeProblem();
	return (Short)status;
      }


      Short Problem::solveOneProblem(UShort problemId, Short& inLastPathId, Double& pathLength)
      {

	ktStatus status = KD_ERROR;
	inLastPathId = 0 ;
	pathLength = 0;

	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int nbProblems = planner_->getNbHppProblems();

	// Test that rank is less than number of robots in vector.
	if (hppProblemId < nbProblems) {
	  status = planner_->solveOneProblem(hppProblemId);
	}


	inLastPathId = planner_->getNbPaths(hppProblemId) - 1;
	if (inLastPathId > -1) {
	  pathLength = planner_->getPath(hppProblemId, inLastPathId)->length();
	}
	else {
	  hppDout (error, ":solveOneProblem: no path in hppProblem " << hppProblemId);
	}

	return (Short)status;
      }


      Short Problem::solve()
      {
	ktStatus status  = planner_->solve();
	return (Short)status;
      }

      Short Problem::interruptPathPlanning()
      {
	planner_->interruptPathPlanning();
	return 0;
      }

      Short Problem::optimizePath(UShort problemId, UShort inPathId)
      {
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int pathId = (unsigned int)inPathId;

	if (hppProblemId > planner_->getNbHppProblems() - 1) {
	  hppDout (error, ":optimizePath: wrong problem id");
	  return -1;
	}

	if (pathId > planner_->getNbPaths(hppProblemId) - 1) {
	  hppDout (error, ":optimizePath: wrong path id");
	  return -1;
	}

	ktStatus status  = planner_->optimizePath((unsigned int) problemId, (unsigned int) inPathId);
	return (Short)status;
      }

      Double Problem::pathLength(UShort problemId, UShort inPathId)
      {
	double length = -1.0;

	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int pathId = (unsigned int)inPathId;

	if (hppProblemId > planner_->getNbHppProblems() - 1) {
	  hppDout (error, ":pathLength: wrong problem id");
	  return -1.0;
	}

	if (pathId > planner_->getNbPaths(hppProblemId) - 1) {
	  hppDout (error, ":pathLength: wrong path id");
	  return -1.0;
	}

	length = planner_->getPath(hppProblemId, pathId)->length();
	return length;
      }

      hpp::dofSeq* Problem::configAtDistance(UShort problemId, UShort inPathId, Double atDistance)
      {
	unsigned int hppProblemId = (unsigned int)problemId;
	unsigned int pathId = (unsigned int)inPathId;

	if (hppProblemId > planner_->getNbHppProblems() - 1) {
	  hppDout (error, ":configAtDistance: wrong problem id");
	  return new hpp::dofSeq(0, 0, NULL, true);
	}

	if (pathId > planner_->getNbPaths(hppProblemId) - 1) {
	  hppDout (error, ":configAtDistance: wrong path id");
	  return new hpp::dofSeq(0, 0, NULL, true);
	}

	double length = planner_->getPath(hppProblemId, pathId)->length();

	//get the nb of dof of the robot in the problem
	unsigned int nbDofRobot = planner_->robotIthProblem(problemId)->countDofs();

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
	  inConfig = planner_->getPath(hppProblemId, pathId)->configAtDistance(atDistance) ;
	  //convert the config in dofseq
	  for ( unsigned int i = 0 ; i < inConfig->size() ; i++){
	    DofList[i] =  inConfig->dofValue(i) ;
	  }
	}

	return dofSeq;
      }

      /// \brief set tolerance to the objects in the planner
      Short Problem::setObstacleTolerance(UShort problemId, Double tolerance)
	throw(SystemException)
      {
	// get the planner
#ifdef HPP_DEBUG
	unsigned int hppProblemId = (unsigned int)problemId;
#endif
	// get object hppPlanner of Corba server.
	if(!planner_)
	  {
	    hppDout (error, ":setObstacleTolerance: problem " << hppProblemId << " not found");
	    return -1;
	  }

	std::vector<CkcdObjectShPtr> oList = planner_->obstacleList ();

	if(oList.size() == 0)
	  hppDout (warning, "there are no obstacle in problem " << hppProblemId);

	for (unsigned int i = 0; i < oList.size(); ++i)
	  {
	    oList[i]->tolerance(tolerance);
	    hppDout (error, ":setObstacleTolerance: tolerance " << tolerance << " set to obstacle " << i);
	  }
	return 0;
      }
      Short Problem::parseFile
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

      Short Problem::loadPathFromFile
	(const char* inFilename) throw (SystemException)
      {
	if(!planner_)
	  {
	    hppDout (error, ":parseFile: no planner.");
	    return -1;
	  }
	if (planner_->loadPathFromFile(inFilename) != KD_OK) return -1;
	return 0;
      }

      Short Problem::countNodes (UShort problemId)
      {
	unsigned int hppProblemId = (unsigned int)problemId;

	if (hppProblemId > planner_->getNbHppProblems() - 1) {
	  hppDout (error, "wrong problem id");
	  return -1;
	}
	if (CkwsRoadmapBuilderShPtr rdmbuilder =
	    planner_->roadmapBuilderIthProblem (hppProblemId))
	  {
	    return rdmbuilder->roadmap ()->countNodes ();
	  }
	return -1;
      }

      hpp::dofSeq* Problem::node (UShort problemId, UShort nodeId)
      {
	unsigned int hppProblemId = (unsigned int)problemId;

	if (hppProblemId > planner_->getNbHppProblems() - 1) {
	  hppDout (error, "wrong problem id");
	  return new hpp::dofSeq(0, 0, NULL, true);
	}
	if (CkwsRoadmapBuilderShPtr rdmbuilder =
	    planner_->roadmapBuilderIthProblem (hppProblemId))
	  {
	    CkwsNodeShPtr node =
	      rdmbuilder->roadmap ()->node ((unsigned int)nodeId);
	    if (node) {
	      const CkwsConfig& config = node->config ();
	      //init the seqdof
	      // Allocate result now that the size is known.
	      ULong size = (ULong)config.size ();
	      double* DofList = hpp::dofSeq::allocbuf(size);
	      hpp::dofSeq* dofSeq = new hpp::dofSeq(size, size, DofList, true);

	      //convert the config in dofseq
	      for (unsigned int i = 0 ; i < config.size() ; i++) {
		DofList[i] =  config.dofValue(i) ;
	      }
	      return dofSeq;
	    }
	  }
	return new hpp::dofSeq(0, 0, NULL, true);
      }

      Short Problem::countEdges (UShort problemId)
      {
	unsigned int hppProblemId = (unsigned int)problemId;

	if (hppProblemId > planner_->getNbHppProblems() - 1) {
	  hppDout (error, "wrong problem id");
	  return -1;
	}
	if (CkwsRoadmapBuilderShPtr rdmbuilder =
	    planner_->roadmapBuilderIthProblem (hppProblemId))
	  {
	    return rdmbuilder->roadmap ()->countEdges ();
	  }
	return -1;
      }
      Short Problem::countConnectedComponents (UShort problemId)
      {
	unsigned int hppProblemId = (unsigned int)problemId;

	if (hppProblemId > planner_->getNbHppProblems() - 1) {
	  hppDout (error, "wrong problem id");
	  return -1;
	}
	if (CkwsRoadmapBuilderShPtr rdmbuilder =
	    planner_->roadmapBuilderIthProblem (hppProblemId))
	  {
	    return rdmbuilder->roadmap ()->countConnectedComponents ();
	  }
	return -1;
      }
      Short Problem::clearRoadmaps ()
      {
	planner_->clearRoadmaps ();
      }
    }
  }
}
