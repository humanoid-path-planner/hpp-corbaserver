/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#include <iostream>
#include "hppCorbaServer/hppciServer.h"
#include "hppCorbaServer/hppciProblem.h"
#include "kwsPlus/directPath/flicSteeringMethod.h"
#include "kwsPlus/directPath/reedsSheppSteeringMethod.h"
#include "kwsPlus/roadmap/hppVisRdmBuilder.h"
#include "kwsPlus/roadmap/kwsPlusPCARdmBuilder.h"
#include "KineoWorks2/kwsIPPRdmBuilder.h"

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==2
#define ODEBUG2(x) std::cout << "ChppciProblem:" << x << std::endl
#define ODEBUG1(x) std::cerr << "ChppciProblem:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "ChppciProblem:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

ChppciProblem_impl::ChppciProblem_impl(ChppciServer* inHppciServer) : 
  attHppciServer(inHppciServer), attHppPlanner(inHppciServer->getHppPlanner())
{
}


CORBA::Short ChppciProblem_impl::setSteeringMethod(CORBA::UShort inProblemId, 
						   const char* inSteeringMethod, CORBA::Boolean inOriented)
{
  std::string steeringMethodName(inSteeringMethod);

  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

    /* Check that name correspond to a steering method factory */
    if (!attHppciServer->steeringMethodFactoryAlreadySet(steeringMethodName)) {
      ODEBUG1(":setSteeringMethod: unknown steering method.");
      return -1;
    }

    // Create steering method
    CkwsSteeringMethodShPtr steeringMethod = 
      attHppciServer->createSteeringMethod(steeringMethodName, inOriented);

    hppRobot->steeringMethod(steeringMethod);
  }
  else {
    ODEBUG1(":setSteeringMethod: wrong problem Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setRoadmapbuilder(CORBA::UShort inProblemId, const char* inRoadmapBuilderName,
						   CORBA::Boolean inDisplay)
{
  std::string roadmapBuilderName(inRoadmapBuilderName);
  unsigned int hppProblemId = (unsigned int)inProblemId;
  ktStatus status; 

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

    // Create an empty roadmap for the robot.
    CkwsRoadmapShPtr roadmap = CkwsRoadmap::create(hppRobot->kwsDevice());
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
      ODEBUG1(":setRoadmapbuilder: unknown roadmap builder");
      return -1;
    }
    status = attHppPlanner->roadmapBuilderIthProblem(hppProblemId, roadmapBuilder, inDisplay);
    return status;      
    
  }
  else {
    ODEBUG1(":setRoadmapbuilder: wrong problem Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setDiffusingNode(CORBA::UShort inProblemId, 
						  const char* inDiffusingNode)
{
  std::string diffusingNode(inDiffusingNode);
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get roadmap builder in hppPlanner object.
    CkwsDiffusingRdmBuilderShPtr roadmapBuilder = 
      KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder, 
			   attHppPlanner->roadmapBuilderIthProblem(hppProblemId));
    // Check that diffusion roadmap builder is set
    if (!roadmapBuilder) {
      ODEBUG1(":setDiffusingNode: roadmap builder is not set or not of type diffusion");
      return -1;
    }
    if (diffusingNode == "start") {
      roadmapBuilder->diffuseFromProblemStart(true);
      roadmapBuilder->diffuseFromProblemGoal(false);
      ODEBUG2(":setDiffusingNode: diffusing from start");
    }
    else if (diffusingNode == "goal") {
      roadmapBuilder->diffuseFromProblemStart(false);
      roadmapBuilder->diffuseFromProblemGoal(true);
      ODEBUG2(":setDiffusingNode: diffusing from goal");
    } 
    else if (diffusingNode == "start and goal") {
      roadmapBuilder->diffuseFromProblemStart(true);
      roadmapBuilder->diffuseFromProblemGoal(true);
      ODEBUG2(":setDiffusingNode: diffusing from start and goal");
    }
    else  {
      ODEBUG2(":setDiffusingNode: unknown command " << diffusingNode);
    }
  }
  else {
    ODEBUG1(":setDiffusingNode: wrong problem Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setPathOptimizer(CORBA::UShort inProblemId, 
						  const char* inPathOptimizerName,
						  CORBA::UShort inMaxNumberLoop)
{
  std::string pathOptimizerName(inPathOptimizerName);

  ktStatus status;
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  ODEBUG2(":setPathOptimizer: nbProblems " << nbProblems << "problem id " 
	  << inProblemId << ", pathOptimizerName " <<pathOptimizerName);
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);
    CkwsDistanceShPtr distance = CkwsDistance::create();

    // Get distance function associated to robot if any
    if (hppRobot) {
      if (hppRobot->specificDistance()) {
	distance = hppRobot->specificDistance();
      }
    }

    if (pathOptimizerName == "clear") {
      CkwsPathOptimizerShPtr pathOptimizer = CkwsClearOptimizer::create();
      pathOptimizer->distance(distance);
      pathOptimizer->shortcutMethod(CkwsShortcutDirect::create());

      status = attHppPlanner->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
      ODEBUG2(":setPathOptimizer: clear path optimizer set.");
      return (CORBA::Short)status;
    } 
    else if (pathOptimizerName == "adaptiveShortcut") {
      CkwsLoopOptimizerShPtr pathOptimizer = CkwsAdaptiveShortcutOptimizer::create();
      pathOptimizer->distance(distance);
      pathOptimizer->shortcutMethod(CkwsShortcutDirect::create());
      pathOptimizer->maxNbLoop(inMaxNumberLoop);

      status = attHppPlanner->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
      ODEBUG2(":setPathOptimizer: adaptive shortcut path optimizer set");
      return (CORBA::Short)status;
    } 
    else if (pathOptimizerName == "random") {
      CkwsLoopOptimizerShPtr pathOptimizer = CkwsRandomOptimizer::create();
      pathOptimizer->distance(distance);
      pathOptimizer->shortcutMethod(CkwsShortcutDirect::create());
      pathOptimizer->maxNbLoop(inMaxNumberLoop);

      status = attHppPlanner->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
      ODEBUG2(":setPathOptimizer: random path optimizer set");
      return (CORBA::Short)status;
    } 
    else if (pathOptimizerName == "none") {
      CkwsPathOptimizerShPtr pathOptimizer;
      status = attHppPlanner->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
      ODEBUG2(":setPathOptimizer: no path optimizer set");
      return (CORBA::Short)status;
    } 
    else {
      ODEBUG2(":setPathOptimizer: unknown path optimizer");
      return -1;
    }
  }
  else {
    ODEBUG1(":setPathOptimizer: wrong problem Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setConfigExtractor(CORBA::UShort inProblemId, CORBA::Double inMinRadius,
						    CORBA::Double inMaxRadius, CORBA::Double inScaleFactor)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    /*
      If inMinRadius = 0 remove configuration extractor from problem.
    */
    CkwsConfigExtractorShPtr configExtractor;
    if (inMinRadius == 0) {
      if (attHppPlanner->configExtractorIthProblem(hppProblemId, configExtractor) == KD_ERROR) {
	ODEBUG1(":setConfigExtractor: failed to delete configuration extractor.");
	return -1;
      } 
    } else {
      configExtractor = CkwsConfigExtractor::create(inMinRadius, inMaxRadius, inScaleFactor);
      if (!configExtractor) {
	ODEBUG1(":setConfigExtractor: failed at creating a configuration extractor.");
	return -1;
      }
      if (attHppPlanner->configExtractorIthProblem(hppProblemId, configExtractor) != KD_OK) {
	ODEBUG1(":setConfigExtractor: failed at setting configuration extractor.");
	return -1;
      }
    }
  }  
  else {
    ODEBUG1(":setConfigExtractor: wrong problem Id");
    return -1;
  }

  return 0;
}


CORBA::Short ChppciProblem_impl::setDistanceFunction(CORBA::UShort inProblemId, const char* inDistanceName, CORBA::Boolean inOriented)
{
  std::string distanceName(inDistanceName);
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {

    /* Check that name corresponds to a distance function factory */
    if (!attHppciServer->distanceFactoryAlreadySet(distanceName)) {
      ODEBUG1(" unknown distance function.");
      return -1;
    }

    // Create distance function
    CkwsDistanceShPtr distance = 
      attHppciServer->createDistanceFunction(distanceName, inOriented);

    ODEBUG2(":setDistanceFunction: set roadmap builder distance function to " << distanceName);
    //
    // Set distance function in 
    //   - device
    //   - roadmap builder if any
    //   - path optimizer if any

    // Device
    CkppDeviceComponentShPtr robot = attHppPlanner->robotIthProblem(hppProblemId);
    if (!robot) {
      ODEBUG1(":setDistanceFunction: no robot in problem " << hppProblemId);
      return -1;
    }
    robot->distance(distance);

    // Roadmap builder
    CkwsRoadmapBuilderShPtr roadmapBuilder = attHppPlanner->roadmapBuilderIthProblem(hppProblemId);
    if (roadmapBuilder) {
      roadmapBuilder->distance(distance);
    }

    // path optimizer
    CkwsPathOptimizerShPtr pathOptimizer = 
      attHppPlanner->pathOptimizerIthProblem(hppProblemId);
    if (pathOptimizer) {
      pathOptimizer->distance(distance);
    }
  }
  else {
    ODEBUG1(":setDistanceFunction: wrong problem Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setDiffusionNodePicker(CORBA::UShort inProblemId, 
							const char* inDiffusionNodePickerName)
{
  std::string diffusionNodePickerName(inDiffusionNodePickerName);
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get roadmap builder in hppPlanner object.
    CkwsDiffusingRdmBuilderShPtr roadmapBuilder = 
      KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder, 
			   attHppPlanner->roadmapBuilderIthProblem(hppProblemId));
    // Check that diffusion roadmap builder is set
    if (!roadmapBuilder) {
      ODEBUG1(":setDiffusionNodePicker: roadmap builder is not set or not of type diffusion");
      return -1;
    }
    /* Check that name corresponds to a diffusion node picker factory */
    if (!attHppciServer->diffusionNodePickerFactoryAlreadySet(diffusionNodePickerName)) {
      ODEBUG1(":setDiffusionNodePicker: unknown diffusion node picker.");
      return -1;
    }

    // Create diffusion node picker
    CkwsDiffusionNodePickerShPtr diffusionNodePicker = 
      attHppciServer->createDiffusionNodePicker(diffusionNodePickerName);

    ODEBUG2(":setDiffusionNodePicker: set roadmap builder diffusion node picker to " << diffusionNodePickerName);
    roadmapBuilder->diffusionNodePicker(diffusionNodePicker);
  }
  else {
    ODEBUG1(":setDiffusionNodePicker: wrong problem Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setDiffusionShooter(CORBA::UShort inProblemId, 
						     const char* inDiffusionShooterName,
						     CORBA::Double inStandardDeviation)
{
  std::string diffusionShooterName(inDiffusionShooterName);
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get roadmap builder in hppPlanner object.
    CkwsDiffusingRdmBuilderShPtr roadmapBuilder = 
      KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder, 
			   attHppPlanner->roadmapBuilderIthProblem(hppProblemId));
    // Check that diffusion roadmap builder is set
    if (!roadmapBuilder) {
      ODEBUG1(":setDiffusionShooter: roadmap builder is not set or not of type diffusion");
      return -1;
    }
    /* Check that name corresponds to a diffusion shooter factory */
    if (!attHppciServer->diffusionShooterFactoryAlreadySet(diffusionShooterName)) {
      ODEBUG1(":setDiffusionShooter: unknown diffusion shooter: " << diffusionShooterName);
      return -1;
    }

    // Create diffusion shooter
    CkwsDiffusionShooterShPtr diffusionShooter = 
      attHppciServer->createDiffusionShooter(diffusionShooterName,
					     inStandardDeviation);

    ODEBUG2(":setDiffusionShooter: set roadmap builder diffusion shooter to " << diffusionShooterName);
    roadmapBuilder->diffusionShooter(diffusionShooter);
  }
  else {
    ODEBUG1(":setDiffusionShooter: wrong problem Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setInitialConfig(CORBA::UShort inProblemId, const hppCorbaServer::dofSeq& dofArray) 
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int configDim = (unsigned int)dofArray.length();
  std::vector<double> dofVector;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than nulber of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

    // Compare size of input array with number of degrees of freedom of robot.
    if (configDim != hppRobot->countDofs()) {
      ODEBUG1(":setInitialConfig: robot nb dof is different from config size");
      return -1;
    }
    // Fill dof vector with dof array.
    for (unsigned int iDof=0; iDof<configDim; iDof++) {
      dofVector.push_back(dofArray[iDof]);
    }
    // Create a config for robot initialized with dof vector.
    CkwsConfigShPtr config = CkwsConfig::create(hppRobot, dofVector);
    if (!config) {
      ODEBUG1(":setInitialConfig: cannot create config. Check that robot nb dof is equal to config size");
      return -1;
    }
    
    return (short)attHppPlanner->initConfIthProblem(hppProblemId, config);
  }
  else {
    ODEBUG1(":setInitialConfig: wrong problem Id");
    return -1;
  }
  return 0;
}

CORBA::Short ChppciProblem_impl::setGoalConfig(CORBA::UShort inProblemId, const hppCorbaServer::dofSeq& dofArray) 
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int configDim = (unsigned int)dofArray.length();
  std::vector<double> dofVector;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than nulber of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

    // Compare size of input array with number of degrees of freedom of robot.
    if (configDim != hppRobot->countDofs()) {
      ODEBUG1(":setInitialConfig: robot nb dof is different from config size");
      return -1;
    }

    // Fill dof vector with dof array.
    for (unsigned int iDof=0; iDof<configDim; iDof++) {
      dofVector.push_back(dofArray[iDof]);
    }
    // Create a config for robot initialized with dof vector.
    CkwsConfigShPtr config = CkwsConfig::create(hppRobot, dofVector);
    if (!config) {
      ODEBUG1(":setGoalConfig: cannot create config. Check that robot nb dof is equal to config size");
      return -1;
    }
    return (short)attHppPlanner->goalConfIthProblem(hppProblemId, config);
  }
  else {
    ODEBUG1(":setGoalConfig: wrong problem Id");
    return -1;
  }
  return 0;
}

hppCorbaServer::dofSeq* ChppciProblem_impl::getInitialConfig(CORBA::UShort inProblemId)
  throw(CORBA::SystemException)
{
  hppCorbaServer::dofSeq *dofArray;
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

    std::vector<double> dofVector;
    CkwsConfigShPtr config = attHppPlanner->initConfIthProblem(hppProblemId);

    if (config) {
      // by Yoshida 06/08/25
      unsigned int deviceDim = config->size();

      // cerr<<"deviceDim "<<deviceDim<<" [ ";
      dofArray = new hppCorbaServer::dofSeq();
      dofArray->length(deviceDim);

      for(unsigned int i=0; i<deviceDim; i++){
	(*dofArray)[i] = config->dofValue(i);
      }
      return dofArray;
    }
    else {
      ODEBUG1(":getInitialConfig: no initial configuration defined");
      dofArray = new hppCorbaServer::dofSeq(1);
      return dofArray;
    }
  }

  else {
    ODEBUG1(":getInitialConfig: wrong problem Id");
    dofArray = new hppCorbaServer::dofSeq(1);
    return dofArray;
  }

  return new hppCorbaServer::dofSeq(1);
}


hppCorbaServer::dofSeq* ChppciProblem_impl::getGoalConfig(CORBA::UShort inProblemId)
  throw(CORBA::SystemException)
{
  hppCorbaServer::dofSeq *dofArray;
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);

    std::vector<double> dofVector;
    CkwsConfigShPtr config = attHppPlanner->goalConfIthProblem(hppProblemId);

    if (config) {
      // by Yoshida 06/08/25
      unsigned int deviceDim = config->size();

      // cerr<<"deviceDim "<<deviceDim<<" [ ";
      dofArray = new hppCorbaServer::dofSeq();
      dofArray->length(deviceDim);

      for(unsigned int i=0; i<deviceDim; i++){
	(*dofArray)[i] = config->dofValue(i);
      }
      return dofArray;
    }
    else {
      ODEBUG1(":getInitialConfig: no initial configuration defined");
      dofArray = new hppCorbaServer::dofSeq(1);
      return dofArray;
    }
  }

  else {
    ODEBUG1(":getInitialConfig: wrong problem Id");
    dofArray = new hppCorbaServer::dofSeq(1);
    return dofArray;
  }

  return new hppCorbaServer::dofSeq(1);
}


CORBA::Short ChppciProblem_impl::initializeProblem()
{
  ktStatus status  = attHppPlanner->initializeProblem();
  return (CORBA::Short)status;
}


CORBA::Short ChppciProblem_impl::solveOneProblem(CORBA::UShort inProblemId, CORBA::Short& inLastPathId, CORBA::Double& pathLength)
{
  
  ktStatus status = KD_ERROR;
  inLastPathId = 0 ;
  pathLength = 0;
  
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    status = attHppPlanner->solveOneProblem(hppProblemId);
  }
 

  inLastPathId = attHppPlanner->getNbPaths(hppProblemId) - 1;
  if (inLastPathId > -1) {
    pathLength = attHppPlanner->getPath(hppProblemId, inLastPathId)->length();
  }
  else {
    ODEBUG1(":solveOneProblem: no path in hppProblem " << hppProblemId);
  }

  return (CORBA::Short)status;
}


CORBA::Short ChppciProblem_impl::solve()
{
  ktStatus status  = attHppPlanner->solve();
  return (CORBA::Short)status;
}

CORBA::Short ChppciProblem_impl::interruptPathPlanning()
{
  attHppPlanner->interruptPathPlanning();
  return 0;
}

CORBA::Short ChppciProblem_impl::optimizePath(CORBA::UShort inProblemId, CORBA::UShort inPathId)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int pathId = (unsigned int)inPathId;
  
  if (hppProblemId > attHppPlanner->getNbHppProblems() - 1) {
    ODEBUG1(":optimizePath: wrong problem id");
    return -1;
  }
  
  if (pathId > attHppPlanner->getNbPaths(hppProblemId) - 1) {
    ODEBUG1(":optimizePath: wrong path id");
    return -1;
  }

  ktStatus status  = attHppPlanner->optimizePath((unsigned int) inProblemId, (unsigned int) inPathId);
  return (CORBA::Short)status;
}

CORBA::Double ChppciProblem_impl::pathLength(CORBA::UShort inProblemId, CORBA::UShort inPathId)
{
  double length = -1.0;

  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int pathId = (unsigned int)inPathId;
  
  if (hppProblemId > attHppPlanner->getNbHppProblems() - 1) {
    ODEBUG1(":pathLength: wrong problem id");
    return -1.0;
  }
  
  if (pathId > attHppPlanner->getNbPaths(hppProblemId) - 1) {
    ODEBUG1(":pathLength: wrong path id");
    return -1.0;
  }

  length = attHppPlanner->getPath(hppProblemId, pathId)->length();
  return length;
}

hppCorbaServer::dofSeq* ChppciProblem_impl::configAtDistance(CORBA::UShort inProblemId, CORBA::UShort inPathId, CORBA::Double atDistance) 
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int pathId = (unsigned int)inPathId;

  if (hppProblemId > attHppPlanner->getNbHppProblems() - 1) {
    ODEBUG1(":configAtDistance: wrong problem id");
    return new hppCorbaServer::dofSeq(0, 0, NULL, true);
  }

  if (pathId > attHppPlanner->getNbPaths(hppProblemId) - 1) {
    ODEBUG1(":configAtDistance: wrong path id");
    return new hppCorbaServer::dofSeq(0, 0, NULL, true);
  }

  double length = attHppPlanner->getPath(hppProblemId, pathId)->length();

  //get the nb of dof of the robot in the problem
  unsigned int nbDofRobot = attHppPlanner->robotIthProblem(inProblemId)->countDofs();

  //init the seqdof
  // Allocate result now that the size is known.
  CORBA::ULong size = (CORBA::ULong)nbDofRobot;
  double* DofList = hppCorbaServer::dofSeq::allocbuf(size);
  hppCorbaServer::dofSeq* dofSeq = new hppCorbaServer::dofSeq(size, size, DofList, true);
  
  //get the config of the robot on the path
  if (atDistance > length) {
    ODEBUG1(":configAtParam: param out of range (longer than path Length) " << "Param : "<< atDistance << " length : " << length);
   
  }
  else {
     CkwsConfigShPtr inConfig ;
     inConfig = attHppPlanner->getPath(hppProblemId, pathId)->configAtDistance(atDistance) ; 
     //convert the config in dofseq
     for ( unsigned int i = 0 ; i < inConfig->size() ; i++){
       DofList[i] =  inConfig->dofValue(i) ;
     }
  }

  

  return dofSeq;
}

/// \brief set tolerance to the objects in the planner
CORBA::Short ChppciProblem_impl::setObstacleTolerance(CORBA::UShort inProblemId, CORBA::Double tolerance)
    throw(CORBA::SystemException)
{
  // get the planner  
  unsigned int hppProblemId = (unsigned int)inProblemId;
  // get object hppPlanner of Corba server.

  if(!attHppPlanner){
    ODEBUG1(":setObstacleTolerance: problem " << hppProblemId << " not found");
    return -1;
  }

  std::vector<CkcdObjectShPtr> oList = attHppPlanner->obstacleList();

  if(oList.size() == 0)
    cerr << " there are no obstacle in problem " << hppProblemId << std::endl;

  for(unsigned int i =0; i<oList.size(); i++){
    oList[i]->tolerance(tolerance);
    ODEBUG1(":setObstacleTolerance: tolerance " << tolerance << " set to obstacle " << i);
  }
  return 0;
}
