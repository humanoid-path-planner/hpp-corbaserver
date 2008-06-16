/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#include <iostream>
#include "hppCorbaServer/hppciServer.h"
#include "hppCorbaServer/hppciProblem.h"
#include "flicSteeringMethod.h"
#include "reedsSheppSteeringMethod.h"
#include "hppVisRdmBuilder.h"
#include "kwsPlusPCARdmBuilder.h"

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


CORBA::Short ChppciProblem_impl::setSteeringMethod(CORBA::Short inProblemId, 
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
    ODEBUG1(":setSteeringMethod: wrong robot Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setRoadmapbuilder(CORBA::Short inProblemId, const char* inRoadmapBuilderName,
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
    ODEBUG1(":setRoadmapbuilder: wrong robot Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setDiffusingNode(CORBA::Short inProblemId, 
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
    ODEBUG1(":setDiffusingNode: wrong robot Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setPathOptimizer(CORBA::Short inProblemId, 
						  const char* inPathOptimizerName,
						  CORBA::Short inMaxNumberLoop)
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
    ODEBUG1(":setPathOptimizer: wrong robot Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setDistanceFunction(CORBA::Short inProblemId, const char* inDistanceName, CORBA::Boolean inOriented)
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
    ODEBUG1(":setDistanceFunction: wrong robot Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setDiffusionNodePicker(CORBA::Short inProblemId, 
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
    ODEBUG1(":setDiffusionNodePicker: wrong robot Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setDiffusionShooter(CORBA::Short inProblemId, 
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
    ODEBUG1(":setDiffusionShooter: wrong robot Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setInitialConfig(CORBA::Short inProblemId, const hppCorbaServer::dofSeq& dofArray) 
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int configDim = (unsigned int)dofArray.length();
  std::vector<double> dofVector;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than nulber of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);
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
    ODEBUG1(":setInitialConfig: wrong robot Id");
    return -1;
  }
  return 0;
}

CORBA::Short ChppciProblem_impl::setGoalConfig(CORBA::Short inProblemId, const hppCorbaServer::dofSeq& dofArray) 
{
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int configDim = (unsigned int)dofArray.length();
  std::vector<double> dofVector;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than nulber of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);
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
    ODEBUG1(":setGoalConfig: wrong robot Id");
    return -1;
  }
  return 0;
}

CORBA::Short ChppciProblem_impl::initializeProblem()
{
  ktStatus status  = attHppPlanner->initializeProblem();
  return (CORBA::Short)status;
}


CORBA::Short ChppciProblem_impl::solveOneProblem(CORBA::Short inProblemId, CORBA::Short& inLastPathId, CORBA::Double& pathLength)
{
  
  ktStatus status = KD_ERROR;
  inLastPathId = 0 ;
  pathLength = 0;
  
  unsigned int hppProblemId = (unsigned int)inProblemId;
  unsigned int nbProblems = attHppPlanner->getNbHppProblems();
  
  // Test that rank is less than nulber of robots in vector.
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

CORBA::Short ChppciProblem_impl::optimizePath(CORBA::Short inProblemId, CORBA::Short inPathId)
{
  ktStatus status  = attHppPlanner->optimizePath((unsigned int) inProblemId, (unsigned int) inPathId);
  return (CORBA::Short)status;
}

hppCorbaServer::dofSeq* ChppciProblem_impl::configAtDistance(CORBA::Short inProblemId, CORBA::Short pathId, CORBA::Double pathLength, CORBA::Double atDistance) {

  hppCorbaServer::dofSeq* inDofSeq ;

  // get the planner  
  unsigned int hppProblemId = (unsigned int)inProblemId;

  //get the nb of dof of the robot in the problem
  unsigned int nbDofRobot = attHppPlanner->robotIthProblem(inProblemId)->countDofs();

  //init the seqdof
  // Allocate result now that the size is known.
  CORBA::ULong size = (CORBA::ULong)nbDofRobot;
  double* DofList = hppCorbaServer::dofSeq::allocbuf(size);
  inDofSeq = new hppCorbaServer::dofSeq(size, size, DofList, true);
  
  //get the config of the robot on the path
  if (atDistance > pathLength) {
    ODEBUG1(":configAtParam: param out of range (longer than path Length) " << "Param : "<< atDistance << " length : " << pathLength);
   
  }
  else {
     CkwsConfigShPtr inConfig ;
     inConfig = attHppPlanner->getPath(hppProblemId, pathId)->configAtDistance(atDistance) ; 
     //convert the config in dofseq
     for ( unsigned int i = 0 ; i < inConfig->size() ; i++){
       DofList[i] =  inConfig->dofValue(i) ;
     }
  }

  

  return inDofSeq;
}

/// \brief set tolerance to the objects in the planner
CORBA::Short ChppciProblem_impl::setObstacleTolerance(CORBA::Short inProblemId, CORBA::Double tolerance)
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
