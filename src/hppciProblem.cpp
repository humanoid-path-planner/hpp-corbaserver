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
      ODEBUG1("unknown steering method.");
      return -1;
    }

    // Create steering method
    CkwsSteeringMethodShPtr steeringMethod = 
      attHppciServer->createSteeringMethod(steeringMethodName, inOriented);

    hppRobot->steeringMethod(steeringMethod);
  }
  else {
    ODEBUG1("setSteeringMethod: wrong robot Id");
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
    if (roadmapBuilderName == "basic") {
      CkwsBasicRdmBuilderShPtr roadmapBuilder = CkwsBasicRdmBuilder::create(roadmap, penetration);
      status = attHppPlanner->roadmapBuilderIthProblem(hppProblemId, roadmapBuilder, inDisplay);
      return status;      
    } else if (roadmapBuilderName == "diffusing") {
      CkwsDiffusingRdmBuilderShPtr roadmapBuilder = CkwsDiffusingRdmBuilder::create(roadmap, penetration);
      status = attHppPlanner->roadmapBuilderIthProblem(hppProblemId, roadmapBuilder, inDisplay);
      return status;      
    } else if (roadmapBuilderName == "bi-diffusing") {
      CkwsDiffusingRdmBuilderShPtr roadmapBuilder = CkwsDiffusingRdmBuilder::create(roadmap, penetration);
      roadmapBuilder->diffuseFromProblemGoal(true);
      status = attHppPlanner->roadmapBuilderIthProblem(hppProblemId, roadmapBuilder, inDisplay);
      return status;      
    } else if (roadmapBuilderName == "IPP") {
      CkwsIPPRdmBuilderShPtr roadmapBuilder = CkwsIPPRdmBuilder::create(roadmap, penetration);
      status = attHppPlanner->roadmapBuilderIthProblem(hppProblemId, roadmapBuilder, inDisplay);
    } else if (roadmapBuilderName == "visibility") {
      ChppVisRdmBuilderShPtr roadmapBuilder = ChppVisRdmBuilder::create(roadmap, penetration);
      status = attHppPlanner->roadmapBuilderIthProblem(hppProblemId, roadmapBuilder, inDisplay);
    } else if (roadmapBuilderName == "PCA<diffusing>") {
      KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<CkwsDiffusingRdmBuilder>) roadmapBuilder = 
	CkwsPlusPCARdmBuilder<CkwsDiffusingRdmBuilder>::create(roadmap, penetration);
      status = attHppPlanner->roadmapBuilderIthProblem(hppProblemId, roadmapBuilder, inDisplay);
    } else {
      ODEBUG1("unknown roadmap builder");
      return -1;
    }
  }
  else {
    ODEBUG1("setRoadmapbuilder: wrong robot Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setPathOptimizer(CORBA::Short inProblemId, const char* inPathOptimizerName)
{
  std::string pathOptimizerName(inPathOptimizerName);

  ktStatus status;
  unsigned int hppProblemId = (unsigned int)inProblemId;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  ODEBUG2("hppciProblem.cpp: setPathOptimizer: nbProblems " << nbProblems << "problem id " 
	  << inProblemId << ", pathOptimizerName " <<pathOptimizerName);
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);
    if (pathOptimizerName == "clear") {
      CkwsClearOptimizerShPtr pathOptimizer = CkwsClearOptimizer::create();
      status = attHppPlanner->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
      ODEBUG2("ChppciProblem_impl::setPathOptimizer: clear path optimizer set.");
      return (CORBA::Short)status;
    } else if (pathOptimizerName == "adaptiveShortcut") {
      CkwsAdaptiveShortcutOptimizerShPtr pathOptimizer = CkwsAdaptiveShortcutOptimizer::create();
      status = attHppPlanner->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
      ODEBUG2("ChppciProblem_impl::setPathOptimizer: adaptive shortcut path optimizer set");
      return (CORBA::Short)status;
    } else if (pathOptimizerName == "random") {
      CkwsRandomOptimizerShPtr pathOptimizer = CkwsRandomOptimizer::create();
      status = attHppPlanner->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
      ODEBUG2("ChppciProblem_impl::setPathOptimizer: random path optimizer set");
      return (CORBA::Short)status;
    } else if (pathOptimizerName == "none") {
      CkwsPathOptimizerShPtr pathOptimizer;
      status = attHppPlanner->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
      ODEBUG2("ChppciProblem_impl::setPathOptimizer: no path optimizer set");
      return (CORBA::Short)status;
    } else {
      ODEBUG2("ChppciProblem_impl::setPathOptimizer: unknown path optimizer");
      return -1;
    }
  }
  else {
    ODEBUG1("setPathOptimizer: wrong robot Id");
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
    // Get roadmap builder in hppPlanner object.
    CkwsRoadmapBuilderShPtr roadmapBuilder = attHppPlanner->roadmapBuilderIthProblem(hppProblemId);
    // Check that roadmap builder is set
    if (!roadmapBuilder) {
      ODEBUG1(" setDistanceFunction: roadmap builder is not set");
      return -1;
    }
    /* Check that name corresponds to a distance function factory */
    if (!attHppciServer->distanceFactoryAlreadySet(distanceName)) {
      ODEBUG1(" unknown distance function.");
      return -1;
    }

    // Create distance function
    CkwsDistanceShPtr distance = 
      attHppciServer->createDistanceFunction(distanceName, inOriented);

    ODEBUG2(" set roadmap builder distance function to " << distanceName);
    roadmapBuilder->distance(distance);
  }
  else {
    ODEBUG1(" setDistanceFunction: wrong robot Id");
    return -1;
  }

  return 0;
}

CORBA::Short ChppciProblem_impl::setInitialConfig(CORBA::Short inProblemId, const dofSeq& dofArray) 
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
      ODEBUG1("setInitialConfig: cannot create config. Check that robot nb dof is equal to config size");
      return -1;
    }
    
    return (short)attHppPlanner->initConfIthProblem(hppProblemId, config);
  }
  else {
    ODEBUG1("setInitialConfig: wrong robot Id");
    return -1;
  }
  return 0;
}

CORBA::Short ChppciProblem_impl::setGoalConfig(CORBA::Short inProblemId, const dofSeq& dofArray) 
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
      ODEBUG1("setGoalConfig: cannot create config. Check that robot nb dof is equal to config size");
      return -1;
    }
    
    return (short)attHppPlanner->goalConfIthProblem(hppProblemId, config);
  }
  else {
    ODEBUG1("setGoalConfig: wrong robot Id");
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
    ODEBUG1("solveOneProblem: no path in hppProblem " << hppProblemId);
  }

  return (CORBA::Short)status;
}


CORBA::Short ChppciProblem_impl::solve()
{
  ktStatus status  = attHppPlanner->solve();
  return (CORBA::Short)status;
}

CORBA::Short ChppciProblem_impl::optimizePath(CORBA::Short inProblemId, CORBA::Short inPathId)
{
  ktStatus status  = attHppPlanner->optimizePath((unsigned int) inProblemId, (unsigned int) inPathId);
  return (CORBA::Short)status;
}

dofSeq* ChppciProblem_impl::configAtDistance(CORBA::Short inProblemId, CORBA::Short pathId, CORBA::Double pathLength, CORBA::Double atDistance) {

  dofSeq* inDofSeq ;

  // get the planner  
  unsigned int hppProblemId = (unsigned int)inProblemId;

  //get the nb of dof of the robot in the problem
  unsigned int nbDofRobot = attHppPlanner->robotIthProblem(inProblemId)->countDofs();

  //init the seqdof
  // Allocate result now that the size is known.
  CORBA::ULong size = (CORBA::ULong)nbDofRobot;
  double* DofList = dofSeq::allocbuf(size);
  inDofSeq = new dofSeq(size, size, DofList, true);
  
  //get the config of the robot on the path
  if (atDistance > pathLength) {
    ODEBUG1("configAtParam: param out of range (longer than path Length) " << "Param : "<< atDistance << " length : " << pathLength);
   
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
    ODEBUG1("problem " << hppProblemId << " not found");
    return -1;
  }

  std::vector<CkcdObjectShPtr> oList = attHppPlanner->obstacleList();

  if(oList.size() == 0)
    cerr << " there are no obstacle in problem " << hppProblemId << std::endl;

  for(unsigned int i =0; i<oList.size(); i++){
    oList[i]->tolerance(tolerance);
    ODEBUG1("tolerance " << tolerance << " set to obstacle " << i);
  }
  return 0;
}
