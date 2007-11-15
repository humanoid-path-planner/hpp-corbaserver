/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#include <iostream>
#include "hppCorbaServer/hppciServer.h"
#include "hppCorbaServer/hppciProblem.h"
#include "flicSteeringMethod.h"
#include "reedsSheppSteeringMethod.h"
#include "kwsPlusRoadmap.h"
#include "hppVisRdmBuilder.h"

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

    // Create steering method
    if (steeringMethodName == "linear") {
      CkwsSMLinearShPtr steeringMethod = CkwsSMLinear::create(inOriented);
      hppRobot->steeringMethod(steeringMethod);

    } else if (steeringMethodName == "flic") {
      CflicSteeringMethodShPtr steeringMethod = CflicSteeringMethod::create(inOriented);
      hppRobot->steeringMethod(steeringMethod);
    } else if (steeringMethodName == "rs") {
      CreedsSheppSteeringMethodShPtr steeringMethod = CreedsSheppSteeringMethod::create(1.0, inOriented);
      hppRobot->steeringMethod(steeringMethod);
    } else {
      cerr << "ChppciProblem_impl::setSteeringMethod: unknown steering method." << endl;
    }
  }
  else {
    cerr << "ChppciProblem_impl::setSteeringMethod: wrong robot Id" << endl;
    return KD_ERROR;
  }

  return KD_OK;
}

CORBA::Short ChppciProblem_impl::setRoadmapbuilder(CORBA::Short inProblemId, const char* inRoadmapBuilderName)
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
    CkwsPlusRoadmapShPtr roadmap = CkwsPlusRoadmap::create(hppRobot, std::string("kwsPlusRoadmap"));
    // Set arbitrary penetration.
    double penetration = 0.05;

    // Create roadmapBuilder
    if (roadmapBuilderName == "basic") {
      CkwsBasicRdmBuilderShPtr roadmapBuilder = CkwsBasicRdmBuilder::create(roadmap, penetration);
      status = attHppPlanner->roadmapBuilderIthProblem(hppProblemId, roadmapBuilder);
      return status;      
    } else if (roadmapBuilderName == "diffusing") {
      CkwsDiffusingRdmBuilderShPtr roadmapBuilder = CkwsDiffusingRdmBuilder::create(roadmap, penetration);
      status = attHppPlanner->roadmapBuilderIthProblem(hppProblemId, roadmapBuilder);
      return status;      
    } else if (roadmapBuilderName == "IPP") {
      CkwsIPPRdmBuilderShPtr roadmapBuilder = CkwsIPPRdmBuilder::create(roadmap, penetration);
      status = attHppPlanner->roadmapBuilderIthProblem(hppProblemId, roadmapBuilder);
    } else if (roadmapBuilderName == "visibility") {
      ChppVisRdmBuilderShPtr roadmapBuilder = ChppVisRdmBuilder::create(roadmap, penetration);
      status = attHppPlanner->roadmapBuilderIthProblem(hppProblemId, roadmapBuilder);
    } else {
      cerr << "ChppciProblem_impl::setRoadmapbuilder: unknown roadmap builder" << endl;
      return KD_ERROR;
    }
  }
  else {
    cerr << "ChppciProblem_impl::setRoadmapbuilder: wrong robot Id" << endl;
    return KD_ERROR;
  }

  return KD_OK;
}

CORBA::Short ChppciProblem_impl::setPathOptimizer(CORBA::Short inProblemId, const char* inPathOptimizerName)
{
  std::string pathOptimizerName(inPathOptimizerName);

  ktStatus status;
  unsigned int hppProblemId = (unsigned int)inProblemId;

  unsigned int nbProblems = attHppPlanner->getNbHppProblems();

  cerr<<"ChppciProblem_impl::setPathOptimizer: nbProblems "<<nbProblems
      <<"problem id "<<inProblemId<<", pathOptimizerName "
      <<pathOptimizerName<<endl;
  // Test that rank is less than number of robots in vector.
  if (hppProblemId < nbProblems) {
    // Get robot in hppPlanner object.
    CkppDeviceComponentShPtr hppRobot = attHppPlanner->robotIthProblem(hppProblemId);
    if (pathOptimizerName == "clear") {
      CkwsClearOptimizerShPtr pathOptimizer = CkwsClearOptimizer::create();
      status = attHppPlanner->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
      cerr<<"ChppciProblem_impl::setPathOptimizer: clear path optimizer set."<<endl;
      return (CORBA::Short)status;
    } else if (pathOptimizerName == "adaptiveShortcut") {
      CkwsAdaptiveShortcutOptimizerShPtr pathOptimizer = CkwsAdaptiveShortcutOptimizer::create();
      status = attHppPlanner->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
      cerr<<"ChppciProblem_impl::setPathOptimizer: adaptive shortcut path optimizer set."<<endl;
      return (CORBA::Short)status;
    } else if (pathOptimizerName == "random") {
      CkwsRandomOptimizerShPtr pathOptimizer = CkwsRandomOptimizer::create();
      status = attHppPlanner->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
      cerr<<"ChppciProblem_impl::setPathOptimizer: random path optimizer set."<<endl;
      return (CORBA::Short)status;
    } else if (pathOptimizerName == "none") {
      CkwsPathOptimizerShPtr pathOptimizer;
      status = attHppPlanner->pathOptimizerIthProblem(hppProblemId, pathOptimizer);
      cerr<<"ChppciProblem_impl::setPathOptimizer: no path optimizer set."<<endl;
      return (CORBA::Short)status;
    } else {
      cerr << "ChppciProblem_impl::setPathOptimizer: unknown path optimizer" << endl;
      return KD_ERROR;
    }
  }
  else {
    cerr << "ChppciProblem_impl::setPathOptimizer: wrong robot Id" << endl;
    return KD_ERROR;
  }

  return KD_OK;
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
      cerr << "ChppciProblem_impl::setInitialConfig: cannot create config. Check that robot nb dof is equal to config size" << endl;
      return KD_ERROR;
    }
    
    return (short)attHppPlanner->initConfIthProblem(hppProblemId, config);
  }
  else {
    cerr << "ChppciProblem_impl::setInitialConfig: wrong robot Id" << endl;
    return KD_ERROR;
  }
  return KD_OK;
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
      cerr << "ChppciProblem_impl::setGoalConfig: cannot create config. Check that robot nb dof is equal to config size" << endl;
      return KD_ERROR;
    }
    
    return (short)attHppPlanner->goalConfIthProblem(hppProblemId, config);
  }
  else {
    cerr << "ChppciProblem_impl::setGoalConfig: wrong robot Id" << endl;
    return KD_ERROR;
  }
  return KD_OK;
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
    cerr << "ChppciProblem_impl::solveOneProblem: no path in hppProblem "
	 << hppProblemId << endl;
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
    cerr << "ChppciProblem_impl::ConfigAtParam: Param out of range (longer than path Length) "
	 << "Param : "<< atDistance << " length : " << pathLength << endl;
   
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
    cerr<<"problem "<<hppProblemId<<" not found"<<endl;
    return -1;
  }

  std::vector<CkcdObjectShPtr> oList = attHppPlanner->obstacleList();

  if(oList.size() == 0)
    cerr<<" there are no obstacle in problem "<<hppProblemId<<endl;

  for(unsigned int i =0; i<oList.size(); i++){
    oList[i]->tolerance(tolerance);
    cerr<<"tolerance "<<tolerance<<" set to obstacle "<<i<<endl;
  }
  
  return 0;
}

short ChppciProblem_impl::drawRoadmap(CORBA::Short inProblemId)
  throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;

  if (hppProblemId >= attHppPlanner->getNbHppProblems()) {
    cerr << "ChppciProblem_impl::drawRoadmap: inProblemId = " << hppProblemId << " >= " 
	 <<  attHppPlanner->getNbHppProblems() << endl;
    return -1;
  }
  if (attHppPlanner->drawRoadmap(hppProblemId) == KD_ERROR) {
    return -1;
  }
  return 0;
}

short ChppciProblem_impl::stopDrawingRoadmap(CORBA::Short inProblemId)
    throw(CORBA::SystemException)
{
  unsigned int hppProblemId = (unsigned int)inProblemId;

  if (hppProblemId >= attHppPlanner->getNbHppProblems()) {
    cerr << "ChppciProblem_impl::drawRoadmap: inProblemId = " << hppProblemId << " >= " 
	 <<  attHppPlanner->getNbHppProblems() << endl;
    return -1;
  }
  if (attHppPlanner->stopdrawingRoadmap(hppProblemId) == KD_ERROR) {
    return -1;
  }
  return 0;
}

