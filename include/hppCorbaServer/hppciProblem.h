/*
  Copyright 2006 LAAS-CNRS

  Author: Florent Lamiraux

*/

#ifndef HPPCI_PROBLEM_H
#define HPPCI_PROBLEM_H

#include <vector>

#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION
#include "hppciProblemServer.hh"

class ChppciServer;
/**
 * \brief Implementation of corba interface ChppciProblem.
 */
class ChppciProblem_impl : public virtual POA_hppCorbaServer::ChppciProblem {
public:
  ChppciProblem_impl(ChppciServer* inHppciServer);
  /// \brief Comment in interface hppCorbaServer::ChppciProblem::setSteeringMethod.
  virtual CORBA::Short setSteeringMethod(CORBA::UShort inProblemId, 
					 const char* inSteeringMethod, CORBA::Boolean inOriented);

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::setRoadmapbuilder
  virtual CORBA::Short setRoadmapbuilder(CORBA::UShort inProblemId, const char* inRoadmapBuilder,
					 CORBA::Boolean inDisplay);

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::setDiffusingNode.
  virtual CORBA::Short setDiffusingNode(CORBA::UShort inProblemId, const char* inDiffusingNode);

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::setPathOptimizer
  virtual CORBA::Short setPathOptimizer(CORBA::UShort inProblemId, const char* inPathOptimizer,
					CORBA::UShort inMaxNumberLoop);

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::setDistanceFunction
  virtual CORBA::Short setDistanceFunction(CORBA::UShort inProblemId, const char* inDistanceName, CORBA::Boolean inOriented);

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::setDiffusionNodePicker
  virtual CORBA::Short setDiffusionNodePicker(CORBA::UShort inProblemId, 
					      const char* inDiffusionNodePickerName);

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::setDiffusionShooter
  virtual CORBA::Short setDiffusionShooter(CORBA::UShort inProblemId, 
					   const char* inDiffusionShooterName,
					   CORBA::Double inStandardDeviation);

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::setInitialConfig
  virtual CORBA::Short setInitialConfig(CORBA::UShort inProblemId, const hppCorbaServer::dofSeq& dofArray);

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::setGoalConfig
  virtual CORBA::Short setGoalConfig(CORBA::UShort inProblemId, const hppCorbaServer::dofSeq& dofArray);

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::getInitialConfig
  virtual hppCorbaServer::dofSeq* getInitialConfig(CORBA::UShort inProblemId)
    throw(CORBA::SystemException);

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::getGoalConfig
  virtual hppCorbaServer::dofSeq* getGoalConfig(CORBA::UShort inProblemId)
    throw(CORBA::SystemException);


  /// \brief Comment in interface hppCorbaServer::ChppciProblem::initializeProblem
  virtual CORBA::Short initializeProblem();

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::solveOneProblem
  virtual CORBA::Short solveOneProblem(CORBA::UShort inProblemId, CORBA::Short& inLastPathId, CORBA::Double& pathLength) ;

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::solve
  virtual CORBA::Short solve();

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::interruptPathPlanning
  virtual CORBA::Short interruptPathPlanning();

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::optimizePath
  virtual CORBA::Short optimizePath(CORBA::UShort inProblemId, CORBA::UShort inPathId);

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::configAtDistance
  virtual hppCorbaServer::dofSeq* configAtDistance(CORBA::UShort inProblemId, CORBA::UShort pathId, CORBA::Double pathLength, CORBA::Double atDistance) ;

  /// \brief Comment in interface hppCorbaServer::ChppciProblem::setObstacleTolerance
  virtual CORBA::Short setObstacleTolerance(CORBA::UShort inProblemId, CORBA::Double tolerance)
    throw(CORBA::SystemException);

private:
  /// \brief Pointer to the ChppciServer owning this object
  ChppciServer* attHppciServer;
  /// \brief Pointer to hppPlanner object of hppciServer.
  /// Instantiated at construction.
  ChppPlanner *attHppPlanner;
};  


#endif
