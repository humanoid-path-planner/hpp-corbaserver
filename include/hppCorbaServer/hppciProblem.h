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
  /// \brief Comment in interface ChppciProblem::setSteeringMethod.
  virtual CORBA::Short setSteeringMethod(CORBA::Short inProblemId, 
					 const char* inSteeringMethod, CORBA::Boolean inOriented);

  /// \brief Comment in interface ChppciProblem::setRoadmapbuilder
  virtual CORBA::Short setRoadmapbuilder(CORBA::Short inProblemId, const char* inRoadmapBuilder,
					 CORBA::Boolean inDisplay);

  /// \brief Comment in interface ChppciProblem::setDiffusingNode.
  virtual CORBA::Short setDiffusingNode(CORBA::Short inProblemId, const char* inDiffusingNode);

  /// \brief Comment in interface ChppciProblem::setPathOptimizer
  virtual CORBA::Short setPathOptimizer(CORBA::Short inProblemId, const char* inPathOptimizer);

  /// \brief Comment in interface ChppciProblem::setDistanceFunction
  virtual CORBA::Short setDistanceFunction(CORBA::Short inProblemId, const char* inDistanceName, CORBA::Boolean inOriented);

  /// \brief Comment in interface ChppciProblem::setDiffusionNodePicker
  virtual CORBA::Short setDiffusionNodePicker(CORBA::Short inProblemId, 
					      const char* inDiffusionNodePickerName);

  /// \brief Comment in interface ChppciProblem::setDiffusionShooter
  virtual CORBA::Short setDiffusionShooter(CORBA::Short inProblemId, 
					   const char* inDiffusionShooterName,
					   CORBA::Double inStandardDeviation);

  /// \brief Comment in interface ChppciProblem::setInitialConfig
  virtual CORBA::Short setInitialConfig(CORBA::Short inProblemId, const hppCorbaServer::dofSeq& dofArray);

  /// \brief Comment in interface ChppciProblem::setGoalConfig
  virtual CORBA::Short setGoalConfig(CORBA::Short inProblemId, const hppCorbaServer::dofSeq& dofArray);

  /// \brief Comment in interface ChppciProblem::initializeProblem
  virtual CORBA::Short initializeProblem();

  /// \brief Comment in interface ChppciProblem::solveOneProblem
  virtual CORBA::Short solveOneProblem(CORBA::Short inProblemId, CORBA::Short& inLastPathId, CORBA::Double& pathLength) ;

  /// \brief Comment in interface ChppciProblem::solve
  virtual CORBA::Short solve();

  /// \brief Comment in interface ChppciProblem::optimizePath
  virtual CORBA::Short optimizePath(CORBA::Short inProblemId, CORBA::Short inPathId);

  /// \brief Comment in interface ChppciProblem::configAtDistance
  virtual hppCorbaServer::dofSeq* configAtDistance(CORBA::Short inProblemId, CORBA::Short pathId, CORBA::Double pathLength, CORBA::Double atDistance) ;

  /// \brief Comment in interface ChppciProblem::setObstacleTolerance
  virtual CORBA::Short setObstacleTolerance(CORBA::Short inProblemId, CORBA::Double tolerance)
    throw(CORBA::SystemException);

private:
  /// \brief Pointer to the ChppciServer owning this object
  ChppciServer* attHppciServer;
  /// \brief Pointer to hppPlanner object of hppciServer.
  /// Instantiated at construction.
  ChppPlanner *attHppPlanner;
};  


#endif
