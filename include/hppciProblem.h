/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#ifndef HPPCI_PROBLEM_H
#define HPPCI_PROBLEM_H

#include <vector>
#include "hppciProblemServer.hh"

class ChppciServer;
/**
 * \brief Implementation of corba interface ChppciProblem.
 */
class ChppciProblem_impl : public virtual POA_ChppciProblem {
public:
  ChppciProblem_impl(ChppciServer* inHppciServer);
  virtual CORBA::Short setSteeringMethod(CORBA::Short inProblemId, 
					 const char* inSteeringMethod, CORBA::Boolean inOriented);
  virtual CORBA::Short setRoadmapbuilder(CORBA::Short inProblemId, const char* inRoadmapBuilder);
  virtual CORBA::Short setPathOptimizer(CORBA::Short inProblemId, const char* inPathOptimizer);
  virtual CORBA::Short setInitialConfig(CORBA::Short inProblemId, const dofSeq& dofArray);
  virtual CORBA::Short setGoalConfig(CORBA::Short inProblemId, const dofSeq& dofArray);
  virtual CORBA::Short initializeProblem();
  virtual CORBA::Short solveOneProblem(CORBA::Short inProblemId, CORBA::Short& inLastPathId, CORBA::Double& pathLength) ;
  virtual CORBA::Short solve();
  virtual dofSeq* configAtDistance(CORBA::Short inProblemId, CORBA::Short pathId, CORBA::Double pathLength, CORBA::Double atDistance) ;
  /// \brief set tolerance to the objects in the planner
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
