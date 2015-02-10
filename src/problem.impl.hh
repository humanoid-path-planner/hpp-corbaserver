// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_PROBLEM_IMPL_HH
# define HPP_CORBASERVER_PROBLEM_IMPL_HH
# include <vector>

# include "hpp/corbaserver/fwd.hh"
# include "common.hh"
# include "problem.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      using CORBA::Long;
      using CORBA::ULong;
      /// \brief Implement CORBA interface ``Problem''.
      class Problem : public virtual POA_hpp::corbaserver::Problem
      {
      public:
	Problem (corbaServer::Server* server);

	virtual void
	setInitialConfig (const hpp::floatSeq& dofArray) throw (hpp::Error);

	virtual hpp::floatSeq*
	getInitialConfig () throw (hpp::Error);

	virtual void
	addGoalConfig (const hpp::floatSeq& dofArray) throw (hpp::Error);

	virtual hpp::floatSeqSeq*	getGoalConfigs () throw (hpp::Error);

	virtual void
	resetGoalConfigs ()  throw (hpp::Error);

	virtual void createOrientationConstraint
	(const char* constraintName, const char* joint1Name,
	 const char* joint2Name, const Double* p, const hpp::boolSeq& mask)
	  throw (hpp::Error);

        void createRelativeComConstraint (const char* constraintName,
            const char* comn, const char* jointName, const floatSeq& point,
            const hpp::boolSeq& mask)
          throw (hpp::Error);

        void createComBeetweenFeet (const char* constraintName, const char* comn,
            const char* jointLName, const char* jointRName,
            const floatSeq& pointL, const floatSeq& pointR,
            const char* jointRefName, const hpp::boolSeq& mask)
          throw (hpp::Error);

	virtual void createStaticStabilityGravityConstraint
	(const char* constraintName, const char* jointName,
         const hpp::floatSeqSeq& points, const hpp::intSeqSeq& objTriangles,
         const hpp::intSeqSeq& floorTriangles)
	  throw (hpp::Error);

	virtual void createPositionConstraint (const char* constraintName,
					       const char* joint1Name,
					       const char* joint2Name,
					       const hpp::floatSeq& point1,
					       const hpp::floatSeq& point2,
					       const hpp::boolSeq& mask)
	  throw (hpp::Error);

	virtual bool applyConstraints (const hpp::floatSeq& input,
				       hpp::floatSeq_out output,
				       Double& residualError)
	  throw (hpp::Error);

	virtual bool generateValidConfig (UShort maxIter,
				       hpp::floatSeq_out output,
				       Double& residualError)
	  throw (hpp::Error);

        virtual void addPassiveDofs (const char* constraintName,
                                    const hpp::Names_t& dofName)
          throw (hpp::Error);

        virtual void setConstantRightHandSide (const char* constraintName,
					       CORBA::Boolean constant)
          throw (hpp::Error);

        virtual bool getConstantRightHandSide (const char* constraintName)
          throw (hpp::Error);

	virtual void resetConstraints () throw (hpp::Error);
	virtual void setNumericalConstraints
	(const char* constraintName, const hpp::Names_t& constraintNames)
	  throw (Error);
	virtual void lockJoint (const char* jointName,
				const hpp::floatSeq& value)
	  throw (hpp::Error);
	virtual void setErrorThreshold (Double threshold) throw (Error);
	virtual void setMaxIterations (UShort iterations) throw (Error);

	virtual void selectPathOptimizer (const char* pathOptimizerType)
	  throw (Error);

	virtual void selectPathValidation (const char* pathValidationType,
					   Double tolerance) throw (Error);

        virtual void selectPathProjector (const char* pathProjectorType,
                                          Double tolerance) throw (Error);

	virtual void selectPathPlanner (const char* pathPlannerType)
	  throw (Error);

	virtual bool prepareSolveStepByStep () throw (hpp::Error);
	virtual bool executeOneStep () throw (hpp::Error);
	virtual void finishSolveStepByStep () throw (hpp::Error);

	virtual void solve () throw (hpp::Error);

	virtual void directPath (const hpp::floatSeq& startConfig,
				  const hpp::floatSeq& endConfig)
	  throw (hpp::Error);

	virtual void interruptPathPlanning () throw (hpp::Error);

	virtual Short numberPaths () throw (hpp::Error);

	virtual void optimizePath (UShort pathId) throw (hpp::Error);

	virtual Double pathLength (UShort pathId) throw (hpp::Error);

	virtual hpp::floatSeq* configAtParam (UShort pathId,
					      Double atDistance)
	  throw (hpp::Error);

	virtual hpp::floatSeqSeq* getWaypoints (UShort inPathId)
	  throw (hpp::Error);
	virtual hpp::floatSeqSeq* nodes () throw (hpp::Error);
	virtual Long numberEdges () throw (hpp::Error);
	virtual void edge (ULong edgeId, hpp::floatSeq_out q1,
			   hpp::floatSeq_out q2) throw (hpp::Error);
	virtual Long numberConnectedComponents () throw (hpp::Error);
	virtual hpp::floatSeqSeq*
	nodesConnectedComponent (ULong connectedComponentId) throw (hpp::Error);
	virtual void clearRoadmap () throw (hpp::Error);
	virtual void resetRoadmap ();

      private:
	/// \brief Pointer to the Server owning this object
	corbaServer::Server* server_;
	/// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
	/// Instantiated at construction.
	core::ProblemSolverPtr_t problemSolver_;
      };
    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_OBSTACLE_IMPL_HH
