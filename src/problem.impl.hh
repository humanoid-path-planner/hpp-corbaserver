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
# include <stdlib.h>

# include "hpp/corbaserver/fwd.hh"
# include "hpp/corbaserver/problem.hh"
# include "hpp/corbaserver/problem-solver-map.hh"

# include "hpp/corbaserver/deprecated.hh"

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

        virtual void shutdown ();

        virtual Names_t* getAvailable (const char* what) throw (hpp::Error);

        virtual Names_t* getSelected (const char* what) throw (hpp::Error);

        virtual void setParameter (const char* name, const CORBA::Any& value)
          throw (Error);

        virtual CORBA::Any* getParameter (const char* name) throw (Error);

        virtual char* getParameterDoc (const char* name) throw (Error);

        virtual bool selectProblem (const char* problemName) throw (hpp::Error);

        virtual void resetProblem () throw (hpp::Error);

        virtual void movePathToProblem (ULong pathId, const char* problemName,
            const Names_t& jointNames) throw (hpp::Error);

	virtual void
	setRandomSeed (const Long seed) throw (hpp::Error) {
          srand ((int) seed);
        }

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

	virtual void createTransformationConstraint
	(const char* constraintName, const char* joint1Name,
	 const char* joint2Name, const Transform_ p, const hpp::boolSeq& mask)
	  throw (hpp::Error);

	virtual void createTransformationConstraint2
	(const char* constraintName, const char* joint1Name,
	 const char* joint2Name, const Transform_ frame1,
         const Transform_ frame2, const hpp::boolSeq& mask)
	  throw (hpp::Error);

        virtual void createLockedJoint (const char* lockedJointName,
                                        const char* jointName,
                                        const hpp::floatSeq& value)
          throw (hpp::Error);

        virtual void createLockedExtraDof (const char* lockedDofName,
                                           ULong index,
                                           const hpp::floatSeq& value)
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

	virtual void createConvexShapeContactConstraint
        (const char* constraintName, const Names_t& floorJoints,
         const Names_t& objectJoints,
         const hpp::floatSeqSeq& points, const hpp::intSeqSeq& objTriangles,
         const hpp::intSeqSeq& floorTriangles)
	  throw (hpp::Error);

        void createStaticStabilityConstraint (
            const char* constraintName, const hpp::Names_t& jointNames,
            const hpp::floatSeqSeq& points, const hpp::floatSeqSeq& normals,
            const char* comRootJointName)
          throw (hpp::Error);

	virtual void createPositionConstraint (const char* constraintName,
					       const char* joint1Name,
					       const char* joint2Name,
					       const hpp::floatSeq& point1,
					       const hpp::floatSeq& point2,
					       const hpp::boolSeq& mask)
	  throw (hpp::Error);

	virtual void createConfigurationConstraint (const char* constraintName,
                                               const hpp::floatSeq& goal,
					       const hpp::floatSeq& weights)
	  throw (hpp::Error);

	virtual void createDistanceBetweenJointConstraint
	(const char* constraintName, const char* joint1Name,
	 const char* joint2Name, Double distance) throw (Error);

	virtual void createDistanceBetweenJointAndObjects
	(const char* constraintName, const char* joint1Name,
	 const hpp::Names_t& objects, Double distance) throw (Error);

        virtual void createIdentityConstraint
        (const char* constraintName, const Names_t& inJoints,
         const hpp::Names_t& outJoints) throw (Error);

	virtual bool applyConstraints (const hpp::floatSeq& input,
				       hpp::floatSeq_out output,
				       Double& residualError)
	  throw (hpp::Error);

	virtual bool optimize (const hpp::floatSeq& input,
                               hpp::floatSeq_out output,
                               hpp::floatSeq_out residualError)
	  throw (hpp::Error);

	virtual void computeValueAndJacobian
	(const hpp::floatSeq& config, hpp::floatSeq_out value,
	 hpp::floatSeqSeq_out jacobian) throw (hpp::Error);

	virtual bool generateValidConfig (ULong maxIter,
				       hpp::floatSeq_out output,
				       Double& residualError)
	  throw (hpp::Error);

        virtual void addPassiveDofs (const char* constraintName,
                                    const hpp::Names_t& dofName)
          throw (hpp::Error);

        virtual void getConstraintDimensions (const char* constraintName,
            ULong& inputSize , ULong& inputDerivativeSize,
            ULong& outputSize, ULong& outputDerivativeSize)
          throw (hpp::Error);

        virtual void setConstantRightHandSide (const char* constraintName,
					       CORBA::Boolean constant)
          throw (hpp::Error);

        virtual bool getConstantRightHandSide (const char* constraintName)
          throw (hpp::Error);

        virtual floatSeq* getRightHandSide ()
          throw (hpp::Error);

        virtual void setRightHandSide (const hpp::floatSeq& rhs)
          throw (hpp::Error);

        virtual void setRightHandSideFromConfig (const hpp::floatSeq& config)
          throw (hpp::Error);

        virtual void setRightHandSideByName (const char* constraintName,
                                             const hpp::floatSeq& rhs)
          throw (hpp::Error);

        virtual void setRightHandSideFromConfigByName (const char* constraintName,
                                                       const hpp::floatSeq& config)
          throw (hpp::Error);

	virtual void resetConstraints () throw (hpp::Error);
	virtual void addNumericalConstraints
	(const char* constraintName, const hpp::Names_t& constraintNames,
         const hpp::intSeq& priorities)
	  throw (Error);
	virtual void setNumericalConstraintsLastPriorityOptional
          (const bool optional)
	  throw (Error);

        virtual void addLockedJointConstraints
        (const char* configProjName,const hpp::Names_t& lockedJointNames)
          throw (Error);

        virtual char* displayConstraints ()
          throw (Error);

        virtual void filterCollisionPairs () throw (hpp::Error);
	virtual Double getErrorThreshold () throw (Error);
	virtual void setErrorThreshold (Double threshold) throw (Error);
        virtual void setDefaultLineSearchType (const char* type) throw (Error);
	virtual ULong getMaxIterProjection () throw (Error);
	virtual void setMaxIterProjection (ULong iterations) throw (Error);
	virtual ULong getMaxIterPathPlanning () throw (Error);
	virtual void setMaxIterPathPlanning (ULong iterations) throw (Error);

	virtual void addPathOptimizer (const char* pathOptimizerType)
	  throw (Error);

	virtual void clearPathOptimizers () throw (Error);

	virtual void addConfigValidation (const char* configValidationType)
	  throw (Error);

	virtual void clearConfigValidations () throw (Error);

	virtual void selectPathValidation (const char* pathValidationType,
					   Double tolerance) throw (Error);

        virtual void selectPathProjector (const char* pathProjectorType,
                                          Double tolerance) throw (Error);

	virtual void selectPathPlanner (const char* pathPlannerType)
	  throw (Error);

    virtual void selectDistance (const char* distanceType)
      throw (Error);

    virtual void selectSteeringMethod (const char* steeringMethodType)
      throw (Error);

    virtual void selectConfigurationShooter (const char* configurationShooterType)
      throw (Error);

	virtual bool prepareSolveStepByStep () throw (hpp::Error);
	virtual bool executeOneStep () throw (hpp::Error);
	virtual void finishSolveStepByStep () throw (hpp::Error);

	virtual hpp::intSeq* solve () throw (hpp::Error);

	virtual bool directPath (const hpp::floatSeq& startConfig,
				 const hpp::floatSeq& endConfig,
				 CORBA::Boolean validate,
				 ULong& pathId,
				 CORBA::String_out report)
	  throw (hpp::Error);

	virtual void addConfigToRoadmap (const hpp::floatSeq& config) throw (hpp::Error);

	virtual void addEdgeToRoadmap (const hpp::floatSeq& config1,
				       const hpp::floatSeq& config2,
				       ULong pathId, bool bothEdges)
	  throw (hpp::Error);

	virtual void appendDirectPath (ULong pathId,
				       const hpp::floatSeq& config)
	  throw (hpp::Error);

        virtual void concatenatePath (ULong startId, ULong endId)
          throw (hpp::Error);

        virtual void extractPath (ULong pathId, Double start, Double end)
        throw (hpp::Error);

        virtual void erasePath (ULong pathId)
          throw (hpp::Error);

	virtual bool projectPath (ULong pathId)
	  throw (hpp::Error);

	virtual void interruptPathPlanning () throw (hpp::Error);

	virtual Long numberPaths () throw (hpp::Error);

	virtual hpp::intSeq* optimizePath (ULong pathId) throw (hpp::Error);

	virtual Double pathLength (ULong pathId) throw (hpp::Error);

	virtual hpp::floatSeq* configAtParam (ULong pathId,
					      Double atDistance)
	  throw (hpp::Error);

	virtual hpp::floatSeq* derivativeAtParam (ULong pathId,
						  ULong order,
						  Double atDistance)
	  throw (hpp::Error);

	virtual hpp::floatSeqSeq* getWaypoints (ULong inPathId, floatSeq_out times)
	  throw (hpp::Error);
	virtual hpp::floatSeqSeq* nodes () throw (hpp::Error);
	virtual Long numberEdges () throw (hpp::Error);
	virtual void edge (ULong edgeId, hpp::floatSeq_out q1,
			   hpp::floatSeq_out q2) throw (hpp::Error);
    virtual Long connectedComponentOfEdge (ULong edgeId) throw (hpp::Error);
    virtual hpp::floatSeq* node (ULong nodeId) throw (hpp::Error);
    virtual Long connectedComponentOfNode (ULong nodeId) throw (hpp::Error);
    virtual Long numberNodes () throw (hpp::Error);
	virtual Long numberConnectedComponents () throw (hpp::Error);
	virtual hpp::floatSeqSeq*
	nodesConnectedComponent (ULong connectedComponentId) throw (hpp::Error);
	
	virtual hpp::floatSeq*
	getNearestConfig (const hpp::floatSeq& config, const Long connectedComponentId, 
			  hpp::core::value_type& distance) throw (hpp::Error);
	
	virtual void clearRoadmap () throw (hpp::Error);
	virtual void resetRoadmap ();
        virtual void saveRoadmap (const char* filename) throw (hpp::Error);
        virtual void readRoadmap (const char* filename) throw (hpp::Error);

        virtual void scCreateScalarMultiply (const char* outName, Double scalar, const char* inName) throw (hpp::Error);

      private:
        /// Return the selected problem solver
        core::ProblemSolverPtr_t problemSolver ();
	/// \brief Pointer to the Server owning this object
	corbaServer::Server* server_;
      };
    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_OBSTACLE_IMPL_HH
