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
# include "hpp/corbaserver/problem-idl.hh"
# include "hpp/corbaserver/problem-solver-map.hh"

# include "hpp/core_idl/distances-idl.hh"
# include "hpp/core_idl/_problem-idl.hh"

# include "hpp/corbaserver/deprecated.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      using CORBA::Any;
      using CORBA::Boolean;
      using CORBA::Long;
      using CORBA::ULong;
      using CORBA::UShort;

      /// \brief Implement CORBA interface ``Problem''.
      class Problem : public virtual POA_hpp::corbaserver::Problem
      {
      public:
	Problem ();

        void setServer (ServerPlugin* server)
        {
          server_ = server;
        }

        virtual Names_t* getAvailable (const char* what);

        virtual Names_t* getSelected (const char* what);

        virtual void setParameter (const char* name, const Any& value);

        virtual Any* getParameter (const char* name);

        virtual char* getParameterDoc (const char* name);

        virtual bool selectProblem (const char* problemName);

        virtual void resetProblem ();

        virtual bool loadPlugin (const char* pluginName);

        virtual void movePathToProblem (ULong pathId, const char* problemName,
            const Names_t& jointNames);

        virtual void setMaxNumThreads (UShort n);

        virtual UShort getMaxNumThreads ();

	virtual void
	setRandomSeed (const Long seed) {
          srand ((int) seed);
        }

	virtual void
	setInitialConfig (const hpp::floatSeq& dofArray);

	virtual hpp::floatSeq*
	getInitialConfig ();

	virtual void
	addGoalConfig (const hpp::floatSeq& dofArray);

	virtual hpp::floatSeqSeq*	getGoalConfigs ();

	virtual void
	resetGoalConfigs () ;

	virtual void createOrientationConstraint
	(const char* constraintName, const char* joint1Name,
	 const char* joint2Name, const Double* p, const hpp::boolSeq& mask);

	virtual void createTransformationConstraint
	(const char* constraintName, const char* joint1Name,
	 const char* joint2Name, const Transform_ p, const hpp::boolSeq& mask);

	virtual void createTransformationConstraint2
	(const char* constraintName, const char* joint1Name,
	 const char* joint2Name, const Transform_ frame1,
         const Transform_ frame2, const hpp::boolSeq& mask);

	virtual void createTransformationSE3Constraint
	(const char* constraintName, const char* joint1Name,
	 const char* joint2Name, const Transform_ frame1,
         const Transform_ frame2, const hpp::boolSeq& mask);

        virtual void createLockedJoint (const char* lockedJointName,
                                        const char* jointName,
                                        const hpp::floatSeq& value);

        virtual void createLockedJointWithComp
        (const char* lockedJointName, const char* jointName,
         const hpp::floatSeq& value, const hpp::ComparisonTypes_t& comp);

        virtual void createLockedExtraDof (const char* lockedDofName,
                                           ULong index,
                                           const hpp::floatSeq& value);

        virtual void createManipulability (const char* name, const char* function);

        void createRelativeComConstraint (const char* constraintName,
            const char* comn, const char* jointName, const floatSeq& point,
            const hpp::boolSeq& mask);

        void createComBeetweenFeet (const char* constraintName, const char* comn,
            const char* jointLName, const char* jointRName,
            const floatSeq& pointL, const floatSeq& pointR,
            const char* jointRefName, const floatSeq& pRef,
            const hpp::boolSeq& mask);

	virtual void createConvexShapeContactConstraint
        (const char* constraintName, const Names_t& floorJoints,
         const Names_t& objectJoints,
         const hpp::floatSeqSeq& points, const hpp::intSeqSeq& objTriangles,
         const hpp::intSeqSeq& floorTriangles);

        void createStaticStabilityConstraint (
            const char* constraintName, const hpp::Names_t& jointNames,
            const hpp::floatSeqSeq& points, const hpp::floatSeqSeq& normals,
            const char* comRootJointName);

	virtual void createPositionConstraint (const char* constraintName,
					       const char* joint1Name,
					       const char* joint2Name,
					       const hpp::floatSeq& point1,
					       const hpp::floatSeq& point2,
					       const hpp::boolSeq& mask);

	virtual void createConfigurationConstraint (const char* constraintName,
                                               const hpp::floatSeq& goal,
					       const hpp::floatSeq& weights);

	virtual void createDistanceBetweenJointConstraint
	(const char* constraintName, const char* joint1Name,
	 const char* joint2Name, Double distance);

	virtual void createDistanceBetweenJointAndObjects
	(const char* constraintName, const char* joint1Name,
	 const hpp::Names_t& objects, Double distance);

        virtual void createIdentityConstraint
        (const char* constraintName, const Names_t& inJoints,
         const hpp::Names_t& outJoints);

	virtual bool applyConstraints (const hpp::floatSeq& input,
				       hpp::floatSeq_out output,
				       Double& residualError);

	virtual bool optimize (const hpp::floatSeq& input,
                               hpp::floatSeq_out output,
                               hpp::floatSeq_out residualError);

	virtual void computeValueAndJacobian
	(const hpp::floatSeq& config, hpp::floatSeq_out value,
	 hpp::floatSeqSeq_out jacobian);

	virtual bool generateValidConfig (ULong maxIter,
				       hpp::floatSeq_out output,
				       Double& residualError);

        virtual void addPassiveDofs (const char* constraintName,
                                    const hpp::Names_t& dofName);

        virtual void getConstraintDimensions (const char* constraintName,
            ULong& inputSize , ULong& inputDerivativeSize,
            ULong& outputSize, ULong& outputDerivativeSize);

        virtual void setConstantRightHandSide (const char* constraintName,
					       CORBA::Boolean constant);

        virtual bool getConstantRightHandSide (const char* constraintName);

        virtual floatSeq* getRightHandSide ();

        virtual void setRightHandSide (const hpp::floatSeq& rhs);

        virtual void setRightHandSideFromConfig (const hpp::floatSeq& config);

        virtual void setRightHandSideByName (const char* constraintName,
                                             const hpp::floatSeq& rhs);

        virtual void setRightHandSideFromConfigByName (const char* constraintName,
                                                       const hpp::floatSeq& config);

	virtual void resetConstraints ();
	virtual void resetConstraintMap ();
	virtual void addNumericalConstraints
	(const char* constraintName, const hpp::Names_t& constraintNames,
         const hpp::intSeq& priorities);
	virtual void setNumericalConstraintsLastPriorityOptional
          (const bool optional);

        virtual void addLockedJointConstraints
        (const char* configProjName,const hpp::Names_t& lockedJointNames);

        virtual char* displayConstraints ();

        virtual void filterCollisionPairs ();
	virtual Double getErrorThreshold ();
	virtual void setErrorThreshold (Double threshold);
        virtual void setDefaultLineSearchType (const char* type);
	virtual ULong getMaxIterProjection ();
	virtual void setMaxIterProjection (ULong iterations);
	virtual ULong getMaxIterPathPlanning ();
	virtual void setMaxIterPathPlanning (ULong iterations);
    virtual void setTimeOutPathPlanning (double timeOut);
    virtual double getTimeOutPathPlanning ();


	virtual void addPathOptimizer (const char* pathOptimizerType);

	virtual void clearPathOptimizers ();

	virtual void addConfigValidation (const char* configValidationType);

	virtual void clearConfigValidations ();

	virtual void selectPathValidation (const char* pathValidationType,
					   Double tolerance);

        virtual void selectPathProjector (const char* pathProjectorType,
                                          Double tolerance);

	virtual void selectPathPlanner (const char* pathPlannerType);

    virtual void selectDistance (const char* distanceType);

    virtual void selectSteeringMethod (const char* steeringMethodType);

    virtual void selectConfigurationShooter (const char* configurationShooterType);

	virtual bool prepareSolveStepByStep ();
	virtual bool executeOneStep ();
	virtual void finishSolveStepByStep ();

	virtual hpp::intSeq* solve ();

	virtual bool directPath (const hpp::floatSeq& startConfig,
				 const hpp::floatSeq& endConfig,
				 CORBA::Boolean validate,
				 ULong& pathId,
				 CORBA::String_out report);

        virtual bool reversePath(ULong pathId, ULong& reversedPathId);

	virtual void addConfigToRoadmap (const hpp::floatSeq& config);

	virtual void addEdgeToRoadmap (const hpp::floatSeq& config1,
				       const hpp::floatSeq& config2,
				       ULong pathId, bool bothEdges);

	virtual void appendDirectPath (ULong pathId,
				       const hpp::floatSeq& config,
                                       Boolean validate);

        virtual void concatenatePath (ULong startId, ULong endId);

        virtual void extractPath (ULong pathId, Double start, Double end);

        virtual void erasePath (ULong pathId);

	virtual bool projectPath (ULong pathId);

	virtual void interruptPathPlanning ();

	virtual Long numberPaths ();

	virtual hpp::intSeq* optimizePath (ULong pathId);

	virtual Double pathLength (ULong pathId);

	virtual hpp::floatSeq* configAtParam (ULong pathId,
					      Double atDistance);

	virtual hpp::floatSeq* derivativeAtParam (ULong pathId,
						  ULong order,
						  Double atDistance);

	virtual hpp::floatSeqSeq* getWaypoints (ULong inPathId, floatSeq_out times);
	virtual hpp::floatSeqSeq* nodes ();
	virtual Long numberEdges ();
	virtual void edge (ULong edgeId, hpp::floatSeq_out q1,
			   hpp::floatSeq_out q2);
    virtual Long connectedComponentOfEdge (ULong edgeId);
    virtual hpp::floatSeq* node (ULong nodeId);
    virtual Long connectedComponentOfNode (ULong nodeId);
    virtual Long numberNodes ();
	virtual Long numberConnectedComponents ();
	virtual hpp::floatSeqSeq*
	nodesConnectedComponent (ULong connectedComponentId);
	
	virtual hpp::floatSeq*
	getNearestConfig (const hpp::floatSeq& config, const Long connectedComponentId, 
			  hpp::core::value_type& distance);
	
	virtual void clearRoadmap ();
	virtual void resetRoadmap ();
        virtual void saveRoadmap (const char* filename);
        virtual void readRoadmap (const char* filename);

        virtual void scCreateScalarMultiply (const char* outName, Double scalar, const char* inName);

        hpp::core_idl::Distance_ptr getDistance ();

        void setDistance (hpp::core_idl::Distance_ptr distance);

        hpp::core_idl::Path_ptr getPath (ULong pathId);

        ULong addPath (hpp::core_idl::PathVector_ptr _path);

        hpp::core_idl::SteeringMethod_ptr getSteeringMethod ();

        hpp::core_idl::PathValidation_ptr getPathValidation ();

        hpp::core_idl::PathPlanner_ptr getPathPlanner ();

        hpp::core_idl::Problem_ptr getProblem ();

        hpp::constraints_idl::Implicit_ptr getConstraint (const char* name);

        void setRobot (hpp::pinocchio_idl::Device_ptr robot);

        pinocchio_idl::CollisionObject_ptr getObstacle(const char* name);

        core_idl::Problem_ptr createProblem (pinocchio_idl::Device_ptr robot);

        core_idl::Roadmap_ptr createRoadmap(core_idl::Distance_ptr distance, pinocchio_idl::Device_ptr robot);
        core_idl::PathPlanner_ptr createPathPlanner (const char* type, core_idl::Problem_ptr _problem, core_idl::Roadmap_ptr roadmap);
        core_idl::PathOptimizer_ptr createPathOptimizer (const char* type, core_idl::Problem_ptr _problem);

        core_idl::PathValidation_ptr createPathValidation (const char* type, pinocchio_idl::Device_ptr robot, value_type parameter);
        core_idl::ConfigurationShooter_ptr createConfigurationShooter (const char* type, core_idl::Problem_ptr _problem);
        core_idl::Distance_ptr createDistance (const char* type, core_idl::Problem_ptr _problem);
        core_idl::SteeringMethod_ptr createSteeringMethod (const char* type, core_idl::Problem_ptr _problem);

      private:
        /// Return the selected problem solver
        core::ProblemSolverPtr_t problemSolver ();
        /// \brief Pointer to the ServerPlugin owning this object.
	ServerPlugin* server_;
      };
    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_OBSTACLE_IMPL_HH
