// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <iostream>
#include <ctime>

#include <hpp/util/debug.hh>
#include <hpp/util/portability.hh>

#include <hpp/fcl/shape/geometric_shapes.h>

#include <hpp/model/configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/locked-joint.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/comparison-type.hh>

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/position.hh>
#include <hpp/constraints/orientation.hh>
#include <hpp/constraints/position.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/com-between-feet.hh>
#include <hpp/constraints/relative-orientation.hh>
#include <hpp/constraints/relative-position.hh>
#include <hpp/constraints/static-stability.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/model/body.hh>

#include "problem.impl.hh"

using hpp::constraints::Orientation;
using hpp::constraints::OrientationPtr_t;
using hpp::constraints::Position;
using hpp::constraints::PositionPtr_t;
using hpp::constraints::RelativeOrientation;
using hpp::constraints::ComBetweenFeetPtr_t;
using hpp::constraints::ComBetweenFeet;
using hpp::constraints::RelativeComPtr_t;
using hpp::constraints::RelativeCom;
using hpp::constraints::RelativeOrientationPtr_t;
using hpp::constraints::RelativePosition;
using hpp::constraints::RelativePositionPtr_t;
using hpp::constraints::StaticStabilityGravity;
using hpp::constraints::StaticStabilityGravityPtr_t;

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      static ConfigurationPtr_t floatSeqToConfig
      (hpp::core::ProblemSolverPtr_t problemSolver,
       const hpp::floatSeq& dofArray)
      {
	size_type configDim = (size_type)dofArray.length();
	ConfigurationPtr_t config (new Configuration_t (configDim));

	// Get robot in hppPlanner object.
	DevicePtr_t robot = problemSolver->robot ();

	// Compare size of input array with number of degrees of freedom of
	// robot.
	if (configDim != robot->configSize ()) {
	  hppDout (error, "robot nb dof=" << configDim <<
		   " is different from config size=" << robot->configSize());
	  throw std::runtime_error
	    ("robot nb dof is different from config size");
	}

	// Fill dof vector with dof array.
	for (size_type iDof=0; iDof < configDim; ++iDof) {
	  (*config) [iDof] = dofArray [iDof];
	}
	return config;
      }

      static vector3_t floatSeqToVector3 (const hpp::floatSeq& dofArray)
      {
	if (dofArray.length() != 3) {
	  std::ostringstream oss
	    ("Expecting vector of size 3, got vector of size");
	  oss << dofArray.length() << ".";
	  throw hpp::Error (oss.str ().c_str ());
	}
	// Fill dof vector with dof array.
	vector3_t result;
	for (unsigned int iDof=0; iDof < 3; ++iDof) {
	  result [iDof] = dofArray [iDof];
	}
	return result;
      }

      static vector_t floatSeqToVector (const hpp::floatSeq& dofArray)
      {
	// Fill dof vector with dof array.
	vector_t result (dofArray.length ());
	for (size_type iDof=0; iDof < result.size (); ++iDof) {
	  result [iDof] = dofArray [iDof];
	}
	return result;
      }

      Problem::Problem (corbaServer::Server* server)
	: server_ (server),
	  problemSolver_ (server->problemSolver ())
      {}

      // ---------------------------------------------------------------

      void Problem::setInitialConfig (const hpp::floatSeq& dofArray)
	throw (hpp::Error)
      {
	try {
	  ConfigurationPtr_t config = floatSeqToConfig (problemSolver_,
							dofArray);
	  problemSolver_->initConfig (config);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::getInitialConfig()
	throw(hpp::Error)
      {
	hpp::floatSeq *dofArray;

	// Get robot in hppPlanner object.
	ConfigurationPtr_t config = problemSolver_->initConfig ();

	if (config) {
	  std::size_t deviceDim = config->size();

	  dofArray = new hpp::floatSeq();
	  dofArray->length(deviceDim);

	  for(unsigned int i=0; i<deviceDim; i++){
	    (*dofArray)[i] = (*config) [i];
	  }
	  return dofArray;
	}
	else {
	  throw hpp::Error ("no initial configuration defined");
	}
      }

      // ---------------------------------------------------------------

      void Problem::addGoalConfig (const hpp::floatSeq& dofArray)
	throw (hpp::Error)
      {
	try {
	  ConfigurationPtr_t config = floatSeqToConfig (problemSolver_,
							dofArray);
	  problemSolver_->addGoalConfig (config);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::floatSeqSeq* Problem::getGoalConfigs () throw(hpp::Error)
      {
	try {
	  hpp::floatSeqSeq *configSequence;
	  const core::Configurations_t goalConfigs
	    (problemSolver_->goalConfigs ());
	  std::size_t nbGoalConfig = goalConfigs.size ();
	  configSequence = new hpp::floatSeqSeq ();
	  configSequence->length (nbGoalConfig);
	  for (std::size_t i=0; i<nbGoalConfig ;++i) {
	    const ConfigurationPtr_t& config = goalConfigs [i];
	    std::size_t deviceDim = config->size ();

	    hpp::floatSeq dofArray;
	    dofArray.length (deviceDim);

	    for (std::size_t j=0; j<deviceDim; ++j)
	      dofArray[j] = (*config) [j];
	    (*configSequence) [i] = dofArray;
	  }
	  return configSequence;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::resetGoalConfigs () throw (hpp::Error)
      {
	problemSolver_->resetGoalConfigs ();
      }

      std::vector<bool> boolSeqToBoolVector (const hpp::boolSeq& mask)
      {
        if (mask.length () != 3)
	  throw hpp::Error ("Mask must be of length 3");
        std::vector<bool> m (3);
	for (size_t i=0; i<3; i++)
	  m[i] = mask[i];
	return m;
      }

      // ---------------------------------------------------------------

      void Problem::createOrientationConstraint
      (const char* constraintName, const char* joint1Name,
       const char* joint2Name, const Double* p, const hpp::boolSeq& mask)
	throw (hpp::Error)
      {
	JointPtr_t joint1;
	JointPtr_t joint2;
	size_type constrainedJoint = 0;
	fcl::Quaternion3f quat (p [0], p [1], p [2], p [3]);
	hpp::model::matrix3_t rotation;
	quat.toRotation (rotation);

	std::vector<bool> m = boolSeqToBoolVector (mask);
	try {
	  // Test whether joint1 is world frame
	  if (std::string (joint1Name) == std::string ("")) {
	    constrainedJoint = 2;
	  } else {
	    joint1 =
	      problemSolver_->robot()->getJointByName(joint1Name);
	  }
	  // Test whether joint2 is world frame
	  if (std::string (joint2Name) == std::string ("")) {
	    if (constrainedJoint == 2) {
	      throw hpp::Error ("At least one joint should be provided.");
	    }
	    constrainedJoint = 1;
	  } else {
	    joint2 =
	      problemSolver_->robot()->getJointByName(joint2Name);
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
  std::string name (constraintName);
	if (constrainedJoint == 0) {
	  // Both joints are provided
	  problemSolver_->addNumericalConstraint
	    (name, RelativeOrientation::create
	      (name, problemSolver_->robot(), joint1, joint2, rotation, m));
	} else {
	  JointPtr_t joint = constrainedJoint == 1 ? joint1 : joint2;
	  problemSolver_->addNumericalConstraint
	    (name, Orientation::create
	      (name, problemSolver_->robot(), joint, rotation, m));
	}
      }

      // ---------------------------------------------------------------

      void Problem::createRelativeComConstraint (const char* constraintName,
          const char* comName, const char* jointName, const floatSeq& p,
          const hpp::boolSeq& mask)
        throw (hpp::Error)
      {
	JointPtr_t joint;
        model::CenterOfMassComputationPtr_t comc;
	vector3_t point = floatSeqToVector3 (p);

	std::vector<bool> m = boolSeqToBoolVector (mask);
	try {
          joint = problemSolver_->robot()->getJointByName(jointName);
	  // Test whether joint1 is world frame
          std::string name (constraintName), comN (comName);
          if (comN.compare ("") == 0) {
            problemSolver_->addNumericalConstraint
              (name, RelativeCom::create (problemSolver_->robot(),
                                             joint, point, m));
          } else {
            comc = problemSolver_->centerOfMassComputation (comN);
            if (!comc)
              throw hpp::Error ("Partial COM not found.");
            problemSolver_->addNumericalConstraint
              (name, RelativeCom::create (problemSolver_->robot(), comc,
                                             joint, point, m));
          }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::createComBeetweenFeet
      (const char* constraintName, const char* comName, const char* jointLName,
       const char* jointRName, const floatSeq& pL, const floatSeq& pR,
       const char* jointRefName, const hpp::boolSeq& mask)
	throw (hpp::Error)
      {
	JointPtr_t jointL, jointR, jointRef;
        model::CenterOfMassComputationPtr_t comc;
	vector3_t pointL = floatSeqToVector3 (pL);
	vector3_t pointR = floatSeqToVector3 (pR);
	vector3_t pointRef (0,0,0);

	std::vector<bool> m = boolSeqToBoolVector (mask);
	try {
          jointL = problemSolver_->robot()->getJointByName(jointLName);
          jointR = problemSolver_->robot()->getJointByName(jointRName);
	  // Test whether joint1 is world frame
          if (std::string (jointRefName) == std::string (""))
            jointRef = problemSolver_->robot()->rootJoint ();
	  else
	    jointRef = problemSolver_->robot()->getJointByName(jointRefName);
          std::string name (constraintName), comN (comName);
          if (comN.compare ("") == 0) {
            problemSolver_->addNumericalConstraint
              (name, ComBetweenFeet::create (name, problemSolver_->robot(),
                                             jointL, jointR, pointL, pointR,
                                             jointRef, pointRef, m));
          } else {
            comc = problemSolver_->centerOfMassComputation (comN);
            if (!comc)
              throw hpp::Error ("Partial COM not found.");
            problemSolver_->addNumericalConstraint
              (name, ComBetweenFeet::create (name, problemSolver_->robot(), comc,
                                             jointL, jointR, pointL, pointR,
                                             jointRef, pointRef, m));
          }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::createStaticStabilityGravityConstraint
      (const char* constraintName, const char* jointName,
       const hpp::floatSeqSeq& points, const hpp::intSeqSeq& objTriangles,
       const hpp::intSeqSeq& floorTriangles)
        throw (hpp::Error)
      {
        JointPtr_t joint;
        try {
          std::string name (constraintName);
          joint = problemSolver_->robot()->getJointByName(jointName);
          StaticStabilityGravityPtr_t f = StaticStabilityGravity::create
            (name, problemSolver_->robot(), joint);
          problemSolver_->addNumericalConstraint (name, f);
          std::vector <fcl::Vec3f> pts (points.length ());
          for (CORBA::ULong i = 0; i < points.length (); ++i) {
            if (points[i].length () != 3)
              throw hpp::Error ("Points must be of size 3.");
            pts [i] = fcl::Vec3f (points[i][0],points[i][1],points[i][2]);
          }
          for (CORBA::ULong i = 0; i < objTriangles.length (); ++i) {
            if (objTriangles[i].length () != 3)
              throw hpp::Error ("Triangle must have size 3.");
            for (size_t j = 0; j < 3; j++)
              if (objTriangles[i][j] < 0 && (size_t) objTriangles[i][j] >= pts.size())
                throw hpp::Error ("Point index out of range.");

            f->addObjectTriangle (fcl::TriangleP (
                  pts [objTriangles[i][0]],
                  pts [objTriangles[i][1]],
                  pts [objTriangles[i][2]]
                  ));
          }
          for (CORBA::ULong i = 0; i < floorTriangles.length (); ++i) {
            if (floorTriangles[i].length () != 3)
              throw hpp::Error ("Triangle must have size 3.");
            for (size_t j = 0; j < 3; j++)
              if (floorTriangles[i][j] < 0 && (size_t) floorTriangles[i][j] >= pts.size())
                throw hpp::Error ("Point index out of range.");

            f->addFloorTriangle (fcl::TriangleP (
                  pts [floorTriangles[i][0]],
                  pts [floorTriangles[i][1]],
                  pts [floorTriangles[i][2]]
                  ));
          }
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------


      void Problem::createPositionConstraint
      (const char* constraintName, const char* joint1Name,
       const char* joint2Name, const hpp::floatSeq& point1,
       const hpp::floatSeq& point2, const hpp::boolSeq& mask)
	throw (hpp::Error)
      {
	JointPtr_t joint1;
	JointPtr_t joint2;
	vector3_t targetInWorldFrame;
	vector3_t targetInLocalFrame;
	vector3_t p1 = floatSeqToVector3 (point1);
	vector3_t p2 = floatSeqToVector3 (point2);
	size_type constrainedJoint = 0;
	std::vector<bool> m = boolSeqToBoolVector (mask);
	try {
	  // Test whether joint1 is world frame
	  if (std::string (joint1Name) == std::string ("")) {
	    constrainedJoint = 2;
	    targetInWorldFrame = p1;
	    targetInLocalFrame = p2;
	  } else {
	    joint1 =
	      problemSolver_->robot()->getJointByName(joint1Name);
	  }
	  // Test whether joint2 is world frame
	  if (std::string (joint2Name) == std::string ("")) {
	    if (constrainedJoint == 2) {
	      throw hpp::Error ("At least one joint should be provided.");
	    }
	    constrainedJoint = 1;
	    targetInWorldFrame = p2;
	    targetInLocalFrame = p1;
	  } else {
	    joint2 =
	      problemSolver_->robot()->getJointByName(joint2Name);
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	std::string name (constraintName);
	if (constrainedJoint == 0) {
	  // Both joints are provided
	  problemSolver_->addNumericalConstraint
	    (name, RelativePosition::create
	     (name, problemSolver_->robot(), joint1, joint2, p1, p2, m));
	} else {
	  hpp::model::matrix3_t I3; I3.setIdentity ();
	  JointPtr_t joint = constrainedJoint == 1 ? joint1 : joint2;
	  problemSolver_->addNumericalConstraint
	    (name, Position::create
	     (name, problemSolver_->robot(), joint, targetInLocalFrame,
	      targetInWorldFrame, I3, m));
	}
      }

      // ---------------------------------------------------------------

      bool Problem::applyConstraints (const hpp::floatSeq& input,
				      hpp::floatSeq_out output,
				      double& residualError)
	throw (hpp::Error)
      {
	bool success = false;
	ConfigurationPtr_t config = floatSeqToConfig (problemSolver_, input);
	try {
	  success = problemSolver_->constraints ()->apply (*config);
	  if (hpp::core::ConfigProjectorPtr_t configProjector =
	      problemSolver_->constraints ()->configProjector ()) {
	    residualError = configProjector->residualError ();
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	ULong size = (ULong) config->size ();
	hpp::floatSeq* q_ptr = new hpp::floatSeq ();
	q_ptr->length (size);

	for (std::size_t i=0; i<size; ++i) {
	  (*q_ptr) [i] = (*config) [i];
	}
	output = q_ptr;
	return success;
      }

      // ---------------------------------------------------------------

      bool Problem::generateValidConfig (UShort maxIter,
				      hpp::floatSeq_out output,
				      double& residualError)
	throw (hpp::Error)
      {
        DevicePtr_t robot = problemSolver_->robot ();
        core::BasicConfigurationShooterPtr_t shooter
          = core::BasicConfigurationShooter::create (robot);
	bool success = false, configIsValid = false;
        ConfigurationPtr_t config;
        while (!configIsValid && maxIter > 0)
        {
          try {
            config = shooter->shoot ();
            success = problemSolver_->constraints ()->apply (*config);
            if (hpp::core::ConfigProjectorPtr_t configProjector =
                problemSolver_->constraints ()->configProjector ()) {
              residualError = configProjector->residualError ();
            }
            if (success) {
              robot->currentConfiguration (*config);
              configIsValid = !robot->collisionTest ();
            }
          } catch (const std::exception& exc) {
            throw hpp::Error (exc.what ());
          }
          maxIter--;
        }
	ULong size = (ULong) config->size ();
	hpp::floatSeq* q_ptr = new hpp::floatSeq ();
	q_ptr->length (size);

	for (std::size_t i=0; i<size; ++i) {
	  (*q_ptr) [i] = (*config) [i];
	}
	output = q_ptr;
	return configIsValid;
      }

      // ---------------------------------------------------------------

      void Problem::resetConstraints ()	throw (hpp::Error)
      {
	try {
	  problemSolver_->resetConstraints ();
	  problemSolver_->robot ()->controlComputation
	    (model::Device::JOINT_POSITION);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::addPassiveDofs (const char* passiveDofsName,
          const hpp::Names_t& dofNames)
        throw (hpp::Error)
      {
        DevicePtr_t robot = problemSolver_->robot ();
        if (!robot)
	    throw Error ("You should set the robot before defining"
			 " passive DOFs.");
        std::vector <size_type> dofs;
        /// First, translate names into velocity indexes.
        for (CORBA::ULong i=0; i<dofNames.length (); ++i) {
          std::string name (dofNames[i]);
          JointPtr_t j = robot->getJointByName (name);
          if (!j)
            throw hpp::Error ("One joint not found.");
          for (size_type i = 0; i < j->numberDof (); i++)
            dofs.push_back (j->rankInVelocity() + i);
        }
        /// Then, sort and remove non duplicated elements.
        std::sort (dofs.begin (), dofs.end ());
        std::vector <size_type>::iterator last =
          std::unique (dofs.begin (), dofs.end ());
        dofs.erase (last, dofs.end ());
        /// Then, create the intervals.
        core::SizeIntervals_t passiveDofs;
        dofs.push_back (robot->numberDof () + 1);
        size_type intStart = dofs[0], intEnd = dofs[0];
        for (size_t i = 1; i < dofs.size (); i++) {
          intEnd ++;
          if (intEnd == dofs[i]) {
            continue;
          } else {
            passiveDofs.push_back (
                core::SizeInterval_t (intStart, intEnd - intStart));
            intStart = intEnd = dofs[i];
          }
        }
        problemSolver_->addPassiveDofs (passiveDofsName, passiveDofs);
      }

      // ---------------------------------------------------------------

      void Problem::setConstantRightHandSide (const char* constraintName,
					      CORBA::Boolean constant)
	throw (hpp::Error)
      {
	try {
	  core::ComparisonTypePtr_t comparison =
	    problemSolver_->comparisonType (constraintName);
	  if (!comparison)
	    throw std::runtime_error
	      (std::string ("Numerical constraint ") + constraintName +
	       std::string ("can not be found."));
	  if (constant) {
	    problemSolver_->comparisonType (constraintName,
					   core::EqualToZero::create ());
	  } else {
	    problemSolver_->comparisonType (constraintName,
					   core::Equality::create ());
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      bool Problem::getConstantRightHandSide (const char* constraintName)
	throw (hpp::Error)
      {
	try {
	  core::ComparisonTypePtr_t comparison =
	    problemSolver_->comparisonType (constraintName);
	  if (!comparison) {
	    throw std::runtime_error
	      (std::string ("Numerical constraint ") + constraintName +
	       std::string ("can not be found."));
	  }
	  return comparison->constantRightHandSide ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::setNumericalConstraints
      (const char* constraintName, const Names_t& constraintNames)
	throw (Error)
      {
	try {
	  if (!problemSolver_->robot()) {
	    throw Error ("You should set the robot before defining"
			 " constraints.");
	  }
	  for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
	    std::string name (constraintNames [i]);
            problemSolver_->addFunctionToConfigProjector (constraintName, name);
	    problemSolver_->robot ()->controlComputation
	      (model::Device::ALL);
	  }
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::lockJoint (const char* jointName,
			       const hpp::floatSeq& value)
	throw (hpp::Error)
      {
	try {
	  // Get robot in hppPlanner object.
	  DevicePtr_t robot = problemSolver_->robot ();
	  JointPtr_t joint = robot->getJointByName (jointName);
	  vector_t jointConfig = floatSeqToVector (value);

	  LockedJointPtr_t lockedJoint (LockedJoint::create (joint, jointConfig));
	  problemSolver_->addLockedJoint (lockedJoint);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::setErrorThreshold (Double threshold) throw (Error)
      {
	problemSolver_->errorThreshold (threshold);
      }

      // ---------------------------------------------------------------
      void Problem::setMaxIterations (UShort iterations) throw (Error)
      {
	problemSolver_->maxIterations (iterations);
      }

      // ---------------------------------------------------------------

      void Problem::selectPathPlanner (const char* pathPlannerType)
	throw (Error)
      {
	try {
	  problemSolver_->pathPlannerType (std::string (pathPlannerType));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::addPathOptimizer (const char* pathOptimizerType)
	throw (Error)
      {
	try {
	  problemSolver_->addPathOptimizer (std::string (pathOptimizerType));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::clearPathOptimizers () throw (Error)
      {
	try {
	  problemSolver_->clearPathOptimizers ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::selectPathValidation (const char* pathValidationType,
					  Double tolerance) throw (Error)
      {
	try {
	  problemSolver_->pathValidationType (std::string (pathValidationType),
					      tolerance);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::selectPathProjector (const char* pathProjectorType,
                                         Double tolerance) throw (Error)
      {
        try {
          problemSolver_->pathProjectorType (std::string (pathProjectorType),
                                             tolerance);
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      bool Problem::prepareSolveStepByStep () throw (hpp::Error)
      {
	try {
	  return problemSolver_->prepareSolveStepByStep ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
        return false;
      }

      // ---------------------------------------------------------------

      bool Problem::executeOneStep () throw (hpp::Error)
      {
	try {
	  return problemSolver_->executeOneStep ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
        return false;
      }

      // ---------------------------------------------------------------

      void Problem::finishSolveStepByStep () throw (hpp::Error)
      {
	try {
	  problemSolver_->solve();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      CORBA::Double Problem::solve () throw (hpp::Error)
      {
	try {
          std::clock_t start = std::clock ();
	  problemSolver_->solve();
          return (std::clock () - start) / (double) CLOCKS_PER_SEC;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::directPath (const hpp::floatSeq& startConfig,
				const hpp::floatSeq& endConfig)
	throw (hpp::Error)
      {
	ConfigurationPtr_t start;
	ConfigurationPtr_t end;
	try {
	  start = floatSeqToConfig (problemSolver_, startConfig);
	  end = floatSeqToConfig (problemSolver_, endConfig);
	  if (!problemSolver_->problem ()) {
	    problemSolver_->resetProblem ();
	  }
	  SteeringMethodPtr_t sm = problemSolver_->problem ()->steeringMethod ();
	  PathPtr_t dp = (*sm) (*start, *end);
	  // Add Path in problem
	  PathVectorPtr_t path
	    (core::PathVector::create (dp->outputSize (),
				       dp->outputDerivativeSize ()));
	  path->appendPath (dp);
	  problemSolver_->addPath (path);
	  PathPtr_t unused;
	  problemSolver_->problem()->pathValidation ()->validate
	    (dp, false, unused);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::interruptPathPlanning() throw (hpp::Error)
      {
	problemSolver_->pathPlanner ()->interrupt ();
      }

      // ---------------------------------------------------------------

      Short Problem::numberPaths () throw (hpp::Error)
      {
	try {
	  return (Short) problemSolver_->paths ().size ();
	} catch (std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::optimizePath(UShort pathId) throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver_->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver_->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  PathVectorPtr_t initial = problemSolver_->paths () [pathId];
	  problemSolver_->optimizePath (initial);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      Double Problem::pathLength (UShort pathId) throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver_->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver_->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  return problemSolver_->paths () [pathId]->length ();
	} catch (std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::configAtParam (UShort pathId,
					     Double atDistance)
	throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver_->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver_->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  PathPtr_t path = problemSolver_->paths () [pathId];
	  Configuration_t config = (*path) (atDistance);
	  // Allocate result now that the size is known.
	  std::size_t size =  config.size ();
	  double* dofArray = hpp::floatSeq::allocbuf((ULong)size);
	  hpp::floatSeq* floatSeq = new hpp::floatSeq (size, size,
						       dofArray, true);
	  for (std::size_t i=0; i < size; ++i) {
	    dofArray[i] =  config [i];
	  }
	  return floatSeq;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      //---------------------------------

      // Get end point of path (start if points == 0, end if points == 1)
      void findExtremes (PathPtr_t path, hpp::floatSeqSeq& configSequence,
			 const std::size_t points)
      {
	const std::size_t size_increment = 1;
	Configuration_t config, config_1;
	std::vector<Configuration_t> configs;
	hpp::floatSeq dofArray;
	config = (*path) (0);
	configs.push_back(config);
	config_1 = (*path) (path->length());
	configs.push_back(config_1);
	dofArray.length (config.size());
	// modify configsequence
	configSequence.length (configSequence.length() + size_increment);
	std::size_t ptr = configSequence.length() - size_increment;
	for (std::size_t i=0; i<size_increment; ++i) {
	  for (std::size_t j=0; j < (config.size()); ++j) {
	    dofArray [j] = configs [points][j];
	  }
	  (configSequence)[ptr+i] = dofArray;
	}
      }

      //---------------------------------

      void findExtremities (PathVectorPtr_t path,
			    hpp::floatSeqSeq& configSequence)
      {
        std::size_t num_subpaths  = (*path).numberPaths ();
        if (num_subpaths == 1) {
	  findExtremes (path,configSequence,1);
	}
        else {
	  for (std::size_t i = 0; i < num_subpaths; ++i) {
	    PathPtr_t subpath = (*path).pathAtRank(i);
	    PathVectorPtr_t sp =
	      HPP_DYNAMIC_PTR_CAST (hpp::core::PathVector,subpath);
	    if (sp) {
	      findExtremities(sp, configSequence);
	    } else {
	      findExtremes (subpath,configSequence, 1);
	    }
	  }
	}
      }

      // --------------------------------------------------------------

      hpp::floatSeqSeq* Problem::getWaypoints (UShort pathId)
	throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver_->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver_->paths ().size () << ".";
	    throw std::runtime_error (oss.str ().c_str ());
	  }
	  hpp::floatSeqSeq *configSequence;
	  configSequence = new hpp::floatSeqSeq ();
	  PathVectorPtr_t path = problemSolver_->paths () [pathId];
	  //init path
	  findExtremes (path, *configSequence, 0);
	  findExtremities (path, *configSequence);
	  return configSequence;
	}
	catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::floatSeqSeq* Problem::nodes () throw (hpp::Error)
      {
	hpp::floatSeqSeq* res;
	try {
	  const Nodes_t & nodes
	    (problemSolver_->roadmap ()->nodes ());
	  res = new hpp::floatSeqSeq;
	  res->length (nodes.size ());
	  std::size_t i=0;
	  for (Nodes_t::const_iterator itNode = nodes.begin ();
	       itNode != nodes.end (); itNode++) {
	    ConfigurationPtr_t config = (*itNode)->configuration ();
	    ULong size = (ULong) config->size ();
	    double* dofArray = hpp::floatSeq::allocbuf(size);
	    hpp::floatSeq floats (size, size, dofArray, true);
	    //convert the config in dofseq
	    for (size_type j=0 ; j < config->size() ; ++j) {
	      dofArray[j] = (*config) [j];
	    }
	    (*res) [i] = floats;
	    ++i;
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	return res;
      }

      // ---------------------------------------------------------------

      Long Problem::numberEdges () throw (hpp::Error)
      {
	return problemSolver_->roadmap ()->edges ().size ();
      }

      // ---------------------------------------------------------------


      void Problem::edge (ULong edgeId, hpp::floatSeq_out q1,
			  hpp::floatSeq_out q2) throw (hpp::Error)
      {
	try {
	  const Edges_t & edges
	    (problemSolver_->roadmap ()->edges ());
	  Edges_t::const_iterator itEdge = edges.begin ();
	  std::size_t i=0;
	  while (i < edgeId) {
	    ++i; itEdge++;
	  }
	  ConfigurationPtr_t config1 = (*itEdge)->from ()->configuration ();
	  ConfigurationPtr_t config2 = (*itEdge)->to ()->configuration ();
	  ULong size = (ULong) config1->size ();

	  hpp::floatSeq* q1_ptr = new hpp::floatSeq ();
	  q1_ptr->length (size);
	  hpp::floatSeq* q2_ptr = new hpp::floatSeq ();
	  q2_ptr->length (size);

	  for (i=0; i<size; ++i) {
	    (*q1_ptr) [i] = (*config1) [i];
	    (*q2_ptr) [i] = (*config2) [i];
	  }
	  q1 = q1_ptr;
	  q2 = q2_ptr;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      Long Problem::numberNodes () throw (hpp::Error)
      {
          return problemSolver_->roadmap ()->nodes().size();
      }

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::node (ULong nodeId) throw (hpp::Error)
      {
      try {
        const Nodes_t & nodes (problemSolver_->roadmap()->nodes());

        if (nodes.size() > nodeId)
        {
            Nodes_t::const_iterator itNode = boost::next(nodes.begin(),nodeId);
            ConfigurationPtr_t conf = (*itNode)->configuration ();
            ULong size = (ULong) conf->size ();

            hpp::floatSeq* q_ptr = new hpp::floatSeq ();
            q_ptr->length (size);

            for (ULong i=0; i<size; ++i) {
            (*q_ptr) [i] = (*conf) [i];
            }
            return q_ptr;
        }else{
            std::ostringstream oss ("wrong nodeId :");
            oss << nodeId << ", number of nodes: "
            << nodes.size() << ".";
            throw std::runtime_error (oss.str ().c_str ());
        }
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // -----------------------------------------------------------------

      Long Problem::numberConnectedComponents () throw (hpp::Error)
      {
	return
	  problemSolver_->roadmap ()->connectedComponents ().size ();
      }

      // ---------------------------------------------------------------

      hpp::floatSeqSeq*
      Problem::nodesConnectedComponent (ULong connectedComponentId)
	throw (hpp::Error)
      {
	hpp::floatSeqSeq* res;
	try {
	  const ConnectedComponents_t& connectedComponents
	    (problemSolver_->roadmap ()->connectedComponents ());
	  ConnectedComponents_t::const_iterator itcc =
	    connectedComponents.begin ();
	  ULong i = 0;
	  while (i != connectedComponentId) {
	    ++i; itcc++;
	  }
      const Nodes_t & nodes ((*itcc)->nodes ());
	  res = new hpp::floatSeqSeq;
	  res->length (nodes.size ());
	  i=0;
	  for (Nodes_t::const_iterator itNode = nodes.begin ();
	       itNode != nodes.end (); itNode++) {
	    ConfigurationPtr_t config = (*itNode)->configuration ();
	    ULong size = (ULong) config->size ();
	    double* dofArray = hpp::floatSeq::allocbuf(size);
	    hpp::floatSeq floats (size, size, dofArray, true);
	    //convert the config in dofseq
	    for (size_type j=0 ; j < config->size() ; ++j) {
	      dofArray [j] = (*config) [j];
	    }
	    (*res) [i] = floats;
	    ++i;
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	return res;
      }

      // ---------------------------------------------------------------

      void Problem::clearRoadmap () throw (hpp::Error)
      {
	try {
	  problemSolver_->roadmap ()->clear ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::resetRoadmap ()
      {
	try {
	  problemSolver_->resetRoadmap ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }
    } // namespace impl
  } // namespace corbaServer
} // namespace hpp
