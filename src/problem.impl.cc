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

#include <hpp/util/debug.hh>
#include <hpp/util/portability.hh>

#include <hpp/core/connected-component.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/locked-dof.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/corbaserver/server.hh>

#include "problem.impl.hh"


// ajout Nassime ////////////////////////////////////////////////////////////
#include <hpp/constraints/position.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/model/humanoid-robot.hh>
#include <hpp/model/joint.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/constraints/orientation.hh>
#include <hpp/constraints/position.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/relative-orientation.hh>
#include <hpp/constraints/relative-position.hh>
#include <hpp/wholebody-step/static-stability-constraint.hh>
    using hpp::constraints::Orientation;
    using hpp::constraints::OrientationPtr_t;
    using hpp::constraints::Position;
    using hpp::constraints::PositionPtr_t;
    using hpp::constraints::RelativeOrientation;
    using hpp::constraints::RelativeComPtr_t;
    using hpp::constraints::RelativeCom;
    using hpp::constraints::RelativeOrientationPtr_t;
    using hpp::constraints::RelativePosition;
    using hpp::constraints::RelativePositionPtr_t;
/////////////////////////////////////////////////////////////////////////////



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
	unsigned int configDim = (unsigned int)dofArray.length();
	ConfigurationPtr_t config (new Configuration_t (configDim));

	// Get robot in hppPlanner object.
	DevicePtr_t robot = problemSolver->robot ();

	// Compare size of input array with number of degrees of freedom of
	// robot.
	if (configDim != robot->configSize ()) {
	  hppDout (error, "robot nb dof=" << configDim << " is different from config size=" << robot->configSize());
	  throw std::runtime_error
	    ("robot nb dof is different from config size");
	}

	// Fill dof vector with dof array.
	for (unsigned int iDof=0; iDof < configDim; ++iDof) {
	  (*config) [iDof] = dofArray [iDof];
	}
	return config;
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


      // ---------------------------------------------------------------
      void Problem::createPositionConstraints (const hpp::floatSeq& input,
				      hpp::floatSeq_out output)
	throw (hpp::Error)
      {

                //Params
                JointPtr_t joint1 = problemSolver_->robot()->getJointByName("RLEG_JOINT4");
                JointPtr_t joint2 = problemSolver_->robot()->getJointByName("RARM_JOINT4");
                vector3_t local1;
                vector3_t local2;
                local1.setZero();


		fcl::Vec3f v(0,0,0);
		const fcl::Vec3f& vr(v);

                const Transform3f& transfo1 = joint1->currentTransformation ();
                const Transform3f& transfo2 = joint2->currentTransformation ();
                hpp::model::matrix3_t rot1 (transfo1.getRotation ());
                hpp::model::matrix3_t rot2 (transfo2.getRotation ());
		hpp::model::matrix3_t reference = rot1 * rot2;
                hpp::model::matrix3_t rotNulle;
		rotNulle.setZero();
		
		Transform3f transfo3;
		transfo3.setTranslation(vr);

		
		//const hpp::model::matrix3_t& pRotNulle = rotNulle;

                //local2 = rot2 * (transfo1.getTranslation() - transfo2.getTranslation ());
                //local2.setZero();
                //local2.setValue(0,0,0.0);

		local2.setZero();
		//local2 = (transfo1.getTranslation() - transfo2.getTranslation ());


                // Adding constraints into projector
//		problemSolver_->constraints()->configProjector()->addConstraint(
//			RelativePosition::create(problemSolver_->robot(), joint1, joint2, local1, local2));
//		problemSolver_->constraints()->configProjector()->addConstraint(
//			RelativeOrientation::create(problemSolver_->robot(), joint1, joint2, rotNulle, false) );


	problemSolver_->constraints()->configProjector()->addConstraint(
		RelativePosition::create (problemSolver_->robot(), joint1, joint2, 
				local1,
			 	vector3_t (0, 0, 0.8))
	);





	ConfigurationPtr_t config = floatSeqToConfig (problemSolver_, input);
	try {
	  if (!problemSolver_->constraints ()->apply (*config)) {
	    throw hpp::Error ("Failed to apply constraint");
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
	}
      // ---------------------------------------------------------------
      

	void Problem::applyConstraints (const hpp::floatSeq& input,
				      hpp::floatSeq_out output)
	throw (hpp::Error)
      {
	ConfigurationPtr_t config = floatSeqToConfig (problemSolver_, input);
	try {
	  if (!problemSolver_->constraints ()->apply (*config)) {
	    throw hpp::Error ("Failed to apply constraint");
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
      }

      // ---------------------------------------------------------------

      void Problem::resetConstraints ()	throw (hpp::Error)
      {
	try {
	  problemSolver_->resetConstraints ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::setNumericalConstraints
      (const char* constraintName, const Names_t& constraintNames)
	throw (Error)
      {
	using core::ConstraintSetPtr_t;
	using core::ConfigProjector;
	using core::ConfigProjectorPtr_t;
	try {
	  const ConstraintSetPtr_t& constraints
	    (problemSolver_->constraints ());
	  const DevicePtr_t& robot (problemSolver_->robot ());
	  if (!robot) {
	    throw Error ("You should set the robot before defining"
			 " constraints.");
	  }
	  ConfigProjectorPtr_t  configProjector =
	    constraints->configProjector ();
	  if (!configProjector) {
	    configProjector = ConfigProjector::create
	      (robot, constraintName, problemSolver_->errorThreshold (),
	       problemSolver_->maxIterations ());
	    constraints->addConstraint (configProjector);
	  }
	  for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
	    std::string name (constraintNames [i]);
	    configProjector->addConstraint (problemSolver_->numericalConstraint
					    (name));
	  }
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::lockDof (const char* jointName, Double value)
	throw (hpp::Error)
      {
	try {
	  // Get robot in hppPlanner object.
	  DevicePtr_t robot = problemSolver_->robot ();
	  JointPtr_t joint = robot->getJointByName (jointName);
	  size_type dofId = joint->rankInConfiguration ();
	  std::ostringstream oss;
	  oss << "locked dof, index: " << dofId << ", value: " << value;

	  LockedDofPtr_t lockedDof (LockedDof::create (oss.str (),
						       dofId, value));
	  problemSolver_->addConstraint (lockedDof);
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

      void Problem::solve () throw (hpp::Error)
      {
	try {
	  problemSolver_->solve();
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
	  PathVectorPtr_t path (core::PathVector::create (dp->outputSize ()));
	  path->appendPath (dp);
	  problemSolver_->addPath (path);
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
	  PathVectorPtr_t initial = problemSolver_->paths () [pathId];
	  PathVectorPtr_t optimized =
	    problemSolver_->pathOptimizer ()-> optimize (initial);
	  problemSolver_->addPath (optimized);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      Double Problem::pathLength (UShort pathId) throw (hpp::Error)
      {
	try {
	  return problemSolver_->paths () [pathId]->length ();
	} catch (std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::configAtDistance (UShort inPathId,
					      Double atDistance)
	throw (hpp::Error)
      {
	try {
	  PathPtr_t path = problemSolver_->paths () [inPathId];
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
	    for (std::size_t j=0 ; j < config->size() ; ++j) {
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
	    for (std::size_t j=0 ; j < config->size() ; ++j) {
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
    } // namespace impl
  } // namespace corbaServer
} // namespace hpp
