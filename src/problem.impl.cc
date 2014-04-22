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
	  hppDout (error, "robot nb dof is different from config size");
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

      Short Problem::setInitialConfig (const hpp::floatSeq& dofArray)
      {
	try {
	  ConfigurationPtr_t config = floatSeqToConfig (problemSolver_,
							dofArray);
	  problemSolver_->initConfig (config);
	} catch (const std::exception& exc) {
	  hppDout (error, ":setInitialConfig: " << exc.what ());
	  return -1;
	}
	return 0;
      }

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::getInitialConfig()
	throw(SystemException)
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
	  hppDout (error,
		   ":getInitialConfig: no initial configuration defined");
	  dofArray = new hpp::floatSeq(1);
	  return dofArray;
	}
	return new hpp::floatSeq(1);
      }

      // ---------------------------------------------------------------

      Short Problem::addGoalConfig (const hpp::floatSeq& dofArray)
      {
	try {
	  ConfigurationPtr_t config = floatSeqToConfig (problemSolver_,
							dofArray);
	  problemSolver_->addGoalConfig (config);
	  return 0;
	} catch (const std::exception& exc) {
	  hppDout (error, ":addGoalConfig: " << exc.what ());
	  return -1;
	}
	return 0;
      }

      // ---------------------------------------------------------------

      hpp::floatSeqSeq* Problem::getGoalConfigs () throw(SystemException)
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
	  hppDout (error, exc.what ());
	}
	return new hpp::floatSeqSeq (1);
      }

      // ---------------------------------------------------------------

      Short Problem::resetGoalConfigs () throw (SystemException)
      {
	problemSolver_->resetGoalConfigs ();
	return 0;
      }


      // ---------------------------------------------------------------

      Short Problem::applyConstraints (const hpp::floatSeq& input,
					hpp::floatSeq_out output)
      {
	ConfigurationPtr_t config = floatSeqToConfig (problemSolver_, input);
	try {
	  if (!problemSolver_->constraints ()->apply (*config)) {
	    hppDout (error, "Failed to apply constraint");
	    hpp::floatSeq* q_ptr = new hpp::floatSeq ();
	    q_ptr->length (0);
	    output = q_ptr;
	    return -1;
	  }
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  hpp::floatSeq* q_ptr = new hpp::floatSeq ();
	  q_ptr->length (0);
	  output = q_ptr;
	  return -1;
	}
	ULong size = (ULong) config->size ();
	hpp::floatSeq* q_ptr = new hpp::floatSeq ();
	q_ptr->length (size);

	for (std::size_t i=0; i<size; ++i) {
	  (*q_ptr) [i] = (*config) [i];
	}
	output = q_ptr;
	return 0;
      }

      // ---------------------------------------------------------------

      Short Problem::resetConstraints ()
      {
	try {
	  problemSolver_->resetConstraints ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
	return 0;
      }

      // ---------------------------------------------------------------

      Short Problem::lockDof (UShort dofId, Double value)
      {
	try {
	  std::ostringstream oss;
	  oss << "locked dof, index: " << dofId << ", value: " << value;

	  LockedDofPtr_t lockedDof (LockedDof::create (oss.str (),
						       dofId, value));
	  problemSolver_->addConstraint (lockedDof);
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
	return 0;
      }

      // ---------------------------------------------------------------

      Short Problem::solve ()
      {
	try {
	  problemSolver_->solve();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
	return 0;
      }

      // ---------------------------------------------------------------

      Short Problem::directPath (const hpp::floatSeq& startConfig,
				 const hpp::floatSeq& endConfig,
				 CORBA::String_out message)
	throw (SystemException)
      {
	message = "Success";
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
	  hppDout (error, "directPath: " << exc.what ());
	  message = exc.what ();
	  return -1;
	}
	return 0;
      }

      // ---------------------------------------------------------------

      Short Problem::interruptPathPlanning()
      {
	problemSolver_->pathPlanner ()->interrupt ();
	return 0;
      }

      // ---------------------------------------------------------------

      Short Problem::numberPaths ()
      {
	try {
	  return (Short) problemSolver_->paths ().size ();
	} catch (std::exception& exc) {
	  hppDout (error, exc.what ());
	}
	return -1;
      }

      // ---------------------------------------------------------------

      Short Problem::optimizePath(UShort pathId)
      {
	try {
	  PathVectorPtr_t initial = problemSolver_->paths () [pathId];
	  PathVectorPtr_t optimized =
	    problemSolver_->pathOptimizer ()-> optimize (initial);
	  problemSolver_->addPath (optimized);
	  return 0;
	} catch (const std::exception& exc) {
	  return -1;
	}
      }

      // ---------------------------------------------------------------

      Double Problem::pathLength(UShort pathId)
      {
	try {
	  return problemSolver_->paths () [pathId]->length ();
	} catch (std::exception& exc) {
	  hppDout (error, exc.what ());
	}
	return -1.;
      }

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::configAtDistance (UShort inPathId,
					      Double atDistance)
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
	  hppDout (error, exc.what ());
	}
	return new hpp::floatSeq(0, 0, NULL, true);
      }

      // ---------------------------------------------------------------

      hpp::floatSeqSeq* Problem::nodes ()
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
	  hppDout (error, exc.what ());
	  res->length (0);
	}
	return res;
      }

      // ---------------------------------------------------------------

      Long Problem::numberEdges ()
      {
	return problemSolver_->roadmap ()->edges ().size ();
      }

      // ---------------------------------------------------------------

      Short Problem::edge (ULong edgeId, hpp::floatSeq_out q1,
			   hpp::floatSeq_out q2)
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
	  return 0;
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
      }

      // ---------------------------------------------------------------

      Long Problem::numberConnectedComponents ()
      {
	return
	  problemSolver_->roadmap ()->connectedComponents ().size ();
      }

      // ---------------------------------------------------------------

      hpp::floatSeqSeq*
      Problem::nodesConnectedComponent (ULong connectedComponentId)
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
	  hppDout (error, exc.what ());
	  res->length (0);
	}
	return res;
      }

      // ---------------------------------------------------------------

      Short Problem::clearRoadmap ()
      {
	try {
	  problemSolver_->roadmap ()->clear ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
	return 0;
      }
    } // namespace impl
  } // namespace corbaServer
} // namespace hpp
