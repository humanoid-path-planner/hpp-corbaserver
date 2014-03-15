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
      class Problem : public virtual POA_hpp::Problem
      {
      public:
	Problem (corbaServer::Server* server);

	virtual Short
	setInitialConfig (const hpp::floatSeq& dofArray);

	virtual hpp::floatSeq*
	getInitialConfig () throw (SystemException);

	virtual Short
	addGoalConfig (const hpp::floatSeq& dofArray);

	virtual hpp::floatSeqSeq*	getGoalConfigs () throw (SystemException);

	virtual Short
	resetGoalConfigs ()  throw (SystemException);

	virtual Short applyConstraints (const hpp::floatSeq& input,
					hpp::floatSeq_out output);
	virtual Short resetConstraints ();
	virtual Short lockDof (UShort dofId, Double value);
	virtual Short solve ();

	virtual Short directPath (const hpp::floatSeq& startConfig,
				  const hpp::floatSeq& endConfig,
				  CORBA::String_out message)
	  throw (SystemException);

	virtual Short interruptPathPlanning ();

	virtual Short numberPaths ();

	virtual Short optimizePath (UShort pathId);

	virtual Double pathLength (UShort pathId);

	virtual hpp::floatSeq* configAtDistance (UShort pathId,
					       Double atDistance);

	virtual hpp::floatSeqSeq* nodes ();
	virtual Long numberEdges ();
	virtual Short
	edge (ULong edgeId, hpp::floatSeq_out q1, hpp::floatSeq_out q2);
	virtual Long numberConnectedComponents ();
	virtual hpp::floatSeqSeq*
	nodesConnectedComponent (ULong connectedComponentId);
	virtual Short clearRoadmap ();

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
