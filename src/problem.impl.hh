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
# include "hpp/corbaserver/common.stub.hh"
# include "hpp/corbaserver/problem.stub.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      /// \brief Implement CORBA interface ``Problem''.
      class Problem : public virtual POA_hpp::Problem
      {
      public:
	Problem (corbaServer::Server* server);

	virtual Short
	setSteeringMethod
	(UShort problemId, const char* steeringMethod, Boolean oriented);

	virtual Short
	setRoadmapbuilder
	(UShort problemId, const char* roadmapBuilder, Boolean display);

	virtual Short
	setDiffusingNode
	(UShort problemId, const char* diffusingNode);

	virtual Short
	setPathOptimizer
	(UShort problemId, const char* pathOptimizer, UShort maxNumberLoop);

	virtual Short
	setConfigExtractor
	(UShort problemId, Double minRadius, Double maxRadius, Double scaleFactor);

	virtual Short
	setDistanceFunction
	(UShort problemId, const char* distanceName, Boolean oriented);

	virtual Short
	setDiffusionNodePicker
	(UShort problemId, const char* diffusionNodePickerName);

	virtual Short
	setDiffusionShooter
	(UShort problemId, const char* diffusionShooterName, Double standardDeviation);

	virtual Short
	setInitialConfig
	(UShort problemId, const hpp::dofSeq& dofArray);

	virtual Short
	setGoalConfig
	(UShort problemId, const hpp::dofSeq& dofArray);

	virtual hpp::dofSeq*
	getInitialConfig (UShort problemId) throw (SystemException);

	virtual hpp::dofSeq*
	getGoalConfig (UShort problemId) throw (SystemException);

	virtual Short initializeProblem ();

	virtual Short
	solveOneProblem
	(UShort problemId, Short& inLastPathId, Double& pathLength);

	virtual Short solve ();

	virtual Short interruptPathPlanning ();

	virtual Short optimizePath (UShort problemId, UShort pathId);

	virtual Double pathLength (UShort problemId, UShort pathId);

	virtual hpp::dofSeq* configAtDistance
	(UShort problemId, UShort pathId, Double atDistance);

	virtual Short setObstacleTolerance
	(UShort problemId, Double tolerance) throw (SystemException);

	virtual Short parseFile
	(const char* inFilename) throw (SystemException);


      private:
	/// \brief Pointer to the ChppciServer owning this object
	corbaServer::Server* server_;
	/// \brief Pointer to hppPlanner object of hppciServer.
	/// Instantiated at construction.
	ChppPlanner* planner_;
      };
    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_OBSTACLE_IMPL_HH
