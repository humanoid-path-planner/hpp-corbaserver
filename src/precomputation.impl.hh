// Copyright (C) 2009, 2010 by Florent Lamiraux, Andreas Orthey, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#pragma once
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
      //class Precomputation
      class Precomputation : public virtual POA_hpp::corbaserver::Precomputation
      {
      public:
	Precomputation (corbaServer::Server* server);

	virtual Short getNumberDof () throw (hpp::Error);

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
