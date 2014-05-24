// Copyright (C) 2009, 2010 by Florent Lamiraux, Andreas Orthey, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_PRECOMPUTATION_IMPL_HH
#define HPP_CORBASERVER_PRECOMPUTATION_IMPL_HH
# include <vector>

# include <hpp/corbaserver/fwd.hh>
# include <precomputation.hh>
# include <robot.hh>
# include "common.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      /// \brief Implement CORBA interface ``Precomputation''.
      class Precomputation : public virtual POA_hpp::corbaserver::Precomputation
      {
      public:

	Precomputation (corbaServer::Server* server);

	virtual Short getNumberDof () throw (hpp::Error);

	virtual hpp::floatSeq* computeVolume () throw (hpp::Error);

	virtual hpp::floatSeq* parseCapsulePoints () throw (hpp::Error);

	virtual hpp::floatSeq* projectConfigurationUntilIrreducible 
          (const hpp::floatSeq& dofArray) throw (hpp::Error);

        virtual hpp::floatSeq* gradientConfigurationWrtProjection 
          (const hpp::floatSeq& dofArray) throw (hpp::Error);

        virtual hpp::floatSeq* gradientConfigurationWrtProjection () 
                throw (hpp::Error);

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
#endif 
