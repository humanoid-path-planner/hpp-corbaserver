
// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef SRC_PROBLEM_HH
# define SRC_PROBLEM_HH
# include <vector>
# include <stdlib.h>

# include "hpp/core/path.hh"

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/conversions.hh>
# include "hpp/core_idl/_problem-idl.hh"

# include "../servant-base.hh"
# include "distances.hh"
# include "steering-methods.hh"
# include "path-validations.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace core_idl
    {
      class Problem : public virtual POA_hpp::core_idl::Problem
      {
        public:
          typedef core::ProblemPtr_t Storage;
          SERVANT_BASE_TYPEDEFS(hpp::core_idl::Problem)

          Problem (Server* server, const Storage& p) : server_ (server), p_ (p) {}

          virtual ~Problem () {}

          hpp::core_idl::Distance_ptr getDistance () throw (hpp::Error)
          {
            core::DistancePtr_t distance = p_->distance();
            DevicePtr_t robot = p_->robot();

            hpp::core_idl::Distance_var d = makeServantDownCast <
              hpp::core_idl::Distance, hpp::core_idl::Distance_Helper,
              Distances>
                (server_, Distance::Storage (robot, distance));
            return d._retn();
          }

          void setDistance (hpp::core_idl::Distance_ptr distance) throw (hpp::Error)
          {
            core::DistancePtr_t d;
            try {
              d = reference_to_servant_base<core::DistancePtr_t>(server_, distance)->get();
            } catch (const Error& e) {
              // TODO in this case, we should define a distance from the CORBA type.
              // This would allow to implement a distance class in Python.
              throw;
            }

            p_->distance (d);
          }

          hpp::core_idl::SteeringMethod_ptr getSteeringMethod () throw (hpp::Error)
          {
            core::SteeringMethodPtr_t steeringMethod = p_->steeringMethod();
            DevicePtr_t robot = p_->robot();

            hpp::core_idl::SteeringMethod_var d = makeServantDownCast <
              hpp::core_idl::SteeringMethod, hpp::core_idl::SteeringMethod_Helper,
              SteeringMethods>
                (server_, SteeringMethod::Storage (robot, steeringMethod));
            return d._retn();
          }

          void setSteeringMethod (hpp::core_idl::SteeringMethod_ptr steeringMethod) throw (hpp::Error)
          {
            core::SteeringMethodPtr_t d;
            try {
              d = reference_to_servant_base<core::SteeringMethodPtr_t>(server_, steeringMethod)->get();
            } catch (const Error& e) {
              // TODO in this case, we should define a steeringMethod from the CORBA type.
              // This would allow to implement a steeringMethod class in Python.
              throw;
            }

            p_->steeringMethod (d);
          }

        protected:
          Server* server_;
          Storage p_;
      };
    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // SRC_PROBLEM_HH