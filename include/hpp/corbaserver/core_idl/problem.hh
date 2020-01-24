
// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORE_IDL_PROBLEM_HH
# define HPP_CORE_IDL_PROBLEM_HH
# include <vector>
# include <stdlib.h>

# include <hpp/core/problem.hh>

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/conversions.hh>
# include <hpp/core_idl/_problem-idl.hh>

# include <hpp/corbaserver/servant-base.hh>
# include <hpp/corbaserver/core_idl/distances.hh>
# include <hpp/corbaserver/core_idl/steering-methods.hh>
# include <hpp/corbaserver/core_idl/path-validations.hh>

namespace hpp
{
  namespace corbaServer
  {
    namespace core_idl
    {
      template <typename _Base, typename _Storage>
      class ProblemServant : public ServantBase<core::Problem, _Storage >, public virtual _Base
      {
          SERVANT_BASE_TYPEDEFS(hpp::core_idl::Problem, core::Problem);
        public:
          ProblemServant (Server* server, const Storage& p)
            : _ServantBase (server, p) {}

          virtual ~ProblemServant () {}

          hpp::core_idl::Distance_ptr getDistance ()
          {
            core::DistancePtr_t distance = get()->distance();
            DevicePtr_t robot = get()->robot();

            hpp::core_idl::Distance_var d = makeServantDownCast<Distance>
              (server_, Distance::Storage (robot, distance));
            return d._retn();
          }

          void setDistance (hpp::core_idl::Distance_ptr distance)
          {
            core::DistancePtr_t d;
            try {
              d = reference_to_servant_base<core::Distance>(server_, distance)->get();
            } catch (const Error& e) {
              // TODO in this case, we should define a distance from the CORBA type.
              // This would allow to implement a distance class in Python.
              throw;
            }

            get()->distance (d);
          }

          hpp::core_idl::SteeringMethod_ptr getSteeringMethod ()
          {
            core::SteeringMethodPtr_t steeringMethod = get()->steeringMethod();
            DevicePtr_t robot = get()->robot();

            hpp::core_idl::SteeringMethod_var d = makeServantDownCast<SteeringMethod>
                (server_, SteeringMethod::Storage (robot, steeringMethod));
            return d._retn();
          }

          void setSteeringMethod (hpp::core_idl::SteeringMethod_ptr steeringMethod)
          {
            core::SteeringMethodPtr_t d;
            try {
              d = reference_to_servant_base<core::SteeringMethod>(server_, steeringMethod)->get();
            } catch (const Error& e) {
              // TODO in this case, we should define a steeringMethod from the CORBA type.
              // This would allow to implement a steeringMethod class in Python.
              throw;
            }

            get()->steeringMethod (d);
          }

          hpp::core_idl::PathValidation_ptr getPathValidation ()
          {
            core::PathValidationPtr_t pathValidation = get()->pathValidation();

            hpp::core_idl::PathValidation_var d = makeServantDownCast<PathValidation>
                (server_, PathValidation::Storage (pathValidation));
            return d._retn();
          }

          void setPathValidation (hpp::core_idl::PathValidation_ptr pathValidation)
          {
            core::PathValidationPtr_t d;
            try {
              d = reference_to_servant_base<core::PathValidation>(server_, pathValidation)->get();
            } catch (const Error& e) {
              // TODO in this case, we should define a pathValidation from the CORBA type.
              // This would allow to implement a pathValidation class in Python.
              throw;
            }

            get()->pathValidation (d);
          }
      };

      typedef ProblemServant<POA_hpp::core_idl::Problem, core::ProblemPtr_t> Problem;
    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // HPP_CORE_IDL_PROBLEM_HH
