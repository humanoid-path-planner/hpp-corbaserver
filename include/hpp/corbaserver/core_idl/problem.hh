
// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
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
