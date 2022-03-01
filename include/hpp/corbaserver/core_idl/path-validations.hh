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

#ifndef HPP_CORE_IDL_PATH_VALIATIONS_HH
# define HPP_CORE_IDL_PATH_VALIATIONS_HH

# include <vector>
# include <stdlib.h>

# include "hpp/core/path-validation.hh"
# include "hpp/core/path-validation-report.hh"

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/conversions.hh>
# include "hpp/core_idl/path_validations-idl.hh"

# include "hpp/corbaserver/servant-base.hh"
# include <hpp/corbaserver/core_idl/paths.hh>

namespace hpp
{
  namespace corbaServer
  {
    namespace core_idl
    {
      template <typename _Base, typename _Storage>
      class PathValidationServant : public ServantBase<core::PathValidation, _Storage>, public virtual _Base
      {
          SERVANT_BASE_TYPEDEFS(hpp::core_idl::PathValidation, core::PathValidation);

        public:
          PathValidationServant (Server* server, const Storage& s) :
            _ServantBase (server, s) {}

          virtual ~PathValidationServant () {}

          CORBA::Boolean validate (hpp::core_idl::Path_ptr path,
              CORBA::Boolean reverse,
              hpp::core_idl::Path_out validPart,
              hpp::core_idl::PathValidationReport_out report)
          {
            core::PathPtr_t p (reference_to_servant_base<core::Path>(server_, path)->get());
            core::PathPtr_t vp;
            core::PathValidationReportPtr_t pvr;

            bool res = get()->validate (p, reverse, vp, pvr);

            if (pvr) {
              std::ostringstream oss; oss << *pvr;
              std::string res = oss.str();
              report = CORBA::string_dup(res.c_str());
            } else {
              report = CORBA::string_dup("");
            }

            validPart = makeServant<hpp::core_idl::Path_ptr> (server_, new Path (server_, vp));
            return res;
          }
      };

      typedef PathValidationServant<POA_hpp::core_idl::PathValidation, core::PathValidationPtr_t > PathValidation;

    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // HPP_CORE_IDL_PATH_VALIATIONS_HH
