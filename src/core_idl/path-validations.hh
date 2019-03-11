// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef SRC_PATH_VALIATIONS_HH
# define SRC_PATH_VALIATIONS_HH

# include <vector>
# include <stdlib.h>

# include "hpp/core/path-validation.hh"

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/conversions.hh>
# include "hpp/core_idl/path_validations-idl.hh"

# include "../servant-base.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace core_idl
    {
      typedef AbstractServantBase<core::PathValidationPtr_t> PathValidationBase;

      template <typename D>
      struct PathValidationStorage {
        D d;
        PathValidationStorage (const D& _d) : d(_d) {}
        operator core::PathValidationPtr_t () const { return d; }
      };

      template <typename Base, typename _Storage>
      class PathValidationServant : public PathValidationBase, public virtual Base
      {
        public:
          typedef _Storage Storage;

          PathValidationServant (Server* server, const Storage& s) :
            PathValidationBase (server), s_ (s) {}

          virtual ~PathValidationServant () {}

          CORBA::Boolean validate (hpp::core_idl::Path_ptr path,
              CORBA::Boolean reverse,
              hpp::core_idl::Path_out validPart,
              hpp::core_idl::PathValidationReport_out report) throw (Error)
          {
            core::PathPtr_t p (reference_to_servant_base<core::PathPtr_t>(server_, path)->get());
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

          virtual core::PathValidationPtr_t get ()
          {
            return (core::PathValidationPtr_t)s_;
          }

          Storage getS ()
          {
            return s_;
          }

        protected:
          Storage s_;
      };

      typedef PathValidationServant<POA_hpp::core_idl::PathValidation, PathValidationStorage<core::PathValidationPtr_t> > PathValidation;

    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // SRC_PATH_VALIATIONS_HH
