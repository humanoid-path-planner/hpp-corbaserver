// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef SRC_STEERING_METHODS_HH
# define SRC_STEERING_METHODS_HH

# include <vector>
# include <stdlib.h>

# include "hpp/core/steering-method.hh"

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/conversions.hh>
# include "hpp/core_idl/steering_methods-idl.hh"

# include "../servant-base.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace core_idl
    {
      typedef AbstractServantBase<core::SteeringMethodPtr_t> SteeringMethodBase;

      template <typename D>
      struct SteeringMethodStorage {
        core::DevicePtr_t r;
        D d;
        SteeringMethodStorage (const core::DevicePtr_t& _r, const D& _d) : r(_r), d(_d) {}
        operator core::SteeringMethodPtr_t () const { return d; }
      };

      template <typename Base, typename _Storage>
      class SteeringMethodServant : public SteeringMethodBase, public virtual Base
      {
        public:
          typedef _Storage Storage;

          SteeringMethodServant (Server* server, const Storage& s) :
            SteeringMethodBase (server), s_ (s) {}

          virtual ~SteeringMethodServant () {}

          hpp::core_idl::Path_ptr call (const floatSeq& q1, const floatSeq& q2) throw (Error)
          {
            Configuration_t qq1 (floatSeqToConfig(s_.r, q1, true)),
                            qq2 (floatSeqToConfig(s_.r, q2, true));
            return makeServant<hpp::core_idl::Path_ptr> (server_,
                new Path (server_, (*s_.d) (qq1,qq2)));
          }

          virtual core::SteeringMethodPtr_t get ()
          {
            return (core::SteeringMethodPtr_t)s_;
          }

          Storage getS ()
          {
            return s_;
          }

        protected:
          Storage s_;
      };

      typedef SteeringMethodServant<POA_hpp::core_idl::SteeringMethod, SteeringMethodStorage<core::SteeringMethodPtr_t> > SteeringMethod;

    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // SRC_STEERING_METHODS_HH
