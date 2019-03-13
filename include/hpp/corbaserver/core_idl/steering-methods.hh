// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORE_IDL_STEERING_METHODS_HH
# define HPP_CORE_IDL_STEERING_METHODS_HH

# include <vector>
# include <stdlib.h>

# include "hpp/core/steering-method.hh"

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/conversions.hh>
# include "hpp/core_idl/steering_methods-idl.hh"

# include "hpp/corbaserver/servant-base.hh"

# include <hpp/corbaserver/core_idl/paths.hh>

namespace hpp
{
  namespace corbaServer
  {
    namespace core_idl
    {
      typedef AbstractServantBase<core::SteeringMethodPtr_t> SteeringMethodBase;

      template <typename D>
      class SteeringMethodStorage : public AbstractStorage<D, core::SteeringMethod>
      {
        public:
          typedef AbstractStorage <D, core::SteeringMethod> parent_t;
          using parent_t::element;
          using typename parent_t::ptr_t;

          core::DevicePtr_t r;
          SteeringMethodStorage (const core::DevicePtr_t& _r, const ptr_t& _d)
            : parent_t(_d), r(_r) {}

          template <typename T> SteeringMethodStorage<T> cast () const
          {
            return SteeringMethodStorage<T> (r, HPP_DYNAMIC_PTR_CAST(T, element));
          }
      };

      template <typename _Base, typename _Storage>
      class SteeringMethodServant : public ServantBase<core::SteeringMethodPtr_t, _Storage>, public virtual _Base
      {
          SERVANT_BASE_TYPEDEFS(hpp::core_idl::SteeringMethod, core::SteeringMethodPtr_t);

        public:
          SteeringMethodServant (Server* server, const Storage& s) :
            _ServantBase (server, s) {}

          virtual ~SteeringMethodServant () {}

          hpp::core_idl::Path_ptr call (const floatSeq& q1, const floatSeq& q2) throw (Error)
          {
            Configuration_t qq1 (floatSeqToConfig(getS().r, q1, true)),
                            qq2 (floatSeqToConfig(getS().r, q2, true));
            return makeServant<hpp::core_idl::Path_ptr> (server_,
                new Path (server_, (*get()) (qq1,qq2)));
          }
      };

      typedef SteeringMethodServant<POA_hpp::core_idl::SteeringMethod, SteeringMethodStorage<core::SteeringMethod> > SteeringMethod;
    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // HPP_CORE_IDL_STEERING_METHODS_HH
