// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORE_IDL_DISTANCES_HH
# define HPP_CORE_IDL_DISTANCES_HH

# include <vector>
# include <stdlib.h>

# include "hpp/core/distance.hh"
# include "hpp/core/weighed-distance.hh"

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/conversions.hh>
# include "hpp/core_idl/distances-idl.hh"

# include "hpp/corbaserver/servant-base.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace core_idl
    {
      template <typename D>
      class DistanceStorage : public AbstractStorage <D, core::Distance>
      {
        public:
          typedef AbstractStorage <D, core::Distance> parent_t;
          using parent_t::element;
          using typename parent_t::ptr_t;

          core::DevicePtr_t r;
          DistanceStorage (const core::DevicePtr_t& _r, const ptr_t& _d)
            : parent_t(_d), r(_r) {}

          template <typename T> DistanceStorage<T> cast () const
          {
            return DistanceStorage<T> (r, HPP_DYNAMIC_PTR_CAST(T, element.lock()));
          }
      };

      template <typename _Base, typename _Storage>
      class DistanceServant : public ServantBase<core::Distance, _Storage>, public virtual _Base
      {
          SERVANT_BASE_TYPEDEFS(hpp::core_idl::Distance, core::Distance);
        public:
          DistanceServant (Server* server, const Storage& s)
            : _ServantBase (server, s) {}

          virtual ~DistanceServant () {}

          CORBA::Double value (const floatSeq& q1, const floatSeq& q2)
          {
            Configuration_t qq1 (floatSeqToConfig(getS().r, q1, true)),
                            qq2 (floatSeqToConfig(getS().r, q2, true));
            return (*get()) (qq1,qq2);
          }
      };

      typedef DistanceServant<POA_hpp::core_idl::Distance, DistanceStorage<core::Distance> > Distance;

      template <typename _Base, typename _Storage>
      class WeighedDistanceServant : public DistanceServant<_Base, _Storage>
      {
          SERVANT_BASE_TYPEDEFS(hpp::core_idl::WeighedDistance, core::Distance);
        public:
          typedef DistanceServant<Base, Storage> Parent;

          WeighedDistanceServant (Server* server, const Storage& s)
            : Parent(server, s) {}

          ~WeighedDistanceServant () {}

          floatSeq* getWeights ()
          {
            return vectorToFloatSeq (getT()->weights());
          }

          void setWeights (const floatSeq& weights)
          {
            try {
              return getT()->weights(floatSeqToVector(weights));
            } catch (const std::exception& e) {
              throw Error (e.what ());
            }
          }
      };

      typedef WeighedDistanceServant<POA_hpp::core_idl::WeighedDistance, DistanceStorage<core::WeighedDistance> > WeighedDistance;
    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // HPP_CORE_IDL_DISTANCES_HH
