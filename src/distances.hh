// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef SRC_DISTANCES_HH
# define SRC_DISTANCES_HH

# include <vector>
# include <stdlib.h>

# include "hpp/core/weighed-distance.hh"

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/conversions.hh>
# include "hpp/core_idl/distances-idl.hh"

# include "servant-base.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace core_idl
    {
      typedef AbstractServantBase<core::DistancePtr_t> DistanceBase;

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
            return DistanceStorage<T> (r, HPP_DYNAMIC_PTR_CAST(T, element));
          }
      };

      template <typename _Base, typename _Storage>
      class DistanceServant : public DistanceBase, public virtual _Base
      {
        public:
          typedef _Base    Base;
          typedef _Storage Storage;
          SERVANT_BASE_TYPEDEFS(hpp::core_idl::Distance)

          DistanceServant (Server* server, const Storage& s) :
            DistanceBase (server), s_ (s) {}

          virtual ~DistanceServant () {}

          CORBA::Double value (const floatSeq& q1, const floatSeq& q2) throw (Error)
          {
            Configuration_t qq1 (floatSeqToConfig(s_.r, q1, true)),
                            qq2 (floatSeqToConfig(s_.r, q2, true));
            return (*s_.element) (qq1,qq2);
          }

          virtual core::DistancePtr_t get ()
          {
            return (core::DistancePtr_t)s_;
          }

          Storage getS ()
          {
            return s_;
          }

        protected:
          Storage s_;
      };

      typedef DistanceServant<POA_hpp::core_idl::Distance, DistanceStorage<core::Distance> > Distance;

      template <typename _Base, typename Storage>
      class WeighedDistanceServant : public DistanceServant<_Base, Storage>
      {
        protected:
          using DistanceBase::server_;
        public:
          typedef DistanceServant<_Base, Storage> Parent;
          using Parent::getS;
          SERVANT_BASE_TYPEDEFS(hpp::core_idl::WeighedDistance)

          WeighedDistanceServant (Server* server, const Storage& s)
            : Parent(server, s) {}

          ~WeighedDistanceServant () {}

          floatSeq* getWeights () throw (Error)
          {
            return vectorToFloatSeq (getS().element->weights());
          }

          void setWeights (const floatSeq& weights) throw (Error)
          {
            try {
              return getS().element->weights(floatSeqToVector(weights));
            } catch (const std::exception& e) {
              throw Error (e.what ());
            }
          }
      };

      typedef WeighedDistanceServant<POA_hpp::core_idl::WeighedDistance, DistanceStorage<core::WeighedDistance> > WeighedDistance;

      typedef boost::mpl::vector<WeighedDistance, Distance> Distances;
    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // SRC_DISTANCES_HH
