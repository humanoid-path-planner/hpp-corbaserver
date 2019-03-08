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
      struct DistanceStorage {
        core::DevicePtr_t r;
        D d;
        DistanceStorage (const core::DevicePtr_t& _r, const D& _d) : r(_r), d(_d) {}
        operator core::DistancePtr_t () const { return d; }
      };

      template <typename Base, typename _Storage>
      class DistanceServant : public DistanceBase, public virtual Base
      {
        public:
          typedef _Storage Storage;

          DistanceServant (Server* server, const Storage& s) :
            DistanceBase (server), s_ (s) {}

          virtual ~DistanceServant () {}

          CORBA::Double value (const floatSeq& q1, const floatSeq& q2) throw (Error)
          {
            Configuration_t qq1 (floatSeqToConfig(s_.r, q1, true)),
                            qq2 (floatSeqToConfig(s_.r, q2, true));
            return (*s_.d) (qq1,qq2);
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

      typedef DistanceServant<POA_hpp::core_idl::Distance, DistanceStorage<core::DistancePtr_t> > Distance;

      template <typename Base, typename Storage>
      class WeighedDistanceServant : public DistanceServant<Base, Storage>
      {
        protected:
          using DistanceBase::server_;
        public:
          typedef DistanceServant<Base, Storage> Parent;
          using Parent::getS;

          WeighedDistanceServant (Server* server, const Storage& s)
            : Parent(server, s) {}

          ~WeighedDistanceServant () {}

          floatSeq* getWeights () throw (Error)
          {
            return vectorToFloatSeq (getS().d->weights());
          }

          void setWeights (const floatSeq& weights) throw (Error)
          {
            try {
              return getS().d->weights(floatSeqToVector(weights));
            } catch (const std::exception& e) {
              throw Error (e.what ());
            }
          }
      };

      typedef WeighedDistanceServant<POA_hpp::core_idl::WeighedDistance, DistanceStorage<core::WeighedDistancePtr_t> > WeighedDistance;
    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // SRC_DISTANCES_HH
