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

namespace hpp
{
  namespace corbaServer
  {
    namespace core_idl
    {
      class DistanceBase
      {
        public:
          virtual ~DistanceBase () {}

          virtual core::DistancePtr_t get() = 0;
      };

      template <typename Base>
      class DistanceServant : public DistanceBase, public virtual Base
      {
        public:
          DistanceServant (const core::DevicePtr_t& robot,
              const core::DistancePtr_t& d) : robot_(robot), d_ (d) {}

          virtual ~DistanceServant () {}

          CORBA::Double value (const floatSeq& q1, const floatSeq& q2) throw (Error)
          {
            Configuration_t qq1 (floatSeqToConfig(robot_, q1, true)),
                            qq2 (floatSeqToConfig(robot_, q2, true));
            return (*d_) (qq1,qq2);
          }

          virtual core::DistancePtr_t get ()
          {
            return d_;
          }

        protected:
          core::DevicePtr_t robot_;

        private:
          core::DistancePtr_t d_;
      };

      typedef DistanceServant<POA_hpp::core_idl::Distance> Distance;

      class WeighedDistance : public DistanceServant<POA_hpp::core_idl::WeighedDistance>
      {
        public:
          typedef DistanceServant<POA_hpp::core_idl::WeighedDistance> Base;
          WeighedDistance (const core::DevicePtr_t& robot,
              const core::WeighedDistancePtr_t& d)
            : Base(robot, d), d_ (d) {}

          ~WeighedDistance () {}

          floatSeq* getWeights () throw (Error)
          {
            return vectorToFloatSeq (d_->weights());
          }

          void setWeights (const floatSeq& weights) throw (Error)
          {
            try {
              return d_->weights(floatSeqToVector(weights));
            } catch (const std::exception& e) {
              throw Error (e.what ());
            }
          }

        private:
          core::WeighedDistancePtr_t d_;
      };
    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // SRC_DISTANCES_HH
