// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CONSTRAINTS_IDL_CONSTRAINTS_HH
# define HPP_CONSTRAINTS_IDL_CONSTRAINTS_HH

# include <hpp/constraints/differentiable-function.hh>
# include <hpp/constraints/implicit.hh>

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/conversions.hh>
# include <hpp/constraints_idl/constraints-idl.hh>

# include <hpp/corbaserver/servant-base.hh>

namespace hpp
{
  namespace corbaServer
  {
    namespace constraints_idl
    {
      template <typename _Base, typename _Storage>
      class ImplicitServant : public ServantBase<constraints::Implicit, _Storage>, public virtual _Base
      {
          SERVANT_BASE_TYPEDEFS(hpp::constraints_idl::Implicit, constraints::Implicit);

        public:
          ImplicitServant (Server* server, const Storage& s)
          : _ServantBase (server, s) {}

          virtual ~ImplicitServant () {}

          void setRightHandSideFromConfig (const floatSeq& config) throw (Error)
          {
            get()->rightHandSideFromConfig (
                floatSeqToVector(config, get()->function().inputSize()));
          }

          void setRightHandSide (const floatSeq& rhs) throw (Error)
          {
            get()->rightHandSide (floatSeqToVector(rhs, get()->rhsSize()));
          }

          floatSeq* getRightHandSide () throw (Error)
          {
            return vectorToFloatSeq (get()->rightHandSide());
          }

          hpp::size_type rhsSize () throw (Error)
          {
            return get()->rhsSize();
          }

          CORBA::Boolean constantRightHandSide () throw (Error)
          {
            return get()->constantRightHandSide();
          }

          floatSeq* rightHandSideAt (value_type s) throw (Error)
          {
            return vectorToFloatSeq (get()->rightHandSideAt(s));
          }
      };

      typedef ImplicitServant<POA_hpp::constraints_idl::Implicit, constraints::ImplicitPtr_t> Implicit;
    } // end of namespace constraints.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // HPP_CONSTRAINTS_IDL_CONSTRAINTS_HH
