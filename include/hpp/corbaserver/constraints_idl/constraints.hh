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
      class DifferentiableFunctionServant : public ServantBase<constraints::DifferentiableFunction, _Storage>, public virtual _Base
      {
          SERVANT_BASE_TYPEDEFS(hpp::constraints_idl::DifferentiableFunction, constraints::DifferentiableFunction);

        public:
          DifferentiableFunctionServant (Server* server, const Storage& s)
          : _ServantBase (server, s) {}

          virtual ~DifferentiableFunctionServant () {}

          floatSeq* value (const floatSeq& arg) throw (Error)
          {
            return vectorToFloatSeq (
                (*get()) (floatSeqToVector(arg, get()->inputSize())).vector()
                );
          }

          floatSeqSeq* jacobian (const floatSeq& arg) throw (Error)
          {
            matrix_t J (get()->outputDerivativeSize(), get()->inputDerivativeSize());
            get()->jacobian (J, floatSeqToVector(arg));
            return matrixToFloatSeqSeq (J);
          }

          size_type inputSize () throw (Error)
          {
            return get()->inputSize();
          }
          size_type inputDerivativeSize () throw (Error)
          {
            return get()->inputDerivativeSize();
          }
          size_type  outputSize () throw (Error)
          {
            return get()->outputSize ();
          }
          size_type  outputDerivativeSize () throw (Error)
          {
            return get()->outputDerivativeSize ();
          }
          char* name () throw (Error)
          {
            return c_str(get()->name ());
          }

          char* str () throw (Error)
          {
            std::ostringstream oss; oss << *get();
            std::string res = oss.str();
            return CORBA::string_dup(res.c_str());
          }
      };

      typedef DifferentiableFunctionServant<POA_hpp::constraints_idl::DifferentiableFunction, constraints::DifferentiableFunctionPtr_t> DifferentiableFunction;

      template <typename _Base, typename _Storage>
      class ImplicitServant : public ServantBase<constraints::Implicit, _Storage>, public virtual _Base
      {
          SERVANT_BASE_TYPEDEFS(hpp::constraints_idl::Implicit, constraints::Implicit);

        public:
          ImplicitServant (Server* server, const Storage& s)
          : _ServantBase (server, s) {}

          virtual ~ImplicitServant () {}

          hpp::constraints_idl::DifferentiableFunction_ptr function () throw (Error)
          {
            hpp::constraints_idl::DifferentiableFunction_var f =
              makeServantDownCast<DifferentiableFunction> (server_, get()->functionPtr());
            return f._retn();
          }

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
