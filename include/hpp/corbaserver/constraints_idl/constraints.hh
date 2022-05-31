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

#ifndef HPP_CONSTRAINTS_IDL_CONSTRAINTS_HH
#define HPP_CONSTRAINTS_IDL_CONSTRAINTS_HH

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints_idl/constraints-idl.hh>
#include <hpp/corbaserver/conversions.hh>
#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/servant-base.hh>

namespace hpp {
namespace corbaServer {
namespace constraints_idl {
template <typename _Base, typename _Storage>
class DifferentiableFunctionServant
    : public ServantBase<constraints::DifferentiableFunction, _Storage>,
      public virtual _Base {
  SERVANT_BASE_TYPEDEFS(hpp::constraints_idl::DifferentiableFunction,
                        constraints::DifferentiableFunction);

 public:
  DifferentiableFunctionServant(Server* server, const Storage& s)
      : _ServantBase(server, s) {}

  virtual ~DifferentiableFunctionServant() {}

  floatSeq* value(const floatSeq& arg) {
    return vectorToFloatSeq(
        (*get())(floatSeqToVector(arg, get()->inputSize())).vector());
  }

  floatSeqSeq* jacobian(const floatSeq& arg) {
    matrix_t J(get()->outputDerivativeSize(), get()->inputDerivativeSize());
    get()->jacobian(J, floatSeqToVector(arg));
    return matrixToFloatSeqSeq(J);
  }

  size_type inputSize() { return get()->inputSize(); }
  size_type inputDerivativeSize() { return get()->inputDerivativeSize(); }
  size_type outputSize() { return get()->outputSize(); }
  size_type outputDerivativeSize() { return get()->outputDerivativeSize(); }
  char* name() { return c_str(get()->name()); }

  char* str() {
    std::ostringstream oss;
    oss << *get();
    std::string res = oss.str();
    return CORBA::string_dup(res.c_str());
  }
};

typedef DifferentiableFunctionServant<
    POA_hpp::constraints_idl::DifferentiableFunction,
    constraints::DifferentiableFunctionPtr_t>
    DifferentiableFunction;

template <typename _Base, typename _Storage>
class ImplicitServant : public ServantBase<constraints::Implicit, _Storage>,
                        public virtual _Base {
  SERVANT_BASE_TYPEDEFS(hpp::constraints_idl::Implicit, constraints::Implicit);

 public:
  ImplicitServant(Server* server, const Storage& s) : _ServantBase(server, s) {}

  virtual ~ImplicitServant() {}

  hpp::constraints_idl::DifferentiableFunction_ptr function() {
    hpp::constraints_idl::DifferentiableFunction_var f =
        makeServantDownCast<DifferentiableFunction>(server_,
                                                    get()->functionPtr());
    return f._retn();
  }

  void setRightHandSideFromConfig(const floatSeq& config) {
    get()->rightHandSideFromConfig(
        floatSeqToVector(config, get()->function().inputSize()));
  }

  void setRightHandSide(const floatSeq& rhs) {
    get()->rightHandSide(floatSeqToVector(rhs, get()->rhsSize()));
  }

  floatSeq* getRightHandSide() {
    return vectorToFloatSeq(get()->rightHandSide());
  }

  hpp::size_type rhsSize() { return get()->rhsSize(); }

  CORBA::Boolean constantRightHandSide() {
    return get()->constantRightHandSide();
  }

  floatSeq* rightHandSideAt(value_type s) {
    return vectorToFloatSeq(get()->rightHandSideAt(s));
  }
};

typedef ImplicitServant<POA_hpp::constraints_idl::Implicit,
                        constraints::ImplicitPtr_t>
    Implicit;
}  // namespace constraints_idl
}  // end of namespace corbaServer.
}  // end of namespace hpp.

#endif  // HPP_CONSTRAINTS_IDL_CONSTRAINTS_HH
