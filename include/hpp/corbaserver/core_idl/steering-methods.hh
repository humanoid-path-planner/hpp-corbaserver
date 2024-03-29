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

#ifndef HPP_CORE_IDL_STEERING_METHODS_HH
#define HPP_CORE_IDL_STEERING_METHODS_HH

#include <stdlib.h>

#include <hpp/corbaserver/conversions.hh>
#include <hpp/corbaserver/core_idl/paths.hh>
#include <hpp/corbaserver/fwd.hh>
#include <vector>

#include "hpp/corbaserver/servant-base.hh"
#include "hpp/core/steering-method.hh"
#include "hpp/core_idl/steering_methods-idl.hh"

namespace hpp {
namespace corbaServer {
namespace core_idl {
template <typename D>
class SteeringMethodStorage : public AbstractStorage<D, core::SteeringMethod> {
 public:
  typedef AbstractStorage<D, core::SteeringMethod> parent_t;
  using parent_t::element;
  using typename parent_t::ptr_t;

  core::DevicePtr_t r;
  SteeringMethodStorage(const core::DevicePtr_t& _r, const ptr_t& _d)
      : parent_t(_d), r(_r) {}

  template <typename T>
  SteeringMethodStorage<T> cast() const {
    return SteeringMethodStorage<T>(r, HPP_DYNAMIC_PTR_CAST(T, element.lock()));
  }
};

template <typename _Base, typename _Storage>
class SteeringMethodServant
    : public ServantBase<core::SteeringMethod, _Storage>,
      public virtual _Base {
  SERVANT_BASE_TYPEDEFS(hpp::core_idl::SteeringMethod, core::SteeringMethod);

 public:
  SteeringMethodServant(Server* server, const Storage& s)
      : _ServantBase(server, s) {}

  virtual ~SteeringMethodServant() {}

  hpp::core_idl::Path_ptr call(const floatSeq& q1, const floatSeq& q2) {
    Configuration_t qq1(floatSeqToConfig(getS().r, q1, true)),
        qq2(floatSeqToConfig(getS().r, q2, true));
    hpp::core_idl::Path_var p =
        makeServantDownCast<core_idl::Path>(server_, (*get())(qq1, qq2));
    return p._retn();
  }
};

typedef SteeringMethodServant<POA_hpp::core_idl::SteeringMethod,
                              SteeringMethodStorage<core::SteeringMethod> >
    SteeringMethod;
}  // namespace core_idl
}  // end of namespace corbaServer.
}  // end of namespace hpp.

#endif  // HPP_CORE_IDL_STEERING_METHODS_HH
