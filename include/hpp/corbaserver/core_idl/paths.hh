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

#ifndef HPP_CORE_IDL_PATHS_HH
#define HPP_CORE_IDL_PATHS_HH
#include <stdlib.h>

#include <hpp/corbaserver/conversions.hh>
#include <hpp/corbaserver/fwd.hh>
#include <vector>

#include "hpp/corbaserver/servant-base.hh"
#include "hpp/core/path-vector.hh"
#include "hpp/core/path.hh"
#include "hpp/core_idl/paths-idl.hh"

namespace hpp {
namespace corbaServer {
namespace core_idl {
hpp::core_idl::Path_ptr makePathServant(Server* server, const PathPtr_t& path);

template <typename _Base, typename _Storage>
class PathServant : public ServantBase<core::Path, _Storage>,
                    public virtual _Base {
  SERVANT_BASE_TYPEDEFS(hpp::core_idl::Path, core::Path);

 public:
  PathServant(Server* server, const Storage& s) : _ServantBase(server, s) {}

  virtual ~PathServant() {}

  size_type outputSize() { return get()->outputSize(); }

  size_type outputDerivativeSize() { return get()->outputDerivativeSize(); }

  value_type length() { return get()->length(); }

  char* str() {
    std::ostringstream oss;
    oss << *get();
    std::string res = oss.str();
    return CORBA::string_dup(res.c_str());
  }

  floatSeq* value(value_type t, CORBA::Boolean& success) {
    return vectorToFloatSeq(get()->operator()(t, success));
  }

  floatSeq* derivative(value_type t, CORBA::Short order) {
    vector_t res(get()->outputDerivativeSize());
    get()->derivative(res, t, order);
    return vectorToFloatSeq(res);
  }

  hpp::core_idl::Path_ptr extract(value_type tmin, value_type tmax) {
    return makePathServant(server_,
                           get()->extract(core::interval_t(tmin, tmax)));
  }

  hpp::core_idl::PathVector_ptr asVector();
};

typedef PathServant<POA_hpp::core_idl::Path, core::PathPtr_t> Path;

template <typename _Base, typename _Storage>
class PathVectorServant : public PathServant<_Base, _Storage> {
  SERVANT_BASE_TYPEDEFS(hpp::core_idl::PathVector, core::Path);

 public:
  typedef PathServant<Base, Storage> Parent;

  PathVectorServant(Server* server, const Storage& s) : Parent(server, s) {}

  std::size_t numberPaths() { return getT()->numberPaths(); }

  hpp::core_idl::Path_ptr pathAtRank(std::size_t rank) {
    return makePathServant(server_, getT()->pathAtRank(rank));
  }

  void appendPath(hpp::core_idl::Path_ptr path) {
    getT()->appendPath(
        reference_to_servant_base<core::Path>(server_, path)->get());
  }

  void concatenate(hpp::core_idl::PathVector_ptr path) {
    getT()->concatenate(
        reference_to_servant<PathVectorServant>(server_, path)->getT());
  }
};

typedef PathVectorServant<POA_hpp::core_idl::PathVector, core::PathVectorPtr_t>
    PathVector;

template <typename _Base, typename _Storage>
hpp::core_idl::PathVector_ptr PathServant<_Base, _Storage>::asVector() {
  PathPtr_t p = get();
  PathVectorPtr_t pv =
      core::PathVector::create(p->outputSize(), p->outputDerivativeSize());
  pv->appendPath(p);

  return makeServant<hpp::core_idl::PathVector_ptr>(
      server_, new PathVector(server_, pv));
}

}  // namespace core_idl
}  // end of namespace corbaServer.
}  // end of namespace hpp.

#endif  // HPP_CORE_IDL_PATHS_HH
