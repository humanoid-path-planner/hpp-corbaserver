// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#ifndef HPP_CORBASERVER_PROBLEM_SOLVER_MAP_HH
#define HPP_CORBASERVER_PROBLEM_SOLVER_MAP_HH
#include <boost/thread/mutex.hpp>
#include <map>

#include "hpp/corbaserver/config.hh"
#include "hpp/corbaserver/fwd.hh"
#include "hpp/core/fwd.hh"

namespace hpp {
namespace corbaServer {
class HPP_CORBASERVER_DLLAPI ProblemSolverMap {
 public:
  typedef std::map<std::string, core::ProblemSolverPtr_t> ProblemMap_t;
  typedef shared_ptr<ProblemMap_t> ProblemMapPtr_t;
  typedef boost::mutex mutex_t;
  typedef shared_ptr<mutex_t> mutexPtr_t;

  ProblemSolverMap(core::ProblemSolverPtr_t init,
                   const std::string& name = "default");

  ProblemSolverMap(const ProblemSolverMap& map);

  core::ProblemSolverPtr_t operator->();
  operator core::ProblemSolverPtr_t();

  core::ProblemSolverPtr_t selected() const;
  core::ProblemSolverPtr_t get(const std::string& name) const;
  void selected(const std::string& name);

  bool has(const std::string& name) const;
  void add(const std::string& name, core::ProblemSolverPtr_t ps);
  void remove(const std::string& name);
  void replaceSelected(core::ProblemSolverPtr_t ps);

  template <typename ReturnType>
  ReturnType keys() const {
    mutex_t::scoped_lock lock(*mutex_);
    ReturnType l;
    for (ProblemMap_t::const_iterator it = map_->begin(); it != map_->end();
         ++it)
      l.push_back(it->first);
    return l;
  }

  const std::string& selectedName() const { return selected_; }

 private:
  std::string selected_;
  ProblemMapPtr_t map_;
  mutexPtr_t mutex_;
};
}  // end of namespace corbaServer.
}  // end of namespace hpp.

#endif  //! HPP_CORBASERVER_PROBLEM_SOLVER_MAP_HH
