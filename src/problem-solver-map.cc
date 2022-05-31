// Copyright (c) 2019, Joseph Mirabel
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

#include <hpp/corbaserver/problem-solver-map.hh>
#include <hpp/core/problem-solver.hh>

namespace hpp {
namespace corbaServer {
ProblemSolverMap::ProblemSolverMap(core::ProblemSolverPtr_t init,
                                   const std::string& name)
    : selected_(name), map_(new ProblemMap_t), mutex_(new mutex_t) {
  add(selected_, init);
}

ProblemSolverMap::ProblemSolverMap(const ProblemSolverMap& other)
    : selected_(other.selected_), map_(other.map_), mutex_(other.mutex_) {}

core::ProblemSolverPtr_t ProblemSolverMap::operator->() { return selected(); }

ProblemSolverMap::operator core::ProblemSolverPtr_t() { return selected(); }

core::ProblemSolverPtr_t ProblemSolverMap::selected() const {
  return get(selected_);
}

core::ProblemSolverPtr_t ProblemSolverMap::get(const std::string& name) const {
  mutex_t::scoped_lock lock(*mutex_);
  ProblemMap_t::const_iterator it = map_->find(name);
  if (it == map_->end())
    throw std::invalid_argument("Could not find ProblemSolver named " + name);
  return it->second;
}

bool ProblemSolverMap::has(const std::string& name) const {
  mutex_t::scoped_lock lock(*mutex_);
  // ProblemMap_t::const_iterator it = map_.find (name);
  // return it != map_.end ();
  return map_->end() != map_->find(name);
}

void ProblemSolverMap::selected(const std::string& name) {
  if (!has(name))
    throw std::invalid_argument("Could not find ProblemSolver named " + name);
  mutex_t::scoped_lock lock(*mutex_);
  selected_ = name;
}

void ProblemSolverMap::add(const std::string& name,
                           core::ProblemSolverPtr_t ps) {
  if (has(name))
    throw std::invalid_argument("ProblemSolver named " + name +
                                " already exists");
  mutex_t::scoped_lock lock(*mutex_);
  map_->insert(std::make_pair(name, ps));
}

void ProblemSolverMap::remove(const std::string& name) {
  mutex_t::scoped_lock lock(*mutex_);
  map_->erase(name);
}

void ProblemSolverMap::replaceSelected(core::ProblemSolverPtr_t ps) {
  mutex_t::scoped_lock lock(*mutex_);
  ProblemMap_t::iterator it = map_->find(selected_);
  assert(it != map_->end());
  delete it->second;
  it->second = ps;
}
}  // end of namespace corbaServer.
}  // end of namespace hpp.
