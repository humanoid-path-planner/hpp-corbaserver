// Copyright (c) 2019, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-corbaserver.
// hpp-corbaserver is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-corbaserver is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-corbaserver. If not, see <http://www.gnu.org/licenses/>.

# include <hpp/corbaserver/problem-solver-map.hh>

# include <hpp/core/problem-solver.hh>

namespace hpp
{
  namespace corbaServer
  {
    ProblemSolverMap::ProblemSolverMap (core::ProblemSolverPtr_t init,
        const std::string& name) :
      selected_ (name),
      map_ (new ProblemMap_t),
      mutex_ (new mutex_t)
    {
      add (selected_, init);
    }

    ProblemSolverMap::ProblemSolverMap (const ProblemSolverMap& other)
      : selected_ (other.selected_),
      map_ (other.map_),
      mutex_ (other.mutex_)
    {
    }

    core::ProblemSolverPtr_t ProblemSolverMap::operator-> ()
    {
      return selected();
    }

    ProblemSolverMap::operator core::ProblemSolverPtr_t ()
    {
      return selected();
    }

    core::ProblemSolverPtr_t ProblemSolverMap::selected () const
    {
      return get (selected_);
    }

    core::ProblemSolverPtr_t ProblemSolverMap::get (const std::string& name) const
    {
      mutex_t::scoped_lock lock (*mutex_);
      ProblemMap_t::const_iterator it = map_->find (name);
      if (it == map_->end ())
        throw std::invalid_argument ("Could not find ProblemSolver named " + name );
      return it->second;
    }

    bool ProblemSolverMap::has (const std::string& name) const
    {
      mutex_t::scoped_lock lock (*mutex_);
      // ProblemMap_t::const_iterator it = map_.find (name);
      // return it != map_.end ();
      return map_->end() != map_->find (name);
    }

    void ProblemSolverMap::selected (const std::string& name)
    {
      if (!has (name))
        throw std::invalid_argument ("Could not find ProblemSolver named " + name);
      mutex_t::scoped_lock lock (*mutex_);
      selected_ = name;
    }

    void ProblemSolverMap::add (const std::string& name, core::ProblemSolverPtr_t ps)
    {
      if (has (name))
        throw std::invalid_argument ("ProblemSolver named " + name + " already exists");
      mutex_t::scoped_lock lock (*mutex_);
      map_->insert (std::make_pair(name,ps));
    }

    void ProblemSolverMap::remove (const std::string& name)
    {
      mutex_t::scoped_lock lock (*mutex_);
      map_->erase (name);
    }

    void ProblemSolverMap::replaceSelected (core::ProblemSolverPtr_t ps)
    {
      mutex_t::scoped_lock lock (*mutex_);
      ProblemMap_t::iterator it = map_->find (selected_);
      assert (it != map_->end());
      delete it->second;
      it->second = ps;
    }
  } // end of namespace corbaServer.
} // end of namespace hpp.
