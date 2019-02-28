// Copyright (c) 2016, Joseph Mirabel
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

#ifndef HPP_CORBASERVER_PROBLEM_SOLVER_MAP_HH
# define HPP_CORBASERVER_PROBLEM_SOLVER_MAP_HH
# include <map>

# include "hpp/corbaserver/fwd.hh"
# include "hpp/corbaserver/config.hh"
# include <hpp/corbaserver/problem-solver-map.hh>

# include "hpp/core/fwd.hh"

namespace hpp
{
  namespace corbaServer
  {
    class HPP_CORBASERVER_DLLAPI ProblemSolverMap {
      public:
        typedef std::map<std::string, core::ProblemSolverPtr_t> ProblemMap_t;

        std::string selected_;
        ProblemMap_t map_;

        ProblemSolverMap (core::ProblemSolverPtr_t init,
            const std::string& name = "default") :
          selected_ (name)
          {
            map_[selected_] = init;
          }

        core::ProblemSolverPtr_t operator-> () {
          return selected();
        }
        operator core::ProblemSolverPtr_t () {
          return selected();
        }
        core::ProblemSolverPtr_t selected () {
          return map_[selected_];
        }
        bool has (const std::string& name) const
        {
          // ProblemMap_t::const_iterator it = map_.find (name);
          // return it != map_.end ();
          return map_.end() != map_.find (name);
        }
        template <typename ReturnType> ReturnType keys () const
        {
          ReturnType l;
          for (ProblemMap_t::const_iterator it = map_.begin ();
              it != map_.end (); ++it)
            l.push_back (it->first);
          return l;
        }
    };
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_PROBLEM_SOLVER_MAP_HH
