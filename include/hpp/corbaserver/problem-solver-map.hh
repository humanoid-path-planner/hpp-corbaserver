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
# include <boost/thread/mutex.hpp>

# include "hpp/corbaserver/fwd.hh"
# include "hpp/corbaserver/config.hh"

# include "hpp/core/fwd.hh"

namespace hpp
{
  namespace corbaServer
  {
    class HPP_CORBASERVER_DLLAPI ProblemSolverMap {
      public:
        typedef std::map<std::string, core::ProblemSolverPtr_t> ProblemMap_t;
        typedef boost::shared_ptr<ProblemMap_t> ProblemMapPtr_t;
        typedef boost::mutex mutex_t;
        typedef boost::shared_ptr<mutex_t> mutexPtr_t;

        ProblemSolverMap (core::ProblemSolverPtr_t init,
            const std::string& name = "default");

        ProblemSolverMap (const ProblemSolverMap& map);

        core::ProblemSolverPtr_t operator-> ();
        operator core::ProblemSolverPtr_t ();

        core::ProblemSolverPtr_t selected () const;
        core::ProblemSolverPtr_t get (const std::string& name) const;
        void selected (const std::string& name);

        bool has (const std::string& name) const;
        void add (const std::string& name, core::ProblemSolverPtr_t ps);
        void remove (const std::string& name);
        void replaceSelected (core::ProblemSolverPtr_t ps);

        template <typename ReturnType> ReturnType keys () const
        {
          mutex_t::scoped_lock lock (*mutex_);
          ReturnType l;
          for (ProblemMap_t::const_iterator it = map_->begin ();
              it != map_->end (); ++it)
            l.push_back (it->first);
          return l;
        }

        const std::string& selectedName () const
        {
          return selected_;
        }

      private:
        std::string selected_;
        ProblemMapPtr_t map_;
        mutexPtr_t mutex_;
    };
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_PROBLEM_SOLVER_MAP_HH
