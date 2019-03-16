// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORE_IDL_PATHS_HH
# define HPP_CORE_IDL_PATHS_HH
# include <vector>
# include <stdlib.h>

# include "hpp/core/path.hh"
# include "hpp/core/path-vector.hh"

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/conversions.hh>
# include "hpp/core_idl/paths-idl.hh"

# include "hpp/corbaserver/servant-base.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace core_idl
    {
      hpp::core_idl::Path_ptr makePathServant (Server* server, const PathPtr_t& path);

      template <typename _Base, typename _Storage>
      class PathServant : public ServantBase<core::Path, _Storage>, public virtual _Base
      {
          SERVANT_BASE_TYPEDEFS(hpp::core_idl::Path, core::Path);

        public:
          PathServant (Server* server, const Storage& s)
          : _ServantBase (server, s) {}

          virtual ~PathServant () {}

          CORBA::Long outputSize () throw (Error)
          {
            return (CORBA::Long)get()->outputSize();
          }

          CORBA::Long outputDerivativeSize () throw (Error)
          {
            return (CORBA::Long)get()->outputDerivativeSize();
          }

          CORBA::Double length () throw (Error)
          {
            return get()->length ();
          }

          char* str () throw (Error)
          {
            std::ostringstream oss; oss << *get();
            std::string res = oss.str();
            return CORBA::string_dup(res.c_str());
          }

          floatSeq* value (CORBA::Double t, CORBA::Boolean& success) throw (Error)
          {
            return vectorToFloatSeq (get()->operator() (t, success));
          }

          floatSeq* derivative (CORBA::Double t, CORBA::Short order) throw (Error)
          {
            vector_t res (get()->outputDerivativeSize());
            get()->derivative (res, t, order);
            return vectorToFloatSeq (res);
          }

          hpp::core_idl::PathVector_ptr asVector () throw (Error);
      };

      typedef PathServant<POA_hpp::core_idl::Path, core::PathPtr_t> Path;

      template <typename _Base, typename _Storage>
      class PathVectorServant : public PathServant<_Base, _Storage>
      {
          SERVANT_BASE_TYPEDEFS(hpp::core_idl::PathVector, core::Path);

        public:
          typedef PathServant<Base, Storage> Parent;

          PathVectorServant (Server* server, const Storage& s) : Parent (server, s) {}

          std::size_t numberPaths () throw (Error)
          {
            return getT()->numberPaths();
          }

          hpp::core_idl::Path_ptr pathAtRank (std::size_t rank) throw (Error)
          {
            return makePathServant (server_, getT()->pathAtRank(rank));
          }

          void appendPath (hpp::core_idl::Path_ptr path) throw (Error)
          {
            getT()->appendPath(reference_to_servant_base<core::Path>(server_, path)->get());
          }

          void concatenate (hpp::core_idl::PathVector_ptr path) throw (Error)
          {
            getT()->appendPath(reference_to_servant<PathVectorServant>(server_, path)->getT());
          }
      };

      typedef PathVectorServant<POA_hpp::core_idl::PathVector, core::PathVectorPtr_t> PathVector;


      template <typename _Base, typename _Storage>
      hpp::core_idl::PathVector_ptr PathServant<_Base,_Storage>::asVector () throw (Error)
      {
        PathPtr_t p = get();
        PathVectorPtr_t pv =
          core::PathVector::create (p->outputSize(), p->outputDerivativeSize());
        pv->appendPath (p);

        return makeServant<hpp::core_idl::PathVector_ptr>
          (server_, new PathVector (server_, pv));
      }

    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // HPP_CORE_IDL_PATHS_HH
