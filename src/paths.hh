// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef SRC_PATHS_HH
# define SRC_PATHS_HH
# include <vector>
# include <stdlib.h>

# include "hpp/core/path.hh"

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/conversions.hh>
# include "hpp/core_idl/paths-idl.hh"

# include "servant-base.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace core_idl
    {
      typedef AbstractServantBase<core::PathPtr_t> PathBase;

      template <typename Base, typename _Storage>
      class PathServant : public PathBase, public virtual Base
      {
        public:
          typedef _Storage Storage;

          PathServant (Server* server, const Storage& s) : PathBase (server), s_ (s) {}

          virtual ~PathServant () {}

          CORBA::Long outputSize () throw (Error)
          {
            return (CORBA::Long)s_->outputSize();
          }

          CORBA::Long outputDerivativeSize () throw (Error)
          {
            return (CORBA::Long)s_->outputDerivativeSize();
          }

          CORBA::Double length () throw (Error)
          {
            return s_->length ();
          }

          char* print () throw (Error)
          {
            std::ostringstream oss; oss << *s_;
            std::string res = oss.str();
            return CORBA::string_dup(res.c_str());
          }

          floatSeq* value (CORBA::Double t, CORBA::Boolean& success) throw (Error)
          {
            return vectorToFloatSeq (s_->operator() (t, success));
          }

          floatSeq* derivative (CORBA::Double t, CORBA::Short order) throw (Error)
          {
            vector_t res (s_->outputDerivativeSize());
            s_->derivative (res, t, order);
            return vectorToFloatSeq (res);
          }

          virtual core::PathPtr_t get ()
          {
            return (core::PathPtr_t)s_;
          }

          Storage getS ()
          {
            return s_;
          }

        protected:
          Storage s_;
      };

      typedef PathServant<POA_hpp::core_idl::Path, core::PathPtr_t> Path;

      template <typename Base, typename Storage>
      class PathVectorServant : public PathServant<Base, Storage>
      {
        protected:
          using PathBase::server_;
        public:
          typedef PathServant<Base, Storage> Parent;
          using Parent::getS;

          PathVectorServant (Server* server, const Storage& s) : Parent (server, s) {}

          std::size_t numberPaths () throw (Error)
          {
            return this->getS()->numberPaths();
          }

          hpp::core_idl::Path_ptr pathAtRank (std::size_t rank) throw (Error)
          {
            return server_->template makeServant<hpp::core_idl::Path_ptr> (
                new Path (server_, getS()->pathAtRank(rank))
                );
          }

          void appendPath (hpp::core_idl::Path_ptr path) throw (Error)
          {
            getS()->appendPath(server_->template reference_to_servant<PathBase>(path)->get());
          }

          void concatenate (hpp::core_idl::PathVector_ptr path) throw (Error)
          {
            getS()->appendPath(server_->template reference_to_servant<PathVectorServant>(path)->getS());
          }
      };

      typedef PathVectorServant<POA_hpp::core_idl::PathVector, core::PathVectorPtr_t> PathVector;

    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // SRC_PATHS_HH
