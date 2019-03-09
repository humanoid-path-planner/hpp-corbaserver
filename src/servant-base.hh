// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef SRC_SERVANT_BASE_HH
# define SRC_SERVANT_BASE_HH

#include <hpp/common-idl.hh>

namespace hpp
{
  namespace corbaServer
  {
    template <typename T> class AbstractServantBase
    {
      public:
        virtual ~AbstractServantBase () {}

        virtual T get() = 0;

      protected:
        AbstractServantBase (Server* server) : server_ (server) {}

        Server* server_;
    };

    typedef PortableServer::Servant_var<PortableServer::ServantBase> ServantBase_var;

    template <typename S, typename P> PortableServer::Servant_var<S> reference_to_servant (Server* server, const P& p)
    {
      PortableServer::Servant s = server->poa()->reference_to_servant(p);
      if (s == NULL) throw Error ("The servant is not located here");
      return dynamic_cast<S*> (s);
    }

    template <typename T, typename P> AbstractServantBase<T>* reference_to_servant_base (Server* server, const P& p)
    {
      ServantBase_var s = server->poa()->reference_to_servant(p);
      if (s.in() == NULL) throw Error ("The servant is not located here");
      return dynamic_cast<AbstractServantBase<T>*> (s.in());
    }

    template <typename P, typename S> P makeServant (Server* server, S* s)
    {
      PortableServer::Servant_var<S> d (s);
      // ObjectId_var object is here to delete the servantId.
      PortableServer::ObjectId_var servantId = server->poa()->activate_object(d);
      (void) servantId;
      return d->_this();
    }
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // SRC_SERVANT_BASE_HH
