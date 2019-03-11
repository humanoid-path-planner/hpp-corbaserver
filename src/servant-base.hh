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

# include <boost/mpl/vector.hpp>
# include <boost/mpl/for_each.hpp>

# include <hpp/common-idl.hh>

namespace hpp
{
  namespace corbaServer
  {
#define SERVANT_BASE_TYPEDEFS(Obj)                                             \
    typedef Obj         Object;                                                \
    typedef Obj ## _ptr Object_ptr;

    class AbstractServantKey
    {
      public:
        virtual Server::ServantKey getServantKey () /*const*/ = 0;
    };

    template <typename T> class AbstractServantBase : AbstractServantKey
    {
      public:
        virtual ~AbstractServantBase () {}

        virtual T get() = 0;

        virtual Server::ServantKey getServantKey ()
        {
          return get().get();
        }

      protected:
        AbstractServantBase (Server* server) : server_ (server) {}

        Server* server_;
    };

    /// Abstraction of storage ot HPP class.
    /// Child classed must implement
    /// \code
    /// template <typename T> DistanceStorage<T> cast () const
    /// {
    /// return DistanceStorage<T> (r, HPP_DYNAMIC_PTR_CAST(T, d));
    /// }
    /// \endcode
    template <typename T, typename Base> class AbstractStorage
    {
      public:
        typedef boost::shared_ptr<T> ptr_t;

        ptr_t element;

        AbstractStorage (const ptr_t& _element) : element(_element) {}
        operator boost::shared_ptr<Base> () const { return element; }

        // Mimic boost::shared_ptr<D> interface:
        typedef T element_type;
        operator bool () const { return element; }
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
      Server::ServantKey servantKey = s->getServantKey();
      S* servant = dynamic_cast<S*> (server->getServant (servantKey));
      if (servant != NULL) {
        delete s;
        return servant->_this();
      }

      PortableServer::Servant_var<S> d (s);
      // ObjectId_var object is here to delete the servantId.
      PortableServer::ObjectId_var servantId = server->poa()->activate_object(d);
      (void) servantId;

      server->addServantKeyAndServant (servantKey, d.in());
      return d->_this();
    }

    namespace details
    {
      template <typename T, template<typename> class StorageTpl,
               typename ObjectRef, typename ObjectRef_Helper>
      struct DownCast
      {
        typedef typename ObjectRef::_var_type ObjectRefVar;

        Server* s;
        StorageTpl<T> t;
        ObjectRefVar& servant;
        DownCast (Server* server, const StorageTpl<T>& _t, ObjectRefVar& _servant)
          : s(server), t(_t), servant(_servant) {}
        template<typename Servant> void operator()(boost::type<Servant>)
        {
          // Servant -> Storage to dynamic cast.
          typedef typename Servant::Storage Storage;

          if (!ObjectRef_Helper::is_nil(servant)) return;
          Storage u = storage_cast<typename Storage::element_type>();
          if (u)
            servant = makeServant <typename Servant::Object_ptr> (s, new Servant (s, u));
        }

        /// Enabled only if StorageTpl != boost::shared_ptr
        template <typename U>
        StorageTpl<U> storage_cast(typename boost::enable_if_c<(!boost::is_same<boost::shared_ptr<U>,StorageTpl<U> >::value)>::type* =0) const
        {
          return t.template cast<U>();
        }

        /// Enabled only if StorageTpl == boost::shared_ptr
        template <typename U>
        boost::shared_ptr<U> storage_cast(typename boost::enable_if_c<(boost::is_same<boost::shared_ptr<U>,StorageTpl<U> >::value)>::type* =0) const
        {
          return boost::dynamic_pointer_cast<U>(t);
        }
      };
    }

    /// \tparam ObjectRef_Helper needed for the is_nil static member function.
    template <typename ObjectRef, typename ObjectRef_Helper, typename Types,
      typename T, template<typename> class StorageTpl>
    typename ObjectRef::_var_type makeServantDownCast (Server* server, const StorageTpl<T>& t)
    {
      typedef typename ObjectRef::_var_type ObjectRefVar;
      ObjectRefVar d (ObjectRef_Helper::_nil());

      details::DownCast<T, StorageTpl, ObjectRef, ObjectRef_Helper> downCast (server, t, d);
      boost::mpl::for_each<Types, boost::type<boost::mpl::_> > (downCast);

      return d;
    }
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // SRC_SERVANT_BASE_HH
