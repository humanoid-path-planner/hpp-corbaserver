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

# include <boost/type.hpp>
# include <boost/mpl/vector.hpp>
# include <boost/mpl/for_each.hpp>

# include <hpp/common-idl.hh>
# include <hpp/corbaserver/server.hh>

namespace hpp
{
  namespace corbaServer
  {
    /// \addtogroup hpp_corbaserver_extend_bindings Extending the bindings
    /// Classes and functions related to the addition of HPP class bindings.
    ///
    /// Example usage can be found in classes
    /// hpp::corbaServer::core_idl::Path,
    /// hpp::corbaServer::core_idl::PathVector,
    /// hpp::corbaServer::core_idl::Distance
    ///
    /// \par Steps for the topmost class of an hierarchy, called ClassA:
    /// - optionally write a storage class (or use boost::shared_ptr<ClassA>)
    /// - create a `template <typename _Base, typename _Storage> ClassAServant `
    ///   that inherits
    ///   - \c _Base, this will be an IDL class that inherits from IDLClassA.
    ///   - \c ServantBase<boost::shared_ptr<ClassA>, _Storage >
    /// - Add `SERVANT_BASE_TYPEDEFS(ClassAServant, boost::shared_ptr<ClassA>);`
    /// - implement the functions of IDLClassA
    /// - add after your class `typedef ClassAServant<IDLClassA, Storage> ClassA`.
    ///
    /// \par Steps for a derived class of an hierarchy, called ClassB:
    /// - optionally write a storage class (or use boost::shared_ptr<ClassB>)
    /// - create a ` template <typename _Base, typename _Storage> ClassBServant`
    ///   that inherits
    ///   - \c ClassAServant<_Base, Storage>
    /// - implement the functions of IDLClassB
    /// - add after your class `typedef ClassBServant<IDLClassB, Storage> ClassB`.
    ///
    /// \par about the storage class
    /// See details section of AbstractStorage.
    ///
    /// \{

#define SERVANT_BASE_TYPEDEFS(idlObj, hppObj)                                  \
    protected:                                                                 \
      using AbstractServantBase<hppObj>::server_;                              \
    public:                                                                    \
      typedef _Base    Base;                                                   \
      typedef _Storage Storage;                                                \
      typedef idlObj         Object;                                           \
      typedef idlObj ## _ptr Object_ptr;                                       \
      typedef idlObj ## _var Object_var;                                       \
      typedef ServantBase<hppObj, _Storage> _ServantBase;                      \
      using _ServantBase::get;                                                 \
      using _ServantBase::getS

    /// Abstract class used to avoid duplication of the servants.
    class AbstractServantKey
    {
      public:
        virtual Server::ServantKey getServantKey () const = 0;
    };

    /// Base class for classes which provides bindings for HPP classes.
    /// Example usage are hpp::corbaServer::core_idl::Path,
    /// hpp::corbaServer::core_idl::PathVector,
    /// hpp::corbaServer::core_idl::Distance
    template <typename T> class AbstractServantBase : AbstractServantKey
    {
      public:
        virtual ~AbstractServantBase () {}

        virtual T get() const = 0;

        virtual Server::ServantKey getServantKey () const
        {
          return get().get();
        }

      protected:
        AbstractServantBase (Server* server) : server_ (server) {}

        Server* server_;
    };

    template <typename T, typename _Storage> class ServantBase : public AbstractServantBase<T>
    {
      public:
        typedef _Storage Storage;
        virtual ~ServantBase () {}

        virtual T get () const
        {
          return (T)s;
        }

        const Storage& getS () const
        {
          return s;
        }

      protected:
        ServantBase (Server* server, const Storage& _s)
          : AbstractServantBase<T> (server), s(_s) {}

        Storage s;
    };

    /// Abstraction of storage ot HPP class.
    ///
    /// In most cases, it is sufficient to use boost::shared_ptr<Class>. However,
    /// there are some cases where you need more information in the AbstractServantBase
    /// derived class. For instance, if you want to check the inputs (which is
    /// recommended), you may need to store the robot. See
    /// hpp::corbaServer::core_idl::Distance for an example.
    ///
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

    /// Create and activate a omniORB servant.
    ///
    /// \tparam P An CORBA::Object_ptr (typically S::Object_ptr)
    /// \tparam S A class deriving from AbstractServantKey and PortableServer::ServantBase
    ///           (or a class generated by omniidl in the namespace POA_*).
    /// \return a reference to the OmniORB servant.
    /// \note if a OmniORB servant serving s exists, no servant is created and
    ///       a reference to the existing servant is returned.
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

    /// \cond
    namespace details
    {
      /// Enabled only if StorageTpl != boost::shared_ptr
      template <typename U, typename V, template<typename> class StorageTpl>
      struct storage_cast_impl {
        static StorageTpl<U> run (const StorageTpl<V>& o) {
          return o.template cast<U>();
        }
      };

      /// Enabled only if StorageTpl == boost::shared_ptr
      template <typename U, typename V>
      struct storage_cast_impl<U, V, boost::shared_ptr> {
        static boost::shared_ptr<U> run (const boost::shared_ptr<V>& o) {
          return boost::dynamic_pointer_cast<U>(o);
        }
      };
    }
    /// \endcond

    template <typename U, typename V, template<typename> class StorageTpl>
    static StorageTpl<U> storage_cast(const StorageTpl<V>& o)
    {
      return details::storage_cast_impl<U, V, StorageTpl>::run (o);
    }

    template <typename ServantBaseType>
    class ServantFactoryBase
    {
      public:
        typedef typename ServantBaseType::Storage Storage;
        typedef typename ServantBaseType::Object_var Object_var;

        ServantFactoryBase (const size_type& depth) : depth_ (depth) {}

        virtual Object_var servant (Server* server, const Storage& obj) = 0;

        /// Get the depth in the hierarchy tree.
        /// This allows to ensure that child classes are always handled before
        /// parent classes.
        size_type depth () const
        {
          return depth_;
        }
      private:
        size_type depth_;
    };

    template <typename ServantBaseType, typename ServantType>
    struct ServantFactory : ServantFactoryBase<ServantBaseType>
    {
      typedef typename ServantBaseType::Object_var Object_var;
      typedef typename ServantBaseType::Storage StorageBase;

      ServantFactory (const size_type& depth) :
        ServantFactoryBase<ServantBaseType> (depth) {}

      virtual Object_var servant (Server* s, const StorageBase& o)
      {
        typedef typename ServantType::Storage Storage;
        Storage u = storage_cast<typename Storage::element_type>(o);
        Object_var ret;
        if (u) ret = makeServant <Object_var> (s, new ServantType (s, u));
        return ret;
      }
    };

    template <typename ServantBaseType>
    std::vector< ServantFactoryBase<ServantBaseType>* >& objectDowncasts ();

    template <typename ServantBaseType>
    void addDowncastObjects (ServantFactoryBase<ServantBaseType>* const object)
    {
      typedef std::vector< ServantFactoryBase<ServantBaseType>* > vector_t;
      typedef typename vector_t::iterator iterator;

      vector_t& vec = objectDowncasts<ServantBaseType>();
      size_type d = object->depth();
      for (iterator _obj = vec.begin(); _obj != vec.end(); ++_obj) {
        if (d >= (*_obj)->depth()) {
          vec.insert(_obj, object);
          return;
        }
      }
      vec.push_back(object);
    }

    /// \cond
    namespace details
    {
      template <typename T, template<typename> class StorageTpl,
               typename ObjectRef>
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

          if (!CORBA::Object_Helper::is_nil(servant)) return;
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
    /// \endcond

    /// Create and activate a omniORB servant with class downcasting.
    /// This automatically downcast to the leftmost type in \c Types.
    /// \tparam ObjectRef the return type.
    /// \tparam Types a boost::mpl::vector of types to check. The order matters.
    ///               Put the base classes **after** their children.
    /// \tparam T the HPP class
    /// \tparam StorageTpl either boost::shared_ptr or a class inheriting from
    ///                    AbstractStorage
    template <typename ObjectRef, typename Types,
      typename T, template<typename> class StorageTpl>
    typename ObjectRef::_var_type makeServantDownCast (Server* server, const StorageTpl<T>& t)
    {
      typedef typename ObjectRef::_var_type ObjectRefVar;
      ObjectRefVar d;
      assert (CORBA::Object_Helper::is_nil(d.in()));

      details::DownCast<T, StorageTpl, ObjectRef> downCast (server, t, d);
      boost::mpl::for_each<Types, boost::type<boost::mpl::_> > (downCast);

      return d;
    }

    template <typename ServantBaseType, typename Storage>
    typename ServantBaseType::Object_var makeServantDownCast2 (Server* server, const Storage& t)
    {
      typedef typename ServantBaseType::Object_var Object_var;
      Object_var servant;
      assert (CORBA::Object_Helper::is_nil(d.in()));

      typedef std::vector< ServantFactoryBase<ServantBaseType>* > vector_t;
      typedef typename vector_t::iterator iterator;

      vector_t& vec = objectDowncasts<ServantBaseType>();
      for (iterator _obj = vec.begin(); _obj != vec.end(); ++_obj) {
        servant = (*_obj)->servant(server, t);
        if (!CORBA::Object_Helper::is_nil(servant.in())) break;
      }

      return servant;
    }

    /// \}
  } // end of namespace corbaServer.
} // end of namespace hpp.

#define HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(ServantType, BaseServantType, depth) \
  struct HPP_CORE_DLLAPI __InitializerClass_##ServantType {                    \
    __InitializerClass_##ServantType () {                                      \
      ::hpp::corbaServer::addDowncastObjects<BaseServantType> \
      ( new ::hpp::corbaServer::ServantFactory<BaseServantType, ServantType> (depth)); \
    }                                                                          \
  };                                                                           \
  extern "C" {                                                                 \
    __InitializerClass_##ServantType __instance_##ServantType;                 \
  }

#endif // SRC_SERVANT_BASE_HH
