// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
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
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_SERVANT_BASE_HH
#define HPP_CORBASERVER_SERVANT_BASE_HH

#include <hpp/common-idl.hh>
#include <hpp/corbaserver/server.hh>

namespace hpp {
namespace corbaServer {
/// \addtogroup hpp_corbaserver_extend_bindings Extending the bindings
/// Classes and functions related to the addition of HPP class bindings.
///
/// Example usage can be found in classes
/// hpp::corbaServer::core_idl::Path,
/// hpp::corbaServer::core_idl::PathVector,
/// hpp::corbaServer::core_idl::Distance
///
/// \par Steps for the topmost class of an hierarchy, called ClassA:
/// - optionally write a storage class (or use shared_ptr<ClassA>)
/// - create a `template <typename _Base, typename _Storage> ClassAServant `
///   that inherits
///   - \c _Base, this will be an IDL class that inherits from IDLClassA.
///   - \c ServantBase<shared_ptr<ClassA>, _Storage >
/// - Add `SERVANT_BASE_TYPEDEFS(ClassAServant, shared_ptr<ClassA>);`
/// - implement the functions of IDLClassA
/// - add after your class `typedef ClassAServant<IDLClassA, Storage> ClassA`.
///
/// \par Steps for a derived class of an hierarchy, called ClassB:
/// - optionally write a storage class (or use shared_ptr<ClassB>)
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

#define SERVANT_BASE_TYPEDEFS(idlObj, hppObj)                             \
 protected:                                                               \
  using ::hpp::corbaServer::AbstractServantBase<hppObj>::server_;         \
                                                                          \
 public:                                                                  \
  typedef _Base Base;                                                     \
  typedef _Storage Storage;                                               \
  typedef idlObj Object;                                                  \
  typedef idlObj##_ptr Object_ptr;                                        \
  typedef idlObj##_var Object_var;                                        \
  typedef ::hpp::corbaServer::ServantBase<hppObj, _Storage> _ServantBase; \
  using _ServantBase::get;                                                \
  using _ServantBase::getT;                                               \
  using _ServantBase::getS

template <typename T>
struct hpp_traits {};

/// Abstract class used to avoid duplication of the servants.
class AbstractServantKey {
 public:
  virtual Server::ServantKey getServantKey() const = 0;

  virtual bool expired() const = 0;
};

/// Base class for classes which provides bindings for HPP classes.
/// Example usage are hpp::corbaServer::core_idl::Path,
/// hpp::corbaServer::core_idl::PathVector,
/// hpp::corbaServer::core_idl::Distance
template <typename T>
class AbstractServantBase : AbstractServantKey {
 public:
  virtual ~AbstractServantBase() {}

  typedef shared_ptr<T> TShPtr_t;
  typedef weak_ptr<T> TWkPtr_t;

  virtual TShPtr_t get() const = 0;

  virtual Server::ServantKey getServantKey() const { return get().get(); }

 protected:
  AbstractServantBase(Server* server) : server_(server) {}

  Server* server_;
};

template <typename T, typename _Storage>
class ServantBase : public AbstractServantBase<T> {
 public:
  typedef _Storage Storage;
  using typename AbstractServantBase<T>::TShPtr_t;
  using typename AbstractServantBase<T>::TWkPtr_t;
  typedef weak_ptr<typename Storage::element_type> StorageElementWkPtr_t;
  typedef shared_ptr<typename Storage::element_type> StorageElementShPtr_t;

  virtual ~ServantBase() {}

  virtual TShPtr_t get() const {
    TWkPtr_t wk((StorageElementWkPtr_t)wrappedObject_);
    if (wk.expired()) {
      // Object expired. Remove the servant and throw.
      objectExpired();
    }
    return wk.lock();
  }

  bool expired() const final {
    TWkPtr_t wk((StorageElementWkPtr_t)wrappedObject_);
    return wk.expired();
  }

  StorageElementShPtr_t getT() const {
    StorageElementWkPtr_t wk((StorageElementWkPtr_t)wrappedObject_);
    if (wk.expired()) {
      // Object expired. Remove the servant and throw.
      objectExpired();
    }
    return wk.lock();
  }

  const Storage& getS() const { return wrappedObject_; }

  /// Set to true if the servant should take ownership of this object.
  void persistantStorage(bool persistant) {
    if (persistant)
      p_ = get();
    else
      p_.reset();
  }

  /// See persistantStorage(bool)
  bool persistantStorage() const { return p_; }

  void deleteThis() {
    persistantStorage(false);

    // Object expired. Try to remove the server.
    PortableServer::Servant servant =
        dynamic_cast<PortableServer::Servant>(this);
    if (servant == NULL)
      throw Error("The object was deleted. I could not delete the servant.");
    this->server_->removeServant(servant);
    // Deactivate object
    PortableServer::ObjectId_var objectId =
        this->server_->poa()->servant_to_id(servant);
    this->server_->poa()->deactivate_object(objectId.in());
  }

  bool deleteIfExpired() {
    if (expired()) {
      deleteThis();
      return true;
    }
    return false;
  }

 protected:
  ServantBase(Server* server, const Storage& _s)
      : AbstractServantBase<T>(server), wrappedObject_(_s) {
    persistantStorage(true);
  }

  Storage wrappedObject_;

 private:
  void objectExpired() const {
    // Object expired. Try to remove the server.
    PortableServer::Servant servant =
        dynamic_cast<PortableServer::Servant>(const_cast<ServantBase*>(this));
    if (servant == NULL)
      throw Error("The object was deleted. I could not delete the servant.");
    this->server_->removeServant(servant);
    // Deactivate object
    PortableServer::ObjectId_var objectId =
        this->server_->poa()->servant_to_id(servant);
    this->server_->poa()->deactivate_object(objectId.in());
    throw Error("The object has been deleted. I delete the servant.");
  }

  TShPtr_t p_;
};

/// Abstraction of storage ot HPP class.
///
/// In most cases, it is sufficient to use shared_ptr<Class>. However,
/// there are some cases where you need more information in the
/// AbstractServantBase derived class. For instance, if you want to check the
/// inputs (which is recommended), you may need to store the robot. See
/// hpp::corbaServer::core_idl::Distance for an example.
///
/// Child classed must implement
/// \code
/// template <typename T> DistanceStorage<T> cast () const
/// {
/// return DistanceStorage<T> (r, HPP_DYNAMIC_PTR_CAST(T, d));
/// }
/// \endcode
template <typename T, typename Base>
class AbstractStorage {
 public:
  typedef weak_ptr<T> ptr_t;

  ptr_t element;

  AbstractStorage(const ptr_t& _element) : element(_element) {}
  operator shared_ptr<T>() const { return element.lock(); }
  operator weak_ptr<T>() const { return element; }
  long use_count() const { return element.use_count(); }

  // Mimic shared_ptr<D> interface:
  typedef T element_type;
  operator bool() const { return use_count() > 0; }
};

typedef PortableServer::Servant_var<PortableServer::ServantBase>
    ServantBase_var;

template <typename S, typename P>
PortableServer::Servant_var<S> reference_to_servant(Server* server,
                                                    const P& p) {
  PortableServer::Servant s = server->poa()->reference_to_servant(p);
  if (s == NULL) throw Error("The servant is not located here");
  return dynamic_cast<S*>(s);
}

/// \tparam T the type of the servant object.
/// \tparam P the type of the object which is served.
template <typename T, typename P>
AbstractServantBase<T>* reference_to_servant_base(Server* server, const P& p) {
  ServantBase_var s = server->poa()->reference_to_servant(p);
  if (s.in() == NULL) throw Error("The servant is not located here");
  return dynamic_cast<AbstractServantBase<T>*>(s.in());
}

/// Create and activate a omniORB servant.
///
/// \tparam P An CORBA::Object_ptr (typically S::Object_ptr)
/// \tparam S A class deriving from AbstractServantKey and
/// PortableServer::ServantBase
///           (or a class generated by omniidl in the namespace POA_*).
/// \return a reference to the OmniORB servant.
/// \note if a OmniORB servant serving s exists, no servant is created and
///       a reference to the existing servant is returned.
template <typename P, typename S>
P makeServant(Server* server, S* s) {
  Server::ServantKey servantKey = s->getServantKey();
  S* servant = dynamic_cast<S*>(server->getServant(servantKey));
  if (servant != NULL) {
    delete s;
    return servant->_this();
  }

  PortableServer::Servant_var<S> d(s);
  // ObjectId_var object is here to delete the servantId.
  PortableServer::ObjectId_var servantId = server->poa()->activate_object(d);
  (void)servantId;

  server->addServantKeyAndServant(servantKey, d.in());
  return d->_this();
}

/// \cond
namespace details {
/// Enabled only if StorageTpl != shared_ptr
template <typename U, typename V, template <typename> class StorageTpl>
struct storage_cast_impl {
  static StorageTpl<U> run(const StorageTpl<V>& o) {
    return o.template cast<U>();
  }
};

/// Enabled only if StorageTpl == shared_ptr
template <typename U, typename V>
struct storage_cast_impl<U, V, shared_ptr> {
  static shared_ptr<U> run(const shared_ptr<V>& o) {
    return dynamic_pointer_cast<U>(o);
  }
};

/// Enabled only if StorageTpl == weak_ptr
template <typename U, typename V>
struct storage_cast_impl<U, V, weak_ptr> {
  static weak_ptr<U> run(const weak_ptr<V>& o) {
    return dynamic_pointer_cast<U>(o.lock());
  }
};
}  // namespace details
/// \endcond

/// Cast the storage class of a servant.
template <typename U, typename V, template <typename> class StorageTpl>
static StorageTpl<U> storage_cast(const StorageTpl<V>& o) {
  return details::storage_cast_impl<U, V, StorageTpl>::run(o);
}

/// \tparam T the desired HPP type
/// \tparam P the type of the object which is served.
template <typename T, typename P>
auto reference_to_object(Server* server, const P& p) {
  ServantBase_var s = server->poa()->reference_to_servant(p);
  if (s.in() == NULL) throw Error("The servant is not located here");
  typedef typename hpp_traits<T>::Base TBase;
  AbstractServantBase<TBase>* asb =
      dynamic_cast<AbstractServantBase<TBase>*>(s.in());
  if (asb == NULL) throw Error("Not an object of the correct type.");
  auto ret = storage_cast<T>(asb->get());
  if (!ret) throw Error("Object is not of the correct type.");
  return ret;
}

/// Class used to dynamiccally cast HPP classes and create the corresponding
/// servant.
/// \sa HPP_CORBASERVER_ADD_DOWNCAST_OBJECT
template <typename ServantBaseType>
class ServantFactoryBase {
 public:
  typedef typename ServantBaseType::Storage Storage;
  typedef typename ServantBaseType::Object_var Object_var;

  ServantFactoryBase(const size_type& depth) : depth_(depth) {}

  virtual Object_var servant(Server* server, const Storage& obj) = 0;

  /// Get the depth in the hierarchy tree.
  /// This allows to ensure that child classes are always handled before
  /// parent classes.
  size_type depth() const { return depth_; }

 private:
  size_type depth_;
};

template <typename ServantBaseType, typename ServantType>
struct ServantFactory : ServantFactoryBase<ServantBaseType> {
  typedef typename ServantBaseType::Object_var Object_var;
  typedef typename ServantBaseType::Storage StorageBase;

  ServantFactory(const size_type& depth)
      : ServantFactoryBase<ServantBaseType>(depth) {}

  virtual Object_var servant(Server* s, const StorageBase& o) {
    typedef typename ServantType::Storage Storage;
    Storage u = storage_cast<typename Storage::element_type>(o);
    Object_var ret;
    if (u.use_count() > 0)
      ret = makeServant<Object_var>(s, new ServantType(s, u));
    return ret;
  }
};

template <typename ServantBaseType>
std::vector<ServantFactoryBase<ServantBaseType>*>& objectDowncasts();

template <typename ServantBaseType>
void addDowncastObjects(ServantFactoryBase<ServantBaseType>* const object) {
  typedef std::vector<ServantFactoryBase<ServantBaseType>*> vector_t;
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

/// Create and activate a omniORB servant with class downcasting.
/// \tparam ServantBaseType the top classes of the hierarchy.
template <typename ServantBaseType, typename ReturnType>
typename ReturnType::Object_var makeServantDownCast(
    Server* server, const typename ServantBaseType::Storage& t) {
  typedef typename ServantBaseType::Object_var BaseObject_var;
  typedef typename ReturnType::Object_var Object_var;
  BaseObject_var servant;
  assert(CORBA::Object_Helper::is_nil(servant.in()));

  typedef std::vector<ServantFactoryBase<ServantBaseType>*> vector_t;
  typedef typename vector_t::iterator iterator;

  vector_t& vec = objectDowncasts<ServantBaseType>();
  for (iterator _obj = vec.begin(); _obj != vec.end(); ++_obj) {
    servant = (*_obj)->servant(server, t);
    if (!CORBA::Object_Helper::is_nil(servant.in())) {
      // Cast to child type.
      return Object_var(ReturnType::Object::_narrow(servant._retn()));
    }
  }
  return Object_var();
}

template <typename ServantBaseType>
typename ServantBaseType::Object_var makeServantDownCast(
    Server* server, const typename ServantBaseType::Storage& t) {
  // TODO
  // return makeServantDownCast <ServantBaseType, ServantBaseType> (server, t);
  typedef typename ServantBaseType::Object_var Object_var;
  Object_var servant;
  assert(CORBA::Object_Helper::is_nil(servant.in()));

  typedef std::vector<ServantFactoryBase<ServantBaseType>*> vector_t;
  typedef typename vector_t::iterator iterator;

  vector_t& vec = objectDowncasts<ServantBaseType>();
  for (iterator _obj = vec.begin(); _obj != vec.end(); ++_obj) {
    servant = (*_obj)->servant(server, t);
    if (!CORBA::Object_Helper::is_nil(servant.in())) break;
  }

  return servant;
}

template <typename OutType, typename InnerBaseType,
          typename InnerType = InnerBaseType>
struct vectorToSeqServant {
  Server* wrappedObject_;

  vectorToSeqServant(Server* _s) : wrappedObject_(_s) {}

  template <typename InContainer>
  inline OutType* operator()(const InContainer& input) {
    std::size_t len = std::distance(input.begin(), input.end());
    OutType* seq = new OutType();
    seq->length((CORBA::ULong)len);

    std::size_t i = 0;
    typename InContainer::const_iterator it = input.begin();
    while (it != input.end()) {
      (*seq)[(CORBA::ULong)i] =
          makeServantDownCast<InnerBaseType, InnerType>(wrappedObject_, *it)
              ._retn();
      ++it;
      ++i;
    }
    return seq;
  }
};

/// \}
}  // end of namespace corbaServer.
}  // end of namespace hpp.

/// Declare a new element of a HPP class hierachy.
/// \param depth the depth in the hierarchy tree.
/// Example:
/// \code
/// HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(Path      , Path, 0)
/// // PathVector directly inherits from Path
/// HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(PathVector, Path, 1)
/// \endcode
#define HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(ServantType, BaseServantType,  \
                                            depth)                         \
  struct HPP_CORE_DLLAPI __InitializerClass_##ServantType{                 \
      __InitializerClass_##ServantType(){                                  \
          ::hpp::corbaServer::addDowncastObjects<BaseServantType>(         \
              new ::hpp::corbaServer::ServantFactory<BaseServantType,      \
                                                     ServantType>(depth)); \
  }                                                                        \
  }                                                                        \
  ;                                                                        \
  HPP_CORBASERVER_DLLLOCAL __InitializerClass_##ServantType                \
      __instance_##ServantType;

#endif  // HPP_CORBASERVER_SERVANT_BASE_HH
