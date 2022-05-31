# -*- python -*-
#                           Package   : hpp-corbaserver
# template.py               Created on: 2019
#                           Author    : Joseph Mirabel
#
#    Copyright (C) 2019 LAAS-CNRS
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
#
# Description:
#
#   Interface implementation templates for HPP

# # Code to actually implement an interface
# #
base_interface_def = """
//
// Class implementing IDL interface @fq_name@
//
@open_namespaces@
template <typename _Base, typename _Storage>
class @impl_tpl_name@:
public @impl_base_name@<@hpp_class@, _Storage>, public virtual _Base
{
public:
  typedef @hpp_class@ HppBase;

  SERVANT_BASE_TYPEDEFS(@fq_name@, HppBase);

public:
  // standard constructor
  @impl_tpl_name@(::hpp::corbaServer::Server* server, const _Storage& s);
  virtual ~@impl_tpl_name@();

  // methods corresponding to defined IDL attributes and operations
  @operations@
};

typedef @impl_tpl_name@<@fq_POA_name@,@storage@ > @impl_name@;
@close_namespaces@

namespace hpp {
namespace corbaServer {
template<> struct hpp_traits<@hpp_class@>{ typedef @hpp_class@ Base; };
} // namespace corbaServer
} // namespace corbaServer
"""

base_interface_code = """\
//
// Implementational code for IDL interface @fqname@
//
@open_namespaces@
template <typename _Base, typename _Storage>
@impl_tpl_name@<_Base, _Storage>::@impl_tpl_name@(::hpp::corbaServer::Server* server,
                                                  const _Storage& s)
  : @impl_base_name@<@hpp_class@, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
@impl_tpl_name@<_Base, _Storage>::~@impl_tpl_name@()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations
@operations@
// End of implementational code
@close_namespaces@
"""

inherited_interface_def = """\
//
// Class implementing IDL interface @fq_name@
//
@open_namespaces@
template <typename _Base, typename _Storage>
class @impl_tpl_name@: public @impl_base_name@<_Base, _Storage>
{
public:
  typedef typename @impl_base_name@<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(@fq_name@, HppBase);

public:
  // standard constructor
  @impl_tpl_name@(::hpp::corbaServer::Server* server, const _Storage& s);
  virtual ~@impl_tpl_name@();

  // methods corresponding to defined IDL attributes and operations
  @operations@
};

typedef @impl_tpl_name@<@fq_POA_name@,@storage@ > @impl_name@;
@close_namespaces@

namespace hpp {
namespace corbaServer {
template<> struct hpp_traits<@hpp_class@>{ typedef @hpp_base_class@ Base; };
} // namespace corbaServer
} // namespace corbaServer
"""


inherited_interface_code = """\
//
// Implementational code for IDL interface @fqname@
//
@open_namespaces@
template <typename _Base, typename _Storage>
@impl_tpl_name@<_Base, _Storage>::@impl_tpl_name@(::hpp::corbaServer::Server* server,
                                                  const _Storage& s)
  : @impl_base_name@<_Base, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
@impl_tpl_name@<_Base, _Storage>::~@impl_tpl_name@()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations
@operations@
// End of implementational code
@close_namespaces@
"""


operation_decl_code = """
@return_type@ @opname@ (@arg_defs@);
"""

operation_impl_code = """
template <typename _Base, typename _Storage>
@return_type@ @impl_tpl_name@<_Base, _Storage>::@opname@ (@arg_defs@)
{
  try {
    // automatically generated code.
    @in_conversions@
    @store_return@ (getT()->@hpp_opname@ (@arg_calls@));
    @out_conversions@
    @do_return@
  } catch (const std::exception& e) {
    throw @error_type@ (e.what());
  }
}"""

provided_operation_impl_code = """
template <typename _Base, typename _Storage>
@return_type@ @impl_tpl_name@<_Base, _Storage>::@opname@ (@arg_defs@)
{
  try {
   @implementation@
  } catch (const std::exception& e) {
    throw @error_type@ (e.what());
  }
}"""

predefined_operations_impl_code = {
    "str": """
template <typename _Base, typename _Storage>
@return_type@ @impl_tpl_name@<_Base, _Storage>::str ()
{
  try {
    // automatically generated code.
    std::ostringstream oss; oss << *get();
    std::string res = oss.str();
    return CORBA::string_dup(res.c_str());
  } catch (const std::exception& e) {
    throw @error_type@ (e.what());
  }
}""",
    "deleteThis": """
template <typename _Base, typename _Storage>
void @impl_tpl_name@<_Base, _Storage>::deleteThis ()
{
  try {
    // automatically generated code.
    _ServantBase::deleteThis();
  } catch (const std::exception& e) {
    throw @error_type@ (e.what());
  }
}""",
}

definition_object_downcast = """\
namespace hpp {
namespace corbaServer {
template <>
std::vector< ServantFactoryBase<@servant_class@>* >& objectDowncasts<@servant_class@> ()
{
  static std::vector< ServantFactoryBase<@servant_class@>* > vector;
  return vector;
}
} // namespace corbaServer
} // namespace hpp
"""

definition_add_object_downcast = """\
@open_namespaces@
HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(@class_name@, @base_class_name@, @depth@)
@close_namespaces@
"""

storage_decl = """\
template <typename D>
class @storage_class_name@ :
public ::hpp::corbaServer::AbstractStorage<D, @hpp_base_class@>
{
public:
  typedef ::hpp::corbaServer::AbstractStorage <D, @hpp_base_class@> parent_t;
  using parent_t::element;
  using typename parent_t::ptr_t;

  @storage_attributes@
  @storage_class_name@ (@storage_constr_attr_decl@const ptr_t& _d)
    : parent_t(_d)@storage_constr_attr_defs@ {}

  template <typename T> @storage_class_name@<T> cast () const
  {
    return @storage_class_name@<T> (
        @storage_attr_call@HPP_DYNAMIC_PTR_CAST(T, element.lock()));
  }
};
"""

hpp_file = """\
#ifndef @guard_prefix@__@guard@_hpp__
#define @guard_prefix@__@guard@_hpp__

//
// Implemention of IDL interfaces in file @file@
//

#include <@idl_hh@>
#include "hpp/corbaserver/servant-base.hh"

@includes@

@open_namespaces@

@interface_declarations@

@close_namespaces@

#endif // @guard_prefix@__@guard@_hpp__
"""

hxx_file = """\
#ifndef @guard_prefix@__@guard@_hxx__
#define @guard_prefix@__@guard@_hxx__

//
// Implemention of IDL interfaces in file @file@
//

#include <@idl_hpp@>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

@open_namespaces@

@interface_implementations@

@close_namespaces@

#endif // @guard_prefix@__@guard@_hxx__
"""

cc_file = """\
//
// Implemention of IDL interfaces in file @file@
//

#include <@idl_hxx@>

@open_namespaces@

@object_downcasts_methods@

@add_object_downcasts@

@close_namespaces@

"""
