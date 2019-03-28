# -*- python -*-
#                           Package   : hpp-corbaserver
# template.py               Created on: 2019
#                           Author    : Joseph Mirabel
#
#    Copyright (C) 2019 LAAS-CNRS
#
#  This file is part of hpp-corbaserver.
#
#  hpp-corbaserver is free software; you can redistribute it and/or modify it
#  under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
#  02111-1307, USA.
#
# Description:
#   
#   Interface implementation templates for HPP

## Code to actually implement an interface
##
base_interface_def = """
//
// Class implementing IDL interface @fq_name@
//
@open_namespaces@
template <typename _Base, typename _Storage>
class @impl_tpl_name@: public @impl_base_name@<@hpp_class@, _Storage>, public virtual _Base
{
public:
  typedef @hpp_class@ HppBase;

  SERVANT_BASE_TYPEDEFS(@fq_name@, HppBase);

public:
  // standard constructor
  @impl_tpl_name@(Server* server, const _Storage& s);
  virtual ~@impl_tpl_name@();

  // methods corresponding to defined IDL attributes and operations
  @operations@
};

typedef @impl_tpl_name@<@fq_POA_name@,@storage@ > @impl_name@;
@close_namespaces@
"""

base_interface_code = """\
//
// Implementational code for IDL interface @fqname@
//
@open_namespaces@
template <typename _Base, typename _Storage>
@impl_tpl_name@<_Base, _Storage>::@impl_tpl_name@(Server* server, const _Storage& s)
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
class @impl_tpl_name@: public @impl_base_name@<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename @impl_base_name@<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(@fq_name@, HppBase);

public:
  // standard constructor
  @impl_tpl_name@(Server* server, const _Storage& s);
  virtual ~@impl_tpl_name@();

  // methods corresponding to defined IDL attributes and operations
  @operations@
};

typedef @impl_tpl_name@<@fq_POA_name@,@storage@ > @impl_name@;
@close_namespaces@
"""


inherited_interface_code = """\
//
// Implementational code for IDL interface @fqname@
//
@open_namespaces@
template <typename _Base, typename _Storage>
@impl_tpl_name@<_Base, _Storage>::@impl_tpl_name@(Server* server, const _Storage& s)
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
    @conversions@
    @store_return@ (getT()->@hpp_opname@ (@arg_calls@));
    @do_return@
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}"""

provided_operation_impl_code = """
template <typename _Base, typename _Storage>
@return_type@ @impl_tpl_name@<_Base, _Storage>::@opname@ (@arg_defs@)
{
  try {
   @implementation@
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}"""

predefined_operations_impl_code = \
{
        "str":  """
template <typename _Base, typename _Storage>
@return_type@ @impl_tpl_name@<_Base, _Storage>::str ()
{
  try {
    // automatically generated code.
    std::ostringstream oss; oss << *get();
    std::string res = oss.str();
    return CORBA::string_dup(res.c_str());
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}""",
}

definition_object_downcast = """\
template <>
std::vector< ServantFactoryBase<@servant_class@>* >&
objectDowncasts<@servant_class@> ()
{
  static std::vector< ServantFactoryBase<@servant_class@>* > vector;
  return vector;
}
"""

definition_add_object_downcast = """\
@open_namespaces@
HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(@class_name@, @base_class_name@, @depth@)
@close_namespaces@
"""

storage_decl = """\
template <typename D>
class @storage_class_name@ : public AbstractStorage<D, @hpp_base_class@>
{
public:
  typedef AbstractStorage <D, @hpp_base_class@> parent_t;
  using parent_t::element;
  using typename parent_t::ptr_t;

  @storage_attributes@
  @storage_class_name@ (@storage_constr_attr_decl@const ptr_t& _d)
    : parent_t(_d)@storage_constr_attr_defs@ {}

  template <typename T> @storage_class_name@<T> cast () const
  {
    return @storage_class_name@<T> (@storage_attr_call@HPP_DYNAMIC_PTR_CAST(T, element.lock()));
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
