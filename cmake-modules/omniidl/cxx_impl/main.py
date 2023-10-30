# -*- python -*-
#                           Package   : hpp-corbaserver
# main.py                   Created on: 2019
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
#   Produce interface implementations for HPP

"""Produce interface implementations for HPP"""

from __future__ import print_function

from cxx_impl import template
from omniidl import idlast, idlvisitor
from omniidl_be.cxx import ast, config, id, output, types, util


def if_cpp11(then, _else):
    if config.state["C++11"]:
        return then
    else:
        return _else


def __init__(
    hpp_stream,
    hxx_stream,
    cc_stream,
    idl_filename,
    prefix,
    hh_filename,
    hpp_filename,
    hxx_filename,
):
    self.hpp_stream = hpp_stream
    self.hxx_stream = hxx_stream
    self.cc_stream = cc_stream

    self.idl_filename = idl_filename

    self.prefix = prefix
    self.hh_filename = hh_filename
    self.hpp_filename = hpp_filename
    self.hxx_filename = hxx_filename


def makeError(msg, file, line, errtype=Exception):
    import os

    path = os.path.dirname(__file__)
    path = os.path.dirname(path)
    help = "omniidl -p{} -bcxx_impl -u".format(path)
    return errtype(
        "\n{}:{}: {}\nThe help may help:\n{}\n".format(file, line, msg, help)
    )


# Given an IDL name convert it into the template name of the
# implementation class
def impl_tplname(name):
    return name.suffix("Servant").simple(cxx=1)


# Given an IDL name convert it into the corresponding hpp class name
def hpp_scope(name, suffix=""):
    scope = [n.replace("_idl", "") for n in name.scope()] + [
        name.simple(cxx=0),
    ]
    return id.Name(scope)


# Given an IDL name convert it into the corresponding hpp class name
def hpp_name(name, suffix=""):
    scope = [n.replace("_idl", "") for n in name.scope()] + [
        name.simple(cxx=0),
    ]
    return "::".join(scope)


# Given an IDL name convert it into the corresponding hpp servant class id.Name
def hpp_servant_scope(name):
    scope = [n.replace("_idl", "_impl") for n in name.scope()] + [
        name.simple(cxx=0),
    ]
    return id.Name(scope)


# Given an IDL name convert it into the corresponding hpp servant class name
def hpp_servant_name(name, environment=None):
    scope = hpp_servant_scope(name)
    # scope = [ n.replace('_idl', '') for n in name.scope() ] + [ name.simple(cxx=0), ]
    # scope.insert(1, "corbaServer")
    if environment:
        i = 0
        sc = scope.scope()
        esc = environment.scope()
        while i < len(sc) and i < len(esc) and sc[i] == esc[i]:
            i += 1
        scope._scopedName = scope._scopedName[i:]
    return scope.fullyQualify(cxx=1)


def namespaces(name, environment=None):
    scope = hpp_servant_scope(name)
    if environment:
        env = environment.scope()
        assert scope.scope()[: len(env)] == env
        scope = scope.scope()[len(env) :]
    else:
        scope = scope.scope()
    open_ns = "\n\n".join(["namespace {name} {{".format(name=n) for n in scope])
    closens = "\n\n".join(
        ["}} // namespace {name}".format(name=n) for n in reversed(scope)]
    )
    return open_ns, closens


# Main code entrypoint
def run(tree):
    # first thing is to build the interface implementations
    decl = output.StringStream()
    impl = output.StringStream()
    bii = BuildInterfaceImplementations(decl, impl)
    tree.accept(bii)

    object_downcasts_methods = output.StringStream()
    add_object_downcasts = output.StringStream()
    biod = BuildInterfaceObjectDowncasts(object_downcasts_methods, add_object_downcasts)
    tree.accept(biod)

    openns, closens = "", ""
    guard = id.Name([config.state["Basename"]]).guard()

    # Create header file.
    hpp_stream.out(
        template.hpp_file,
        guard_prefix=config.state["GuardPrefix"],
        guard=guard,
        includes=bii.includes,
        idl_hh=prefix + hh_filename,
        file=idl_filename,
        interface_declarations=str(decl),
        open_namespaces=openns,
        close_namespaces=closens,
    )

    # Create definition of templated functions.
    hxx_stream.out(
        template.hxx_file,
        guard_prefix=config.state["GuardPrefix"],
        guard=guard,
        idl_hpp=prefix + hpp_filename,
        file=idl_filename,
        interface_implementations=str(impl),
        open_namespaces=openns,
        close_namespaces=closens,
    )

    # Create other definition files.
    cc_stream.out(
        template.cc_file,
        idl_hxx=prefix + hxx_filename,
        file=idl_filename,
        object_downcasts_methods=object_downcasts_methods,
        add_object_downcasts=add_object_downcasts,
        open_namespaces=openns,
        close_namespaces=closens,
    )


def get_base_class(node, retDepth=False):
    base = node
    depth = 0
    while base.inherits():
        depth += 1
        base = base.inherits()[0]
    if retDepth:
        return base, depth
    else:
        return base


class Builder(idlvisitor.AstVisitor):
    def __init__(self):
        self._modules = {}

    def registerModule(self, m):
        mname = m.identifier()
        if mname.endswith("_idl"):
            mname = mname[:-4]
        self._modules[m.identifier()] = mname
        for comment in m.comments():
            if comment.text().startswith("// ") or comment.text().startswith("///"):
                pass
            elif comment.text().startswith("//->"):
                self._modules[m.identifier()] = comment.text()[4:].strip()

    def toCppNamespace(self, name):
        def module(n):
            if n in self._modules:
                return self._modules[n]
            if n.endswith("_idl"):
                return n[:-4]
            return n

        scope = [module(n) for n in name.scope()] + [
            name.simple(cxx=0),
        ]
        return id.Name(scope)

    # type conversions
    def argConversion(self, name, _type, _in, _out, param):
        assert _in or _out
        tmp = "_" + name
        in_conv_str = None
        out_conv_str = None

        if _type.string():
            if _in:
                in_conv_str = "std::string {} ({});".format(tmp, name)
            else:
                in_conv_str = "std::string {};".format(tmp)
            if _out:
                out_conv_str = "{} = hpp::corbaServer::c_str ({});".format(name, tmp)
            return tmp, in_conv_str, out_conv_str
        elif _type.typedef():
            if _type.type().name() in ("size_t", "size_type", "value_type"):
                return name, "", out_conv_str
            elif _type.type().name() == "intSeq":
                if _in:
                    in_conv_str = "std::vector<int> {} = hpp::corbaServer::intSeqToVector ({});".format(
                        tmp, name
                    )
                    if _out:
                        out_conv_str = "hpp::corbaServer::toIntSeq ({}, {});".format(
                            tmp, name
                        )
                else:  # !_in => _out
                    assert _out
                    in_conv_str = "std::vector<int> {};".format(
                        tmp,
                    )
                    out_conv_str = "{} = hpp::corbaServer::toIntSeq ({});".format(
                        name, tmp
                    )
                return tmp, in_conv_str, out_conv_str
            elif _type.type().name() == "floatSeq":
                if _in:
                    in_conv_str = "hpp::core::vector_t {} = hpp::corbaServer::floatSeqToVector ({});".format(
                        tmp, name
                    )
                    if _out:
                        out_conv_str = (
                            "hpp::corbaServer::vectorToFloatSeq ({}, {});".format(
                                tmp, name
                            )
                        )
                else:  # !_in => _out
                    assert _out
                    in_conv_str = "hpp::core::vector_t {};".format(
                        tmp,
                    )
                    out_conv_str = (
                        "{} = hpp::corbaServer::vectorToFloatSeq ({});".format(
                            name, tmp
                        )
                    )
                return tmp, in_conv_str, out_conv_str
            elif _type.type().name() == "floatSeqSeq":
                if _in:
                    in_conv_str = "hpp::core::matrix_t {} = hpp::corbaServer::floatSeqSeqToMatrix ({});".format(
                        tmp, name
                    )
                    if _out:
                        out_conv_str = "/* not implemented. See vectorToFloatSeq */ hpp::corbaServer::matrixToFloatSeqSeq ({}, {});".format(
                            tmp, name
                        )
                else:  # !_in => _out
                    in_conv_str = "hpp::core::matrix_t {};".format(
                        tmp,
                    )
                    assert _out
                    out_conv_str = (
                        "{} = hpp::corbaServer::matrixToFloatSeqSeq ({});".format(
                            name, tmp
                        )
                    )
                return tmp, in_conv_str, out_conv_str
            elif _type.type().name() == "Transform_":
                if _out:
                    raise makeError(
                        "out Transform_ is currently not supported",
                        param.file(),
                        param.line(),
                    )
                return (
                    tmp,
                    "hpp::core::Transform3f {} = hpp::corbaServer::toTransform3f ({});".format(
                        tmp, name
                    ),
                    out_conv_str,
                )
            elif _type.type().name() == "TransformSeq":
                if _out:
                    raise makeError(
                        "out TransformSeq is currently not supported",
                        param.file(),
                        param.line(),
                    )
                return (
                    tmp,
                    "std::vector<hpp::core::Transform3f> {} = hpp::corbaServer::toTransform3f ({});".format(
                        tmp, name
                    ),
                    out_conv_str,
                )
            elif _type.type().name() == "Names_t":
                in_conv_str = "typedef std::vector<std::string> strings_t;"
                if _in:
                    in_conv_str += "strings_t {} = hpp::corbaServer::toStrings<strings_t> ({});".format(
                        tmp, name
                    )
                else:
                    in_conv_str += "strings_t {};".format(
                        tmp,
                    )
                if _out:
                    out_conv_str = "hpp::corbaServer::toNames_t ({var}.begin(), {var}.end());".format(
                        var=tmp
                    )
                return tmp, in_conv_str, out_conv_str
            elif _type.type().name() == "ComparisonTypes_t":
                if _in:
                    in_conv_str = "hpp::constraints::ComparisonTypes_t {} = hpp::corbaServer::convertComparison ({});".format(
                        tmp, name
                    )
                    if _out:
                        raise ValueError(
                            "inout ComparisonTypes_t is not implemented. See floatSeq convertion for an example."
                        )
                else:  # !_in => _out
                    assert _out
                    raise ValueError(
                        "inout ComparisonTypes_t is not implemented. See floatSeq convertion for an example."
                    )
                return tmp, in_conv_str, out_conv_str
            elif _type.type().name() == "RelativeMotionMatrix":
                if _in:
                    in_conv_str = "hpp::core::RelativeMotion::matrix_type {} = hpp::corbaServer::intSeqSeqToMatrix ({}).cast<hpp::core::RelativeMotion::RelativeMotionType>();".format(
                        tmp, name
                    )
                    if _out:
                        out_conv_str = "/* not implemented. See vectorToFloatSeq */ hpp::corbaServer::matrixToIntSeqSeq ({}.cast<int>(), {});".format(
                            tmp, name
                        )
                else:  # !_in => _out
                    in_conv_str = "hpp::core::RelativeMotion::matrix_type {}".format(
                        tmp
                    )
                    assert _out
                    out_conv_str = "{} = hpp::corbaServer::matrixToIntSeqSeq ({}.cast<int>());".format(
                        name, tmp
                    )
                return tmp, in_conv_str, out_conv_str
            print("typedef", _type.type().name())
            return name, "", out_conv_str
        elif _type.is_basic_data_types():
            return name, "", out_conv_str
        elif _type.objref():
            if _out:
                raise makeError(
                    "out objects is currently not supported", param.file(), param.line()
                )
            conv = "{typeptr} {tmp} = ::hpp::corbaServer::reference_to_object<{type}>(server_, {name});".format(
                type=self.toCppNamespace(
                    id.Name(_type.type().scopedName())
                ).fullyQualify(cxx=1),
                typeptr=self.toCppNamespace(
                    id.Name(_type.type().scopedName()).suffix("Ptr_t")
                ).fullyQualify(cxx=1),
                tmp=tmp,
                name=name,
            )
            return tmp, conv, out_conv_str
        print(_type.type(), _type.kind())
        return name, "", out_conv_str

    def retConversion(self, _type):
        """
        Returns two values:
        - storing: will be used as "${storing} (${hpp_method_call})."
        - converting: will be copied as such after the line above.
        """
        if _type.void():
            return "", ""
        if _type.is_basic_data_types():
            return "{} __return__".format(_type.op(types.RET)), "return __return__;"
        if _type.string():
            return "char* __return__ = ::hpp::corbaServer::c_str", "return __return__;"
        if _type.typedef():
            if _type.type().name() in ("size_t", "size_type", "value_type"):
                return "{} __return__".format(_type.op(types.RET)), "return __return__;"
            elif _type.type().name() == "floatSeq":
                return (
                    "{} __return__ = hpp::corbaServer::vectorToFloatSeq".format(
                        _type.op(types.RET)
                    ),
                    "return __return__;",
                )
            elif _type.type().name() == "floatSeqSeq":
                return (
                    "{} __return__ = hpp::corbaServer::matrixToFloatSeqSeq".format(
                        _type.op(types.RET)
                    ),
                    "return __return__;",
                )
            elif _type.type().name() == "Transform_":
                return (
                    "hpp::core::Transform3f __return__",
                    "return hpp::corbaServer::toHppTransform (__return__);",
                )
            elif _type.type().name() == "Names_t":
                return (
                    "{} __return__".format(
                        if_cpp11("auto", "std::vector<std::string>")
                    ),
                    "return hpp::corbaServer::toNames_t (__return__);",
                )
            elif _type.type().name() == "ComparisonTypes_t":
                return (
                    "hpp::constraints::ComparisonTypes_t __return__",
                    "return hpp::corbaServer::convertComparison (__return__);",
                )
            else:
                unaliased = _type.deref()
                if unaliased.sequence():
                    innerType = types.Type(unaliased.type().seqType())
                    if innerType.objref():
                        if isinstance(innerType.type().decl(), idlast.Forward):
                            base = get_base_class(innerType.type().decl().fullDecl())
                        else:
                            base = get_base_class(innerType.type().decl())
                        return (
                            "{outType}* __return__ = hpp::corbaServer::vectorToSeqServant<{outType},{innerBaseType},{innerType}>(server_)".format(
                                innerBaseType=hpp_servant_name(
                                    id.Name(base.scopedName())
                                ),
                                innerType=hpp_servant_name(
                                    id.Name(innerType.type().scopedName())
                                ),
                                outType=id.Name(_type.type().scopedName()).fullyQualify(
                                    cxx=1
                                ),
                            ),
                            "return __return__;",
                        )
                    else:
                        print("Unhandled sequence of", innerType.type())
                else:
                    print("Unhandled type", _type.type().name())
            return "ThisTypeCouldNotBeDeduced __return__", "return __return__;"
        if _type.objref():
            if isinstance(_type.type().decl(), idlast.Forward):
                base = get_base_class(_type.type().decl().fullDecl())
            else:
                base = get_base_class(_type.type().decl())
            store = "{type} __return__".format(
                type=self.toCppNamespace(
                    id.Name(_type.type().scopedName()).suffix("Ptr_t")
                ).fullyQualify(cxx=1)
            )
            conv = "return ::hpp::corbaServer::makeServantDownCast<{basetype},{type}>(server_, __return__)._retn();".format(
                type=hpp_servant_name(id.Name(_type.type().scopedName())),
                basetype=hpp_servant_name(id.Name(base.scopedName())),
            )
            return store, conv
        print(_type.type(), _type.kind())
        return "", ""

    # modules can contain interfaces
    def visitModule(self, node):
        self.registerModule(node)
        for n in node.definitions():
            n.accept(self)


# Build the interface implementations (hpp, hxx)
#
class BuildInterfaceImplementations(Builder):
    def __init__(self, decl, impl):
        # super(BuildInterfaceImplementations, self).__init__()
        Builder.__init__(self)
        self.interface_declarations = decl
        self.interface_implementations = impl
        self.includes = ""

        self.storages = {}

    # Tree walking code
    def visitAST(self, node):
        for n in node.declarations():
            if ast.shouldGenerateCodeForDecl(n):
                for c in n.comments():
                    if len(c.text()) > 2 and c.text()[2] == "*":
                        self.includes += c.text()[3:].strip() + "\n"
                n.accept(self)

    def visitStruct(self, node):
        if not node.identifier().endswith("Storage"):
            return
        if not node.members():
            return

        class Storage(object):
            pass

        storage = Storage()

        storage.members = []
        for m in node.members():
            type = self.toCppNamespace(id.Name(m.memberType().scopedName())).suffix(
                "Ptr_t"
            )
            for d in m.declarators():
                storage.members.append((type, d.identifier()))

        storage.sc = hpp_servant_scope(id.Name(node.scopedName()))

        sc = node.scopedName()
        sc[-1] = sc[-1][:-7]
        storage.class_sc = hpp_servant_scope(id.Name(sc))

        st = output.StringStream()
        st.out(
            template.storage_decl,
            storage_class_name=storage.sc.simple(),
            hpp_base_class=storage.class_sc.simple(),
            storage_attributes="\n".join(
                [t.fullyQualify() + " " + n for t, n in storage.members]
            ),
            storage_constr_attr_decl=", ".join(
                [t.fullyQualify() + " _" + n for t, n in storage.members]
            )
            + ", ",
            storage_constr_attr_defs=", "
            + ", ".join([n + " (_" + n + ")" for t, n in storage.members]),
            storage_attr_call=", ".join([n for t, n in storage.members]) + ", ",
        )
        storage.decl = str(st)

        self.storages[storage.class_sc.fullyQualify(cxx=0)] = storage

    # interfaces cannot be further nested
    def visitInterface(self, node):
        scopedName = id.Name(node.scopedName())
        openns, closens = namespaces(scopedName)

        impl_name = scopedName.simple(cxx=1)
        impl_tpl_name = impl_tplname(scopedName)
        cxx_fqname = scopedName.fullyQualify()
        hpp_class = self.toCppNamespace(scopedName).fullyQualify(cxx=1)

        fqname = scopedName.fullyQualify(cxx=0)

        is_base_class = not bool(node.inherits())
        # ptr_t = self.toCppNamespace (scopedName.suffix("Ptr_t")).fullyQualify(cxx=1)
        wkptr_t = "hpp::weak_ptr<{}>".format(
            self.toCppNamespace(scopedName).fullyQualify(cxx=1)
        )
        if is_base_class:
            key = hpp_servant_name(scopedName)
            impl_base_name = "hpp::corbaServer::ServantBase"
            if key in self.storages:
                st = self.storages[key]
                # declare storage
                self.interface_declarations.out(st.decl)
                storage = st.sc.simple() + "< " + wkptr_t + " >"
            else:
                storage = wkptr_t
            hpp_base_class = None
        else:
            baseScopedName = id.Name(node.inherits()[0].scopedName())
            hpp_base_class = self.toCppNamespace(baseScopedName).fullyQualify(cxx=1)
            key = hpp_servant_name(baseScopedName)
            impl_base_name = hpp_servant_name(baseScopedName.suffix("Servant"))
            if key in self.storages:
                st = self.storages[key]
                storage = st.sc.simple() + "< " + wkptr_t + " >"
                self.storages[fqname] = st
            else:
                storage = wkptr_t

        # build methods corresponding to attributes, operations etc.
        # attributes[] and operations[] will contain lists of function
        # signatures eg
        #   [ char *echoString(const char *mesg) ]
        attributes = []
        operations = []

        allCallables = node.callables()

        # declarations contains a list of in-class decl signatures
        # implementations contains a list of out of line impl signatures
        # (typically differ by classname::)
        declarations = output.StringStream()
        implementations = output.StringStream()

        for c in allCallables:
            comments = c.comments()
            hpp_opname = None
            comments_impl = []
            for comment in comments:
                if comment.text().startswith("// ") or comment.text().startswith("///"):
                    # Skip this comment
                    pass
                elif comment.text().startswith("//*"):
                    if not comments_impl:
                        comments_impl.append(
                            " // generated from {}:{}\n".format(
                                node.file(), node.line()
                            )
                        )
                    comments_impl.append(comment.text()[3:])
                elif comment.text().startswith("//->"):
                    if hpp_opname is not None:
                        raise makeError(
                            "Function was already renamed",
                            comment.file(),
                            comment.line(),
                        )
                    hpp_opname = comment.text()[4:].strip()

            if isinstance(c, idlast.Attribute):
                attrType = types.Type(c.attrType())
                d_attrType = attrType.deref()

                for i in c.identifiers():
                    attribname = id.mapID(i)
                    returnType = attrType.op(types.RET)
                    inType = attrType.op(types.IN)
                    attributes.append(returnType + " " + attribname + "()")
                    # need a set method if not a readonly attribute
                    if not c.readonly():
                        declarations.out(
                            template.operation_decl_code,
                            return_type="void",
                            opname=attribname,
                            arg_defs=inType + " _" + attribname,
                        )

                        tmpVar, in_conv, out_conv = self.argConversion(
                            "_" + attribname, attrType, True, False, c
                        )
                        implementations.out(
                            template.operation_impl_code,
                            return_type="void",
                            impl_tpl_name=impl_tpl_name,
                            opname=attribname,
                            hpp_opname=hpp_opname if hpp_opname else attribname,
                            arg_defs=inType + " _" + attribname,
                            in_conversions=in_conv if in_conv else "",
                            out_conversions=out_conv if out_conv else "",
                            store_return="",
                            do_return="",
                            arg_calls=tmpVar,
                        )

                    declarations.out(
                        template.operation_decl_code,
                        return_type=returnType,
                        opname=attribname,
                        arg_defs="",
                    )

                    store_return, do_return = self.retConversion(attrType)
                    implementations.out(
                        template.operation_impl_code,
                        return_type=returnType,
                        impl_tpl_name=impl_tpl_name,
                        opname=attribname,
                        hpp_opname=hpp_opname if hpp_opname else attribname,
                        arg_defs="",
                        in_conversions="",
                        out_conversions="",
                        store_return=store_return,
                        do_return=do_return,
                        arg_calls="",
                    )
            elif isinstance(c, idlast.Operation):
                params = []
                paramNames = []
                in_conversions = []
                out_conversions = []
                for p in c.parameters():
                    paramType = types.Type(p.paramType())
                    cxx_type = paramType.op(types.direction(p), use_out=0)

                    argname = id.mapID(p.identifier())
                    if not comments_impl:
                        tmpVar, in_conv, out_conv = self.argConversion(
                            argname, paramType, p.is_in(), p.is_out(), p
                        )
                        if in_conv:
                            in_conversions.append(in_conv)
                        if out_conv:
                            out_conversions.append(out_conv)
                    else:
                        tmpVar = argname

                    params.append(cxx_type + " " + argname)
                    paramNames.append(tmpVar)

                # deal with possible "context"
                if c.contexts() != []:
                    params.append("CORBA::Context_ptr _ctxt")

                store_return, do_return = self.retConversion(types.Type(c.returnType()))
                return_type = types.Type(c.returnType()).op(types.RET)

                error_type = "::hpp::Error"
                if len(c.raises()) > 0:
                    error_type = "::" + "::".join(c.raises()[0].scopedName())

                opname = id.mapID(c.identifier())
                arguments = ", ".join(params)
                argumentsCall = ", ".join(paramNames)
                args = opname + "(" + arguments + ")"

                declarations.out(
                    template.operation_decl_code,
                    return_type=return_type,
                    opname=opname,
                    arg_defs=arguments,
                )

                if comments_impl:
                    implementations.out(
                        template.provided_operation_impl_code,
                        return_type=return_type,
                        impl_tpl_name=impl_tpl_name,
                        opname=opname,
                        error_type=error_type,
                        implementation="".join(comments_impl),
                        arg_defs=arguments,
                    )
                elif opname in template.predefined_operations_impl_code:
                    # assert not c.parameters(), "Interface operation str should not have arguments"
                    implementations.out(
                        template.predefined_operations_impl_code[opname],
                        return_type=return_type,
                        impl_tpl_name=impl_tpl_name,
                        opname=opname,
                        error_type=error_type,
                        conversions="\n  ".join(in_conversions),
                        arg_defs=arguments,
                        store_return=store_return,
                        do_return=do_return,
                        arg_calls=argumentsCall,
                    )
                else:
                    implementations.out(
                        template.operation_impl_code,
                        return_type=return_type,
                        impl_tpl_name=impl_tpl_name,
                        opname=opname,
                        hpp_opname=hpp_opname if hpp_opname is not None else opname,
                        error_type=error_type,
                        in_conversions="\n  ".join(in_conversions),
                        out_conversions="\n  ".join(out_conversions),
                        arg_defs=arguments,
                        store_return=store_return,
                        do_return=do_return,
                        arg_calls=argumentsCall,
                    )

            else:
                util.fatalError("Internal error generating interface member")
                raise AssertionError("No code for interface member: " + repr(c))

        openns, closens = namespaces(scopedName)

        # Output the _i class definition definition
        self.interface_declarations.out(
            template.base_interface_def
            if is_base_class
            else template.inherited_interface_def,
            fq_name=fqname,
            impl_tpl_name=impl_tpl_name,
            impl_base_name=impl_base_name,
            operations=str(declarations),
            impl_name=impl_name,
            fq_POA_name="POA_" + cxx_fqname,
            hpp_class=hpp_class,
            hpp_base_class=hpp_base_class,
            storage=storage,
            open_namespaces=openns,
            close_namespaces=closens,
        )

        self.interface_implementations.out(
            template.base_interface_code
            if is_base_class
            else template.inherited_interface_code,
            fqname=fqname,
            impl_name=impl_name,
            impl_tpl_name=impl_tpl_name,
            impl_base_name=impl_base_name,
            hpp_class=hpp_class,
            operations=str(implementations),
            open_namespaces=openns,
            close_namespaces=closens,
        )


# Build the interface object downcasts (cc)
#
class BuildInterfaceObjectDowncasts(Builder):
    def __init__(self, methods, adders):
        # super(BuildInterfaceObjectDowncasts, self).__init__()
        Builder.__init__(self)
        self.methods = methods
        self.adders = adders

    # Tree walking code
    def visitAST(self, node):
        for n in node.declarations():
            if ast.shouldGenerateCodeForDecl(n):
                n.accept(self)

    # interfaces cannot be further nested
    def visitInterface(self, node):
        idlScopedName = id.Name(node.scopedName())
        servantScope = hpp_servant_scope(idlScopedName)
        scopedName = hpp_servant_name(idlScopedName)

        base, depth = get_base_class(node, retDepth=True)
        baseScopedName = hpp_servant_name(id.Name(base.scopedName()), servantScope)

        if depth == 0:
            self.methods.out(
                template.definition_object_downcast,
                servant_class=servantScope.fullyQualify(),
            )

        openns, closens = namespaces(servantScope)
        self.adders.out(
            template.definition_add_object_downcast,
            # class_name = scopedName,
            # base_class_name = baseScopedName,
            class_name=idlScopedName.simple(),
            base_class_name=baseScopedName,
            open_namespaces=openns,
            close_namespaces=closens,
            depth=depth,
        )
