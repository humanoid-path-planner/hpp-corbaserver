# -*- python -*-
#                           Package   : hpp-corbaserver
# __init__.py               Created on: 2019
#			    Author    : Joseph Mirabel
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
#   Entrypoint to implementation generation code

import os

from omniidl_be.cxx import id, config, util, ast, output, support, descriptor
from cxx_impl import main

cpp_args = ["-D__OMNIIDL_CXX_IMPL__"]
usage_string = """\
  -Wbh=<suffix>     Specify suffix for generated header files using cxx backend
  -Wbhh=<suffix>     Specify suffix for generated header files using cxx backend
  -Wbi=<suffix>     Specify suffix for generated templated definition files
  -Wbc=<suffix>     Specify suffix for generated definition files
  -Wbinc_prefix=<p> Specify the path prefix for include directives
  -Wbguard_prefix   Prefix for include guards in generated headers

Extra features added to IDL language:

  - automatically generate implementation of interfaces, when the API is rigorously the same.
    For instance, in interface Bar:
    * string foo (in string a);
    assumes class Bar has a method whose prototype is similar to:
    * std::string foo (std::string a);
    generates something equivalent to:
    * char* Bar::foo (const char* a)
    * {
    *   return CORBA::String_dup(served_object.foo(std::string(a)).c_str());
    * }

    As of now, interfaces as return types or arguments works only if the interface storage type
    is implicitely constructible from a shared ptr to the interface internal object.

  - a few function names have special behaviour:
    - str   : prints an object using the c++ operator<<

  - Comment are parsed as follows, depending on their first characters. In the
    examples, it is assumed option -k is provided.
    - "// " or "///": ignored.
    - "//*": these comments replaces the default implementation. Example:
      * string foo (in string a);
      * //* return CORBA::String_dup(a);
    - "//->": the word following is understood as the function name of the HPP object.
      * Distance getDistance () raises (Error);
      * //-> distance
      * void setDistance (in Distance d) raises (Error);
      * //-> distance
    - to come: "//-" and "//+": the comment is insert as code in the default implementation.
      Usefull in order to check the inputs.

  - to define your own storage class. For class Foo, define a struct FooStorage
    * #ifdef __OMNIIDL_CXX_IMPL__
    * interface Robot;
    * struct FooStorage {
    * Robot r;
    * };
    * #endif // __OMNIIDL_CXX_IMPL__

    The option -nf is useful in that context.
  """
unused_usage_string =  """\
  -Wbs=<suffix>     Specify suffix for generated stub files
  -Wba              Generate code for TypeCodes and Any
  -Wbtp             Generate 'tie' implementation skeletons
  -Wbtf             Generate flattened 'tie' implementation skeletons
  -Wbsplice-modules Splice together multiply opened modules into one 
  -Wbexample        Generate example implementation code
  -WbF              Generate code fragments (for experts only)
  -WbBOA            Generate BOA compatible skeletons
  -Wbold            Generate old CORBA 2.1 signatures for skeletons
  -Wbold_prefix     Map C++ reserved words with prefix _ instead of _cxx_
  -Wbinline         Generate code for #included files inline with the main file
  -Wbkeep_inc_path  Preserve IDL #include path in header #includes
  -Wbuse_quotes     Use quotes in #includes: "foo" rather than <foo>
  -Wbdll_includes   Extra support for #included IDL in DLLs
  -Wbvirtual_objref Use virtual functions in object references
  -Wbimpl_mapping   Use 'impl' mapping for object reference methods"""

# Encountering an unknown AST node will cause an AttributeError exception
# to be thrown in one of the visitors. Store a list of those not-supported
# so we can produce a friendly error message later.
AST_unsupported_nodes = [ "Native" ]

try:
    # omniidl 4.1.6
    config.state._config['HPP Suffix'] = ".hpp"
    config.state._config['HXX Suffix'] = ".hxx"
    config.state._config['CC Suffix']  = ".cc"
    config.state._config['Include Prefix']  = ""
except AttributeError: #'dict' object has no attribute '_config'
    # omniidl 4.2.2
    config.state['HPP Suffix'] = ".hpp"
    config.state['HXX Suffix'] = ".hxx"
    config.state['CC Suffix']  = ".cc"
    config.state['Include Prefix']  = ""

def process_args(args):
    for arg in args:
        if False:
            pass
        #if arg == "a":
        #    config.state['Typecode']          = 1
        #elif arg == "tp":
        #    config.state['Normal Tie']        = 1
        #elif arg == "tf":
        #    config.state['Flattened Tie']     = 1
        #elif arg == "splice-modules":
        #    config.state['Splice Modules']    = 1
        #elif arg == "example":
        #    config.state['Example Code']      = 1
        #elif arg == "F":
        #    config.state['Fragment']          = 1
        #elif arg == "BOA":
        #    config.state['BOA Skeletons']     = 1
        #elif arg == "old":
        #    config.state['Old Signatures']    = 1
        #elif arg == "old_prefix":
        #    config.state['Reserved Prefix']   = "_"
        #elif arg == "keep_inc_path":
        #    config.state['Keep Include Path'] = 1
        #elif arg == "use_quotes":
        #    config.state['Use Quotes']        = 1
        #elif arg == "virtual_objref":
        #    config.state['Virtual Objref Methods'] = 1
        #elif arg == "impl_mapping":
        #    config.state['Impl Mapping'] = 1
        #elif arg == "debug":
        #    config.state['Debug']             = 1
        elif arg[:2] == "h=":
            config.state['HH Suffix']         = arg[2:]
        elif arg[:3] == "hh=":
            config.state['HPP Suffix']        = arg[3:]
        elif arg[:2] == "i=":
            config.state['HXX Suffix']        = arg[2:]
        elif arg[:2] == "c=":
            config.state['CC Suffix']         = arg[2:]
        #elif arg[:2] == "s=":
        #    config.state['SK Suffix']         = arg[2:]
        #elif arg[:2] == "d=":
        #    config.state['DYNSK Suffix']      = arg[2:]
        #elif arg[:2] == "e=":
        #    config.state['IMPL Suffix']       = arg[2:]
        #elif arg == "inline":
        #    config.state['Inline Includes']   = 1
        #elif arg == "shortcut":
        #    config.state['Shortcut']          = 1
        #elif arg[:9] == "shortcut=":
        #    if arg[9:] == "refcount":
        #        config.state['Shortcut']      = 2
        #    elif arg[9:] == "simple":
        #        config.state['Shortcut']      = 1
        #    else:
        #        util.fatalError('Unknown shortcut option "%s"' % arg[9:])
        #elif arg == "dll_includes":
        #    config.state['DLLIncludes']       = 1
        elif arg[:len('guard_prefix=')] == "guard_prefix=":
            config.state['GuardPrefix']       = arg[len('guard_prefix='):]
        elif arg[:len('inc_prefix=')]   == "inc_prefix=":
            config.state['Include Prefix']    = arg[len('inc_prefix='):]
        else:
            util.fatalError("Argument \"" + str(arg) + "\" is unknown")

def run(tree, backend_args):
    # Process arguments
    dirname, filename = os.path.split(tree.file())
    basename,ext      = os.path.splitext(filename)
    config.state['Basename']  = basename
    config.state['Directory'] = dirname

    process_args(backend_args)

    try:
        # Check the input tree only contains stuff we understand
        support.checkIDL(tree)

        # initialise the handy ast module
        ast.__init__(tree)

        # Initialise the descriptor generating code
        descriptor.__init__(tree)

        # Build the map of AST nodes to Environments
        tree.accept(id.WalkTree())


        hh_filename  = config.state['Basename'] + config.state['HH Suffix']
        hpp_filename = config.state['Basename'] + config.state['HPP Suffix']
        hxx_filename = config.state['Basename'] + config.state['HXX Suffix']
        cc_filename  = config.state['Basename'] + config.state['CC Suffix']
        if config.state["Include Prefix"]:
            prefix = config.state['Include Prefix'] + '/'
        else:
            prefix = ""

        idl_filename = tree.file()

        hpp_stream = output.Stream(output.createFile(hpp_filename), 2)
        hxx_stream = output.Stream(output.createFile(hxx_filename), 2)
        cc_stream  = output.Stream(output.createFile( cc_filename), 2)
        main.self = main
        main.__init__(hpp_stream, hxx_stream, cc_stream,
                idl_filename,
                prefix, hh_filename, hpp_filename, hxx_filename)

        main.run(tree)

        hpp_stream .close()
        hxx_stream.close()
        cc_stream .close()

    except AttributeError as e:
        name = e.args[0]
        unsupported_visitors = map(lambda x:"visit" + x,
                                   AST_unsupported_nodes[:])
        if name in unsupported_visitors:
            # delete all possibly partial output files
            for file in output.listAllCreatedFiles():
                os.unlink(file)

            util.unsupportedIDL()
            
        raise

    except SystemExit as e:
        # fatalError function throws SystemExit exception
        # delete all possibly partial output files
        for file in output.listAllCreatedFiles():
            os.unlink(file)
        
        raise
