# Copyright (c) 2012, 2020 CNRS
# Author: Florent Lamiraux, Guilhem Saurel
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

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(PROJECT_NAME hpp-corbaserver)
SET(PROJECT_DESCRIPTION "Corba server for Humanoid Path Planner applications")

SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)
SET(CXX_DISABLE_WERROR true)

INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/idl.cmake)
INCLUDE(cmake/python.cmake)

INCLUDE(cmake-modules/omniidl.cmake)

SET(CLIENT_ONLY FALSE CACHE BOOL "Set to true to install the client only")

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

# Dependencies
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.4")
ADD_REQUIRED_DEPENDENCY("omniDynamic4 >= 4.1.4")
FINDPYTHON()
IF(NOT CLIENT_ONLY)
  ADD_PROJECT_DEPENDENCY("hpp-core" REQUIRED)
  ADD_PROJECT_DEPENDENCY("hpp-template-corba" REQUIRED)
ENDIF(NOT CLIENT_ONLY)

set(ALL_IDL_CPP_STUBS "")
set(ALL_IDL_CPP_IMPL_STUBS "")
set(ALL_IDL_PYTHON_STUBS "")

SET(${PROJECT_NAME}_HEADERS
  include/hpp/corbaserver/client.hh
  )

IF(NOT CLIENT_ONLY)
  SET(${PROJECT_NAME}_HEADERS
    ${${PROJECT_NAME}_HEADERS}
    include/hpp/corbaserver/doc.hh
    include/hpp/corbaserver/fwd.hh
    include/hpp/corbaserver/object-map.hh
    include/hpp/corbaserver/problem-solver-map.hh
    include/hpp/corbaserver/servant-base.hh
    include/hpp/corbaserver/server.hh
    include/hpp/corbaserver/server-plugin.hh
    include/hpp/corbaserver/conversions.hh
    )
ENDIF(NOT CLIENT_ONLY)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

SET(HPP_CORBASERVER_CMAKE_PLUGIN "${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}-plugin.cmake")
SET(HPP_CORBASERVER_OMNIIDL_CMAKE "${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME}/omniidl.cmake")
SET(PKG_CONFIG_EXTRA "${PKG_CONFIG_EXTRA}\ncmake_plugin=${HPP_CORBASERVER_CMAKE_PLUGIN}")
SET(PACKAGE_EXTRA_MACROS
    "${PACKAGE_EXTRA_MACROS}\nset(HPP_CORBASERVER_CMAKE_PLUGIN ${HPP_CORBASERVER_CMAKE_PLUGIN})")
SET(PACKAGE_EXTRA_MACROS
    "${PACKAGE_EXTRA_MACROS}\nset(HPP_CORBASERVER_OMNIIDL_CMAKE ${HPP_CORBASERVER_OMNIIDL_CMAKE})")
SET(PACKAGE_EXTRA_MACROS
    "${PACKAGE_EXTRA_MACROS}\nset(HPP_CORBASERVER_DATAROOTDIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR})")
INSTALL(FILES cmake-modules/${PROJECT_NAME}-plugin.cmake
              cmake-modules/omniidl.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}/
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

# install omniidl backend
INSTALL(FILES
  cmake-modules/omniidl/cxx_impl/__init__.py
  cmake-modules/omniidl/cxx_impl/main.py
  cmake-modules/omniidl/cxx_impl/template.py
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/omniidl/cxx_impl
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
