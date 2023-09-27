# Copyright (c) 2012, 2020 CNRS Author: Florent Lamiraux, Guilhem Saurel
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# 1. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME hpp-corbaserver)
set(PROJECT_DESCRIPTION "Corba server for Humanoid Path Planner applications")

set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)
set(CXX_DISABLE_WERROR true)

include(cmake/hpp.cmake)
include(cmake/idl.cmake)
include(cmake/python.cmake)

include(cmake-modules/omniidl.cmake)

set(CLIENT_ONLY
    FALSE
    CACHE BOOL "Set to true to install the client only")

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Dependencies
add_required_dependency("omniORB4 >= 4.1.4")
add_required_dependency("omniDynamic4 >= 4.1.4")
if(NOT CLIENT_ONLY)
  add_project_dependency("hpp-core" REQUIRED)
  add_project_dependency("hpp-template-corba" REQUIRED)
endif(NOT CLIENT_ONLY)

if(NOT FINDPYTHON_ALREADY_CALLED)
  findpython()
endif()

set(ALL_IDL_CPP_STUBS "")
set(ALL_IDL_CPP_IMPL_STUBS "")
set(ALL_IDL_PYTHON_STUBS "")

set(${PROJECT_NAME}_HEADERS include/hpp/corbaserver/client.hh)

if(NOT CLIENT_ONLY)
  set(${PROJECT_NAME}_HEADERS
      ${${PROJECT_NAME}_HEADERS}
      include/hpp/corbaserver/doc.hh
      include/hpp/corbaserver/fwd.hh
      include/hpp/corbaserver/object-map.hh
      include/hpp/corbaserver/problem-solver-map.hh
      include/hpp/corbaserver/servant-base.hh
      include/hpp/corbaserver/server.hh
      include/hpp/corbaserver/server-plugin.hh
      include/hpp/corbaserver/steering-method.hh
      include/hpp/corbaserver/read-write-lock.hh
      include/hpp/corbaserver/conversions.hh)
endif(NOT CLIENT_ONLY)

add_subdirectory(src)
add_subdirectory(tests)

pkg_config_append_libs(${PROJECT_NAME})

set(HPP_CORBASERVER_CMAKE_PLUGIN
    "${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}-plugin.cmake"
)
set(HPP_CORBASERVER_OMNIIDL_CMAKE
    "${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME}/omniidl.cmake")
set(PKG_CONFIG_EXTRA
    "${PKG_CONFIG_EXTRA}\ncmake_plugin=${HPP_CORBASERVER_CMAKE_PLUGIN}")
set(PACKAGE_EXTRA_MACROS
    "${PACKAGE_EXTRA_MACROS}\nset(HPP_CORBASERVER_CMAKE_PLUGIN ${HPP_CORBASERVER_CMAKE_PLUGIN})"
)
set(PACKAGE_EXTRA_MACROS
    "${PACKAGE_EXTRA_MACROS}\nset(HPP_CORBASERVER_OMNIIDL_CMAKE ${HPP_CORBASERVER_OMNIIDL_CMAKE})"
)
set(PACKAGE_EXTRA_MACROS
    "${PACKAGE_EXTRA_MACROS}\nset(HPP_CORBASERVER_DATAROOTDIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR})"
)
install(
  FILES cmake-modules/${PROJECT_NAME}-plugin.cmake cmake-modules/omniidl.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}/
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

# install omniidl backend
install(
  FILES cmake-modules/omniidl/cxx_impl/__init__.py
        cmake-modules/omniidl/cxx_impl/main.py
        cmake-modules/omniidl/cxx_impl/template.py
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/omniidl/cxx_impl
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
