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

set(LIBRARY_NAME ${PROJECT_NAME})
set(IDL_SOURCES obstacle problem robot tools)

omniidl_include_directories(${CMAKE_SOURCE_DIR}/idl)
include_directories(${CMAKE_BINARY_DIR}/src)

make_directory(${CMAKE_BINARY_DIR}/src/hpp/corbaserver)
foreach(IDL ${IDL_SOURCES})
  generate_idl_cpp(
    hpp/corbaserver/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver
    -Wbguard_prefix=hpp_corbaserver_idl HEADER_SUFFIX -idl.hh)
  generate_idl_python(
    ${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver
    ENABLE_DOCSTRING
    STUBS
    hpp_stubs.corbaserver
    ARGUMENTS
    -Wbmodules=hpp_idl
    -Wbextern=common:hpp_stubs
    -Wbextern=robots:hpp_stubs.pinocchio
    -Wbextern=constraints:hpp_stubs.constraints
    -Wbextern=distances:hpp_stubs.core
    -Wbextern=paths:hpp_stubs.core
    -Wbextern=steering_methods:hpp_stubs.core
    -Wbextern=path_projectors:hpp_stubs.core
    -Wbextern=path_validations:hpp_stubs.core
    -Wbextern=path_planners:hpp_stubs.core
    -Wbextern=_constraints:hpp_stubs.core
    -Wbextern=configuration_shooters:hpp_stubs.core
    -Wbextern=_problem:hpp_stubs.core)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/${IDL}-idl.hh
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/corbaserver)
endforeach()
# Enable generation of dynamic type codes (in file commonDynSK.cc)
generate_idl_cpp(hpp/common ${CMAKE_SOURCE_DIR}/idl/hpp ENABLE_Wba
                 HEADER_SUFFIX -idl.hh)
generate_idl_python(
  common
  ${CMAKE_SOURCE_DIR}/idl/hpp
  ENABLE_DOCSTRING
  STUBS
  hpp_stubs
  ARGUMENTS
  -Wbmodules=hpp_idl)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/common-idl.hh
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp)

make_directory(${CMAKE_BINARY_DIR}/src/hpp_idl)
make_directory(${CMAKE_BINARY_DIR}/src/hpp/core_idl)
foreach(
  IDL
  configuration_shooters
  _constraints
  distances
  paths
  path_planners
  path_projectors
  path_validations
  _problem
  steering_methods)
  generate_idl_cpp(hpp/core_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/core_idl
                   ARGUMENTS -Wbguard_prefix=hpp_core_idl HEADER_SUFFIX -idl.hh)
  generate_idl_cpp_impl(
    hpp/core_idl/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/core_idl
    ARGUMENTS
    -Wbguard_prefix=hpp_core_idl
    -Wbinc_prefix=hpp/core_idl
    HH_SUFFIX
    -idl.hh)
  generate_idl_python(
    ${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/core_idl
    ENABLE_DOCSTRING
    STUBS
    hpp_stubs.core
    ARGUMENTS
    -Wbmodules=hpp_idl
    -Wbextern=constraints:hpp_stubs.constraints
    -Wbextern=robots:hpp_stubs.pinocchio
    -Wbextern=common:hpp_stubs)
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/core_idl/${IDL}-idl.hh
          ${CMAKE_CURRENT_BINARY_DIR}/hpp/core_idl/${IDL}.hh
          ${CMAKE_CURRENT_BINARY_DIR}/hpp/core_idl/${IDL}-fwd.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/core_idl)
endforeach()

make_directory(${CMAKE_BINARY_DIR}/src/hpp/constraints_idl)
foreach(IDL constraints)
  generate_idl_cpp(
    hpp/constraints_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/constraints_idl
    ARGUMENTS -Wbguard_prefix=hpp_constraints_idl HEADER_SUFFIX -idl.hh)
  generate_idl_cpp_impl(
    hpp/constraints_idl/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/constraints_idl
    ARGUMENTS
    -Wbguard_prefix=hpp_constraints_idl
    -Wbinc_prefix=hpp/constraints_idl
    HH_SUFFIX
    -idl.hh)
  generate_idl_python(
    ${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/constraints_idl
    ENABLE_DOCSTRING
    STUBS
    hpp_stubs.constraints
    ARGUMENTS
    -Wbmodules=hpp_idl
    -Wbextern=common:hpp_stubs)
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/constraints_idl/${IDL}-idl.hh
          ${CMAKE_CURRENT_BINARY_DIR}/hpp/constraints_idl/${IDL}.hh
          ${CMAKE_CURRENT_BINARY_DIR}/hpp/constraints_idl/${IDL}-fwd.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/constraints_idl)
endforeach()

make_directory(${CMAKE_BINARY_DIR}/src/hpp/pinocchio_idl)
foreach(IDL robots)
  generate_idl_cpp(
    hpp/pinocchio_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/pinocchio_idl
    ARGUMENTS -Wbguard_prefix=hpp_pinocchio_idl HEADER_SUFFIX -idl.hh)
  generate_idl_cpp_impl(
    hpp/pinocchio_idl/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/pinocchio_idl
    ARGUMENTS
    -Wbguard_prefix=hpp_pinocchio_idl
    -Wbinc_prefix=hpp/pinocchio_idl
    HH_SUFFIX
    -idl.hh)
  generate_idl_python(
    ${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/pinocchio_idl
    ENABLE_DOCSTRING
    STUBS
    hpp_stubs.pinocchio
    ARGUMENTS
    -Wbmodules=hpp_idl
    -Wbextern=common:hpp_stubs)
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/pinocchio_idl/${IDL}-idl.hh
          ${CMAKE_CURRENT_BINARY_DIR}/hpp/pinocchio_idl/${IDL}.hh
          ${CMAKE_CURRENT_BINARY_DIR}/hpp/pinocchio_idl/${IDL}-fwd.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/pinocchio_idl)
endforeach()

install(DIRECTORY ${CMAKE_SOURCE_DIR}/idl/hpp
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/idl)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_idl
                  ${CMAKE_CURRENT_BINARY_DIR}/hpp_stubs
        DESTINATION ${PYTHON_SITELIB})

add_custom_target(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS}
                                           ${ALL_IDL_CPP_IMPL_STUBS})
add_custom_target(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})

if(NOT CLIENT_ONLY)
  add_library(
    ${LIBRARY_NAME} SHARED
    object-map.cc
    obstacle.impl.cc
    obstacle.impl.hh
    problem.impl.cc
    problem.impl.hh
    robot.impl.cc
    robot.impl.hh
    client.cc
    server.cc
    problem-solver-map.cc
    tools.cc
    tools.hh
    conversions.cc
    ${ALL_IDL_CPP_STUBS}
    ${ALL_IDL_CPP_IMPL_STUBS})

  target_link_libraries(
    ${LIBRARY_NAME} PUBLIC hpp-core::hpp-core
                           hpp-template-corba::hpp-template-corba)
  target_include_directories(
    ${LIBRARY_NAME} PUBLIC $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  pkg_config_use_dependency(${LIBRARY_NAME} omniORB4)
  pkg_config_use_dependency(${LIBRARY_NAME} omniDynamic4)

  install(
    TARGETS ${LIBRARY_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)

  # Standalone corba server
  add_executable(hppcorbaserver hpp-corbaserver.cc)
  target_link_libraries(hppcorbaserver PUBLIC ${LIBRARY_NAME})
  pkg_config_use_dependency(hppcorbaserver omniORB4)

  install(TARGETS hppcorbaserver DESTINATION bin)
else(NOT CLIENT_ONLY)
  add_library(${LIBRARY_NAME} SHARED client.cc ${ALL_IDL_CPP_STUBS})

  pkg_config_use_dependency(${LIBRARY_NAME} omniORB4)

  install(
    TARGETS ${LIBRARY_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif(NOT CLIENT_ONLY)
target_include_directories(
  ${LIBRARY_NAME} PUBLIC $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

add_dependencies(${LIBRARY_NAME} generate_idl_cpp)
add_dependencies(${LIBRARY_NAME} generate_idl_python)

foreach(FILE __init__.py quaternion.py transform.py utils.py rostools.py)
  python_install_on_site(hpp ${FILE})
endforeach()

foreach(
  FILE
  __init__.py
  client.py
  robot.py
  problem_solver.py
  tools.py
  benchmark.py
  ompl_benchmark_statistics.py)
  python_install_on_site(hpp/corbaserver ${FILE})
endforeach()
