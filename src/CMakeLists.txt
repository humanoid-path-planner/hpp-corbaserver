# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-corbaserver.
# hpp-corbaserver is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-corbaserver is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-corbaserver.  If not, see
# <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})
SET(IDL_SOURCES
  obstacle
  problem
  robot
  tools
  )

OMNIIDL_INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/idl)
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)

FOREACH(IDL ${IDL_SOURCES})
  GENERATE_IDL_CPP (hpp/corbaserver/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver
    HEADER_SUFFIX -idl.hh)
  GENERATE_IDL_PYTHON (${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver
    ENABLE_DOCSTRING
    STUBS hpp_stubs.corbaserver
    ARGUMENTS
    -Wbmodules=hpp_idl
    -Wbextern=common:hpp_stubs
    -Wbextern=constraints:hpp_stubs.constraints
    -Wbextern=distances:hpp_stubs.core
    -Wbextern=paths:hpp_stubs.core
    -Wbextern=steering_methods:hpp_stubs.core
    -Wbextern=path_validations:hpp_stubs.core
    -Wbextern=path_planners:hpp_stubs.core
    -Wbextern=_constraints:hpp_stubs.core
    -Wbextern=configuration_shooters:hpp_stubs.core
    -Wbextern=_problem:hpp_stubs.core
    )
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/${IDL}-idl.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/corbaserver)
ENDFOREACH()
# Enable generation of dynamic type codes (in file commonDynSK.cc)
GENERATE_IDL_CPP (hpp/common ${CMAKE_SOURCE_DIR}/idl/hpp ENABLE_Wba
  HEADER_SUFFIX -idl.hh)
GENERATE_IDL_PYTHON (common ${CMAKE_SOURCE_DIR}/idl/hpp
    ENABLE_DOCSTRING
    STUBS hpp_stubs
    ARGUMENTS
    -Wbmodules=hpp_idl
  )
INSTALL(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/common-idl.hh
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp)

MAKE_DIRECTORY(${CMAKE_BINARY_DIR}/src/hpp_idl)
MAKE_DIRECTORY(${CMAKE_BINARY_DIR}/src/hpp/core_idl)
FOREACH(IDL distances paths steering_methods path_validations _problem configuration_shooters _constraints path_planners)
  GENERATE_IDL_CPP (hpp/core_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/core_idl
    ARGUMENTS
    -Wbguard_prefix=hpp_core_idl
    HEADER_SUFFIX -idl.hh)
  GENERATE_IDL_CPP_IMPL (hpp/core_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/core_idl
    ARGUMENTS
    -Wbguard_prefix=hpp_core_idl
    -Wbinc_prefix=hpp/core_idl
    HH_SUFFIX -idl.hh)
  GENERATE_IDL_PYTHON (${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/core_idl
      ENABLE_DOCSTRING
      STUBS hpp_stubs.core
      ARGUMENTS
      -Wbmodules=hpp_idl
      -Wbextern=constraints:hpp_stubs.constraints
      -Wbextern=robots:hpp_stubs.pinocchio
      -Wbextern=common:hpp_stubs
    )
  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/core_idl/${IDL}-idl.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/core_idl/${IDL}.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/core_idl/${IDL}-fwd.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/core_idl)
ENDFOREACH()

MAKE_DIRECTORY(${CMAKE_BINARY_DIR}/src/hpp/constraints_idl)
FOREACH(IDL constraints)
  GENERATE_IDL_CPP (hpp/constraints_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/constraints_idl
    ARGUMENTS
    -Wbguard_prefix=hpp_constraints_idl
    HEADER_SUFFIX -idl.hh)
  GENERATE_IDL_CPP_IMPL (hpp/constraints_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/constraints_idl
    ARGUMENTS
    -Wbguard_prefix=hpp_constraints_idl
    -Wbinc_prefix=hpp/constraints_idl
    HH_SUFFIX -idl.hh)
  GENERATE_IDL_PYTHON (${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/constraints_idl
      ENABLE_DOCSTRING
      STUBS hpp_stubs.constraints
      ARGUMENTS
      -Wbmodules=hpp_idl
      -Wbextern=common:hpp_stubs
    )
  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/constraints_idl/${IDL}-idl.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/constraints_idl/${IDL}.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/constraints_idl/${IDL}-fwd.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/constraints_idl)
ENDFOREACH()

MAKE_DIRECTORY(${CMAKE_BINARY_DIR}/src/hpp/pinocchio_idl)
FOREACH(IDL robots)
  GENERATE_IDL_CPP (hpp/pinocchio_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/pinocchio_idl
    ARGUMENTS
    -Wbguard_prefix=hpp_pinocchio_idl
    HEADER_SUFFIX -idl.hh)
  GENERATE_IDL_CPP_IMPL (hpp/pinocchio_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/pinocchio_idl
    ARGUMENTS
    -Wbguard_prefix=hpp_pinocchio_idl
    -Wbinc_prefix=hpp/pinocchio_idl
    HH_SUFFIX -idl.hh)
  GENERATE_IDL_PYTHON (${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/pinocchio_idl
      ENABLE_DOCSTRING
      STUBS hpp_stubs.pinocchio
      ARGUMENTS
      -Wbmodules=hpp_idl
      -Wbextern=common:hpp_stubs
    )
  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/pinocchio_idl/${IDL}-idl.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/pinocchio_idl/${IDL}.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/pinocchio_idl/${IDL}-fwd.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/pinocchio_idl)
ENDFOREACH()

INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/idl/hpp
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/idl)

INSTALL(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_idl ${CMAKE_CURRENT_BINARY_DIR}/hpp_stubs
  DESTINATION ${PYTHON_SITELIB}
  )

ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS} ${ALL_IDL_CPP_IMPL_STUBS})
ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})

IF (NOT CLIENT_ONLY)
  ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
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
    ${ALL_IDL_CPP_IMPL_STUBS}
    )

  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-core)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-pinocchio)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniORB4)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniDynamic4)

  INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

  # Standalone corba server
  ADD_EXECUTABLE (hppcorbaserver hpp-corbaserver.cc)
  TARGET_LINK_LIBRARIES (hppcorbaserver ${LIBRARY_NAME})
  PKG_CONFIG_USE_DEPENDENCY(hppcorbaserver hpp-core)
  PKG_CONFIG_USE_DEPENDENCY(hppcorbaserver hpp-pinocchio)
  PKG_CONFIG_USE_DEPENDENCY(hppcorbaserver hpp-constraints)

  INSTALL (TARGETS hppcorbaserver DESTINATION bin)
ELSE (NOT CLIENT_ONLY)
  ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
    client.cc
    ${ALL_IDL_CPP_STUBS}
    )

  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniORB4)

  INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
ENDIF (NOT CLIENT_ONLY)

ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_cpp)
ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_python)

IF(WIN32)
  SET(LINK copy_if_different)
ELSE(WIN32)
  SET(LINK create_symlink)
ENDIF(WIN32)

FOREACH (FILE __init__.py quaternion.py transform.py)
  PYTHON_INSTALL_ON_SITE(
    hpp ${FILE}
    )
ENDFOREACH ()

FOREACH (FILE __init__.py client.py robot.py  problem_solver.py tools.py benchmark.py ompl_benchmark_statistics.py)
  PYTHON_INSTALL_ON_SITE(
    hpp/corbaserver ${FILE}
    )
ENDFOREACH ()
