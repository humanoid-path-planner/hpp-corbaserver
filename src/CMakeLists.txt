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
  common
  obstacle
  problem
  robot
  tools
  )

# search for python
FINDPYTHON(2.7 EXACT REQUIRED)

OMNIIDL_INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/idl)
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)

FOREACH(IDL obstacle problem robot tools)
  GENERATE_IDL_CPP (hpp/corbaserver/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver)
  GENERATE_IDL_PYTHON (hpp/corbaserver/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver
    ARGUMENTS
    -p${CMAKE_SOURCE_DIR}/cmake/hpp/idl
    -bomniidl_be_python_with_docstring
    -K
    )
ENDFOREACH()
# Enable generation of dynamic type codes (in file commonDynSK.cc)
GENERATE_IDL_CPP (hpp/corbaserver/common ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver ENABLE_Wba)
GENERATE_IDL_PYTHON (hpp/corbaserver/common ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver
    ARGUMENTS
    -p${CMAKE_SOURCE_DIR}/cmake/hpp/idl
    -bomniidl_be_python_with_docstring
    -K
  )

FOREACH(IDL ${IDL_SOURCES})
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/${IDL}.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/corbaserver)
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/${IDL}_idl.py
    DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver
    )
  INSTALL(FILES ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/${IDL}.idl
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/idl/hpp/corbaserver)
ENDFOREACH()

ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS})
ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})

# "omniidl -bpython" generates an hpp.corbaserver python module inside the hpp.corbaserver python module...
ADD_CUSTOM_TARGET(clean_idl_python
  COMMAND ${CMAKE_COMMAND} -E remove_directory "${PROJECT_BINARY_DIR}/src/hpp/corbaserver/hpp"
  DEPENDS generate_idl_python
  )

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
    server-private.cc
    server-private.hh
    problem-solver-map.cc
    tools.cc
    tools.hh
    conversions.cc
    ${ALL_IDL_CPP_STUBS}
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
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/common.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/problem.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/obstacle.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/robot.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/commonSK.cc
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/obstacleSK.cc
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/robotSK.cc
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/problemSK.cc
    )

  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniORB4)

  INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
ENDIF (NOT CLIENT_ONLY)

ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_cpp)
ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_python)
ADD_DEPENDENCIES (${LIBRARY_NAME} clean_idl_python)

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

INSTALL(
  FILES
  ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/common.idl
  ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/obstacle.idl
  ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/problem.idl
  ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/robot.idl
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/idl/hpp/corbaserver
)
