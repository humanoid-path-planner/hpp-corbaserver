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
  )

OMNIIDL_INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/idl)
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)
FOREACH(IDL ${IDL_SOURCES})
  GENERATE_IDL_FILE (${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver)
ENDFOREACH()

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  common.hh
  commonSK.cc
  obstacle.impl.cc
  obstacle.impl.hh
  problem.impl.cc
  problem.impl.hh
  robot.impl.cc
  robot.impl.hh
  server.cc
  server-private.cc
  server-private.hh
  tools.cc
  tools.hh
  obstacleSK.cc
  problemSK.cc
  robotSK.cc
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/common_idl.py
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/obstacle_idl.py
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/problem_idl.py
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/robot_idl.py
)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-model-urdf)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniORB4)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

# search for python
FINDPYTHON()

FOREACH (IDL ${IDL_SOURCES})
  ADD_CUSTOM_COMMAND(
    OUTPUT
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/${IDL}_idl.py
    COMMAND ${OMNIIDL}
    ARGS -bpython -Wbpackage="" -I${CMAKE_SOURCE_DIR}/idl
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/${IDL}.idl
    MAIN_DEPENDENCY
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/${IDL}.idl
    )
  SET_SOURCE_FILES_PROPERTIES(
    ${CMAKE_CURRENT_BINARY_DIR}/${IDL}_idl.py
    GENERATED=ON
    )
  INSTALL(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${IDL}_idl.py
    DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver
    )
ENDFOREACH()
INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp
  )
INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver
  )
INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/client.py
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/robot.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver
  )

INSTALL(
  FILES
  ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/common.idl
  ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/obstacle.idl
  ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/problem.idl
  ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/robot.idl
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/idl/hpp/corbaserver
)

# Standalone corba server
ADD_EXECUTABLE (hppcorbaserver hpp-corbaserver.cc)
TARGET_LINK_LIBRARIES (hppcorbaserver ${LIBRARY_NAME})
PKG_CONFIG_USE_DEPENDENCY(hppcorbaserver hpp-core)
PKG_CONFIG_USE_DEPENDENCY(hppcorbaserver hpp-model-urdf)
PKG_CONFIG_USE_DEPENDENCY(hppcorbaserver hpp-constraints)

INSTALL (TARGETS hppcorbaserver DESTINATION bin)
