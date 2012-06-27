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

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR true)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/idl.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/kineo.cmake)

SET(PROJECT_NAME hpp-corbaserver)
SET(PROJECT_DESCRIPTION "Corba server for Humanoid Path Planner applications")
SET(PROJECT_URL "ssh://trac.laas.fr/git/jrl/corba/hpp-corbaserver")

SET(${PROJECT_NAME}_HEADERS
  include/hpp/corbaserver/doc.hh
  include/hpp/corbaserver/fwd.hh
  include/hpp/corbaserver/server.hh
  )

SETUP_PROJECT ()

# Dependencies
ADD_REQUIRED_DEPENDENCY("hpp-core >= 2.5")
ADD_REQUIRED_DEPENDENCY("hpp-model-urdf >= 0.0.1")
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.4")

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
