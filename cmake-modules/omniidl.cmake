# Copyright (c) 2019, Joseph Mirabel
# Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

# Generate servant implementation of the IDL interface for the corresponding HPP class.
#
# Assumes the following has been called before:
# ADD_REQUIRED_DEPENDENCY("hpp-corbaserver")
MACRO(GENERATE_IDL_CPP_IMPL FILENAME DIRECTORY)
  SET(oneValueArgs HH_SUFFIX HPP_SUFFIX HXX_SUFFIX CC_SUFFIX)
  SET(multiValueArgs ARGUMENTS)
  CMAKE_PARSE_ARGUMENTS(_omni "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # Set default values
  IF(NOT DEFINED _omni_HH_SUFFIX )
    SET(_omni_HH_SUFFIX  ".hh" )
  ENDIF()
  IF(NOT DEFINED _omni_HPP_SUFFIX)
    SET(_omni_HPP_SUFFIX "-fwd.hh")
  ENDIF()
  IF(NOT DEFINED _omni_HXX_SUFFIX)
    SET(_omni_HXX_SUFFIX ".hh")
  ENDIF()
  IF(NOT DEFINED _omni_CC_SUFFIX )
    SET(_omni_CC_SUFFIX  ".cc" )
  ENDIF()
  IF(CXX_STANDARD GREATER 10)
    SET(_omni_CXX_STD "-Wc++11")
  ENDIF()

  GET_FILENAME_COMPONENT (_PATH ${FILENAME} PATH)
  GET_FILENAME_COMPONENT (_NAME ${FILENAME} NAME)

  IF (_PATH STREQUAL "")
    SET(_PATH "./")
  ENDIF (_PATH STREQUAL "")
  FIND_PROGRAM(OMNIIDL omniidl)
  IF(${OMNIIDL} STREQUAL OMNIIDL-NOTFOUND)
    MESSAGE(FATAL_ERROR "cannot find omniidl.")
  ENDIF(${OMNIIDL} STREQUAL OMNIIDL-NOTFOUND)

  SET(IDL_COMPILED_FILES)
  FOREACH(suffix HPP_SUFFIX HXX_SUFFIX CC_SUFFIX)
    LIST(APPEND IDL_COMPILED_FILES ${FILENAME}${_omni_${suffix}})
  ENDFOREACH()

  IF(DEFINED HPP_CORBASERVER_DATAROOTDIR
      AND EXISTS ${HPP_CORBASERVER_DATAROOTDIR}/hpp-corbaserver/omniidl/cxx_impl)
    SET(OMNIIDL_CXX_IMPL_BE ${HPP_CORBASERVER_DATAROOTDIR}/hpp-corbaserver/omniidl)
  ELSE()
    SET(OMNIIDL_CXX_IMPL_BE ${CMAKE_SOURCE_DIR}/cmake-modules/omniidl)
  ENDIF()
  SET(_omniidl_args -p${OMNIIDL_CXX_IMPL_BE} -bcxx_impl -k
    -Wbh=${_omni_HH_SUFFIX} -Wbhh=${_omni_HPP_SUFFIX} -Wbi=${_omni_HXX_SUFFIX} -Wbc=${_omni_CC_SUFFIX}
    ${_omni_CXX_STD} ${_OMNIIDL_INCLUDE_FLAG} ${_omni_ARGUMENTS})

  ADD_CUSTOM_COMMAND(
    OUTPUT ${IDL_COMPILED_FILES}
    COMMAND ${OMNIIDL}
    ARGS ${_omniidl_args} -C${_PATH} ${DIRECTORY}/${_NAME}.idl
    MAIN_DEPENDENCY ${DIRECTORY}/${_NAME}.idl
    COMMENT "Generating C++ implementations for ${_NAME}"
    )

  LIST(APPEND ALL_IDL_CPP_IMPL_STUBS ${IDL_COMPILED_FILES})

  # Clean generated files.
  SET_PROPERTY(
    DIRECTORY APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES
    ${IDL_COMPILED_FILES}
    )
  #SET_PROPERTY(SOURCE ${IDL_COMPILED_FILES}
  #APPEND_STRING PROPERTY
  #COMPILE_FLAGS "-Wno-conversion -Wno-cast-qual -Wno-unused-variable -Wno-unused-parameter")

  LIST(APPEND LOGGING_WATCHED_VARIABLES OMNIIDL ALL_IDL_CPP_IMPL_STUBS)
ENDMACRO()
