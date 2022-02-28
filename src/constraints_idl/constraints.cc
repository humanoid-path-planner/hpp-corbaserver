// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include "hpp/corbaserver/constraints_idl/constraints.hh"

namespace hpp
{
  namespace corbaServer
  {
    template <>
    std::vector< ServantFactoryBase<constraints_idl::DifferentiableFunction>* >&
    objectDowncasts<constraints_idl::DifferentiableFunction> ()
    {
      static std::vector< ServantFactoryBase<constraints_idl::DifferentiableFunction>* > vector;
      return vector;
    }

    template <>
    std::vector< ServantFactoryBase<constraints_idl::Implicit>* >&
    objectDowncasts<constraints_idl::Implicit> ()
    {
      static std::vector< ServantFactoryBase<constraints_idl::Implicit>* > vector;
      return vector;
    }

    namespace constraints_idl
    {
      HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(DifferentiableFunction, DifferentiableFunction, 0)

      HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(Implicit, Implicit, 0)
    } // end of namespace constraints.
  } // end of namespace corbaServer.
} // end of namespace hpp.
