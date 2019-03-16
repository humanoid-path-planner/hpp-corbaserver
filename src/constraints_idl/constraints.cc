// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
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
