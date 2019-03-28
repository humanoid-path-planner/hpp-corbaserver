// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include "hpp/corbaserver/core_idl/steering-methods.hh"

namespace hpp
{
  namespace corbaServer
  {
    template <>
    std::vector< ServantFactoryBase<core_idl::SteeringMethod>* >&
    objectDowncasts<core_idl::SteeringMethod> ()
    {
      static std::vector< ServantFactoryBase<core_idl::SteeringMethod>* > vector;
      return vector;
    }

    namespace core_idl
    {
      HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(SteeringMethod, SteeringMethod, 0)
    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.
