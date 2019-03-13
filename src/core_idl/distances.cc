// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

# include "distances.hh"

namespace hpp
{
  namespace corbaServer
  {
    template <>
    std::vector< ServantFactoryBase<core_idl::Distance>* >&
    objectDowncasts<core_idl::Distance> ()
    {
      static std::vector< ServantFactoryBase<core_idl::Distance>* > vector;
      return vector;
    }

    namespace core_idl
    {
      HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(       Distance, Distance, 0)
      HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(WeighedDistance, Distance, 1)
    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.
