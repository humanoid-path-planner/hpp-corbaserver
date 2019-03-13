// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

# include "paths.hh"

namespace hpp
{
  namespace corbaServer
  {
    template <>
    std::vector< ServantFactoryBase<core_idl::Path>* >&
    objectDowncasts<core_idl::Path> ()
    {
      static std::vector< ServantFactoryBase<core_idl::Path>* > vector;
      return vector;
    }

    namespace core_idl
    {
      hpp::core_idl::Path_ptr makePathServant (Server* server, const PathPtr_t& path)
      {
        hpp::core_idl::Path_var d = makeServantDownCast<Path> (server, path);
        return d._retn();
      }

      HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(Path      , Path, 0)
      HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(PathVector, Path, 1)
    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.
