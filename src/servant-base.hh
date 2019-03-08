// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef SRC_SERVANT_BASE_HH
# define SRC_SERVANT_BASE_HH

namespace hpp
{
  namespace corbaServer
  {
    template <typename T> class AbstractServantBase
    {
      public:
        virtual ~AbstractServantBase () {}

        virtual T get() = 0;

      protected:
        AbstractServantBase (Server* server) : server_ (server) {}

        Server* server_;
    };
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // SRC_SERVANT_BASE_HH
