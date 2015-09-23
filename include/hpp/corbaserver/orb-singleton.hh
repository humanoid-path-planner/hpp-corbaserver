
// Copyright (C) 2015 by Joseph Mirabel
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_ORB_SINGLETON_HH
# define HPP_CORBASERVER_ORB_SINGLETON_HH

# include <iostream>

# include <omniORB4/CORBA.h>
# include <boost/serialization/singleton.hpp>

# include <hpp/corbaserver/config.hh>

namespace hpp
{
  namespace corbaServer
  {
    class HPP_CORBASERVER_DLLAPI OrbSingleton
    {
      public:
        OrbSingleton () : created_ (false) {};

        ~OrbSingleton () {
          if (created_) {
            orb_->destroy();
            std::cout << "Ending CORBA..." << std::endl;
          }
        }

        void create (int argc, char *argv[]) {
          if (!created_) {
            orb_ = CORBA::ORB_init (argc, argv);
            created_ = true;
          }
        }

        CORBA::ORB_var orb_;
      private:
        bool created_;
    };

    typedef boost::serialization::singleton <OrbSingleton> OrbSingletonAccessor;
  } // end of namespace corbaServer.
} // end of namespace hpp.
#endif // HPP_CORBASERVER_ORB_SINGLETON_HH
