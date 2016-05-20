// Copyright (C) 2015 by Joseph Mirabel
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_CLIENT_HH
# define HPP_CORBASERVER_CLIENT_HH

# include <omniORB4/CORBA.h>

# include <hpp/corbaserver/robot.hh>
# include <hpp/corbaserver/problem.hh>
# include <hpp/corbaserver/obstacle.hh>

# include <hpp/corbaserver/config.hh>

namespace hpp
{
  namespace corbaServer
  {
    class HPP_CORBASERVER_DLLAPI Client
    {
    public:
      Client (int argc, char* argv[]);

      ~Client ();

      void connect (const char* iiop = "corbaloc:rir:/NameService");

      hpp::corbaserver::Robot_var& robot () {
        return robot_;
      }

      hpp::corbaserver::Problem_var& problem () {
        return problem_;
      }

      hpp::corbaserver::Obstacle_var& obstacle () {
        return obstacle_;
      }

    private:
      hpp::corbaserver::Robot_var robot_;
      hpp::corbaserver::Problem_var problem_;
      hpp::corbaserver::Obstacle_var obstacle_;

      CORBA::ORB_var orb_;
    };

  } // end of namespace corbaServer.
} // end of namespace hpp.
#endif
