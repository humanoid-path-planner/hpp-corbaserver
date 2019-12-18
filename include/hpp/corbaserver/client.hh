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
# include <string>

# include <hpp/corbaserver/tools-idl.hh>
# include <hpp/corbaserver/robot-idl.hh>
# include <hpp/corbaserver/problem-idl.hh>
# include <hpp/corbaserver/obstacle-idl.hh>

# include <hpp/corbaserver/config.hh>

namespace hpp
{
  namespace corbaServer
  {
    class HPP_CORBASERVER_DLLAPI ClientBase
    {
    public:
      ClientBase (int argc, char* argv[]);

      virtual ~ClientBase ();

      /// Connect to hpp object.
      /// \param iiop address to the server (either the NameService or hpp directly).
      void connect (const std::string& iiop = "corbaloc:iiop:");

      hpp::Tools_var& tools () {
        return tools_;
      }

    private:
      bool createFromDirectLink(const std::string& iiop);
      bool createFromNameService(const std::string& iiop);

      hpp::Tools_var tools_;
      CORBA::ORB_var orb_;
    };

    class HPP_CORBASERVER_DLLAPI Client : public ClientBase
    {
    public:
      Client (int argc, char* argv[]);

      ~Client ();

      /// \copydoc ClientBase::connect
      /// \param context the hpp context name (passed to the server)
      void connect (const char* iiop = "corbaloc:iiop:",
          const char* context = "corbaserver");

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
      void createClientsFromTools ();

      hpp::corbaserver::Robot_var robot_;
      hpp::corbaserver::Problem_var problem_;
      hpp::corbaserver::Obstacle_var obstacle_;

      CORBA::ORB_var orb_;
    };

  } // end of namespace corbaServer.
} // end of namespace hpp.
#endif
