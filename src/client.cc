// Copyright (c) 2015, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-corbaserver.
// hpp-corbaserver is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-corbaserver is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-corbaserver. If not, see <http://www.gnu.org/licenses/>.

#include "hpp/corbaserver/client.hh"

#include <iostream>

namespace hpp
{
  namespace corbaServer
  {
    using CORBA::Exception;
    using CORBA::Object_var;
    using CORBA::SystemException;
    using CORBA::ORB_init;
    using CORBA::PolicyList;
    using omniORB::fatalException;

    ClientBase::ClientBase(int argc, char *argv[])
    {
      orb_ = CORBA::ORB_init (argc, argv);
    }

    ClientBase::~ClientBase()
    {
    }

    bool ClientBase::createFromDirectLink (const std::string& iiop)
    {
      std::string url = iiop + "/hpp-corbaserver";

      CORBA::Object_var obj = orb_->string_to_object(url.c_str());
      tools_ = hpp::Tools::_narrow (obj.in ());
      return !CORBA::is_nil(tools_);
    }

    bool ClientBase::createFromNameService (const std::string& iiop)
    {
      std::string url = iiop + "/NameService";

      CORBA::Object_var obj = orb_->string_to_object(url.c_str());
      if (CORBA::is_nil(obj)) return false;
      CosNaming::NamingContext_var nc =
        CosNaming::NamingContext::_narrow(obj.in());
      if (CORBA::is_nil(nc)) return false;

      // Bind robotObj with name Robot to the hppContext:
      CosNaming::Name objectName;
      objectName.length(1);
      objectName[0].id   = (const char*) "hpp";   // string copied
      objectName[0].kind = (const char*) "tools"; // string copied

      // Invoke the root context to retrieve the object reference
      obj = nc->resolve(objectName);
      // Narrow the previous object to obtain the correct type
      tools_ = hpp::Tools::_narrow (obj.in ());

      return CORBA::is_nil(tools_);
    }

    void ClientBase::connect (const std::string& iiop)
    {
      bool ok = createFromDirectLink(iiop);
      if (!ok) ok = createFromNameService(iiop);
      if (!ok) throw std::runtime_error ("Could not connect to hpp server at " + iiop);
    }

    Client::Client(int argc, char *argv[])
      : ClientBase (argc, argv)
    {}

    void Client::connect (const char* iiop, const char* context)
    {
      ClientBase::connect (iiop);

      CORBA::Object_var obj;

      obj = tools()->getServer (context, "", "robot");
      robot_ = hpp::corbaserver::Robot::_narrow(obj.in());

      obj = tools()->getServer (context, "", "problem");
      problem_ = hpp::corbaserver::Problem::_narrow(obj.in());

      obj = tools()->getServer (context, "", "obstacle");
      obstacle_ = hpp::corbaserver::Obstacle::_narrow(obj.in());
    }

    Client::~Client()
    {
    }
  } // end of namespace corbaServer.
} // end of namespace hpp.
