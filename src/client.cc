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

    Client::Client(int argc, char *argv[])
    {
      orb_ = CORBA::ORB_init (argc, argv);
    }

    void Client::connect (const char* iiop, const char* context)
    {
      // Get a reference to the Naming Service
      CORBA::Object_var rootContextObj = orb_->string_to_object(iiop);
      CosNaming::NamingContext_var nc =
        CosNaming::NamingContext::_narrow(rootContextObj.in());

      // Bind robotObj with name Robot to the hppContext:
      CosNaming::Name objectName;
      objectName.length(2);
      objectName[0].id   = (const char*) "hpp"; // string copied
      objectName[0].kind = (const char*) context; // string copied
      objectName[1].id   = (const char*) "basic";   // string copied
      objectName[1].kind = (const char*) "robot"; // string copied

      // Invoke the root context to retrieve the object reference
      CORBA::Object_var managerObj = nc->resolve(objectName);
      // Narrow the previous object to obtain the correct type
      robot_ = hpp::corbaserver::Robot::_narrow (managerObj.in ());

      // Bind obstacleObj with name Obstacle to the hppContext:
      objectName[1].id   = (const char*) "basic"; // string copied
      objectName[1].kind = (const char*) "problem";   // string copied

      managerObj = nc->resolve(objectName);
      // Narrow the previous object to obtain the correct type
      problem_ = hpp::corbaserver::Problem::_narrow(managerObj.in());

      // Bind problemObj with name Problem to the hppContext:
      objectName[1].id   = (const char*) "basic"; // string copied
      objectName[1].kind = (const char*) "obstacle";   // string copied

      managerObj = nc->resolve(objectName);
      // Narrow the previous object to obtain the correct type
      obstacle_ = hpp::corbaserver::Obstacle::_narrow(managerObj.in());
    }

    /// \brief Shutdown CORBA server
    Client::~Client()
    {
    }
  } // end of namespace corbaServer.
} // end of namespace hpp.
