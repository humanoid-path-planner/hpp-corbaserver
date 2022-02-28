// Copyright (c) 2015, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

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
