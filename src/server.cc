// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include "hpp/corbaserver/server.hh"

#include <errno.h>
#include <pthread.h>
#include <iostream>
#include <dlfcn.h>

#include <hpp/util/debug.hh>
#include <hpp/core/plugin.hh>
#include <hpp/corbaserver/server-plugin.hh>
#include "hpp/corbaserver/tools-idl.hh"

#include "basic-server.hh"
#include "hpp/corbaserver/servant-base.hh"

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

    namespace
    {
      /// \brief Forward logging messages to hpp logging mechanism.
      /// If debug is disabled, CORBA logging will be disabled too.
      ///
      /// Tracing has to be enabled in your ``omniORB.cfg'' to use this
      /// feature.
      /// See ``omniORB configuration and API'' > ``Tracing options''
      /// section of omniORB manual for more information.
      void logFunction (const char* msg);

      void logFunction (const char* hppDebugStatement (msg))
      {
	hppDout (info, "omniORB: " << msg);
      }

      void usage (const char* app)
      {
        std::cerr << "Usage: " << app << " [options] ..." << '\n'
                  << "  --name <name>" << '\n'
                  << "  --help" << '\n'
                  << "  --single-thread" << '\n'
                  << "  --multi-thread"  << std::endl;
      }
    } // end of anonymous namespace.

    class Tools : public virtual POA_hpp::Tools
    {
      public:
	Tools () : server_ (NULL) {};

        void setServer (Server* server)
        {
          server_ = server;
        }

        virtual CORBA::Boolean loadServerPlugin (const char* context, const char* pluginName) throw (Error)
        {
          try {
            std::string c (context), pn (pluginName);
            return server_->loadPlugin (c, pn);
          } catch (const std::exception& e) {
            throw hpp::Error (e.what ());
          }
        }

        virtual CORBA::Boolean createContext (const char* context) throw (Error)
        {
          try {
            std::string c (context);
            return server_->createContext (c);
          } catch (const std::exception& e) {
            throw hpp::Error (e.what ());
          }
        }

        virtual void deleteServant (const char* id) throw (Error)
        {
          try {
            CORBA::Object_ptr obj = server_->orb()->string_to_object (id);
            PortableServer::ObjectId_var objectId = server_->poa()->reference_to_id (obj);
            // Remove reference in servant object map
            ServantBase_var servant = server_->poa()->id_to_servant(objectId.in());
            server_->removeServant (servant.in());
            // Deactivate object
            server_->poa()->deactivate_object(objectId.in());
          } catch (const std::exception& e) {
            throw hpp::Error (e.what ());
          }
        }

        virtual void shutdown ()
        {
          server_->requestShutdown(false);
        }

      private:
        Server* server_;
    };

    Server::Server(core::ProblemSolverPtr_t problemSolver, int argc,
		   const char *argv[], bool inMultiThread) :
      multiThread_ (inMultiThread),
      problemSolverMap_ (new ProblemSolverMap (problemSolver))
    {
      // Register log function.
      omniORB::setLogFunction (&logFunction);

      parseArguments (argc, argv);

      initORBandServers (argc, argv, multiThread_);
    }

    Server::Server(ProblemSolverMapPtr_t problemSolverMap, int argc,
		   const char *argv[], bool inMultiThread) :
      multiThread_ (inMultiThread),
      problemSolverMap_ (problemSolverMap)
    {
      // Register log function.
      omniORB::setLogFunction (&logFunction);

      parseArguments (argc, argv);

      initORBandServers (argc, argv, multiThread_);
    }

    /// \brief Shutdown CORBA server
    Server::~Server()
    {
    }

    /// CORBA SERVER INITIALIZATION

    void Server::initORBandServers(int argc, const char* argv[],
				   bool inMultiThread)
    {
      Object_var obj;
      PortableServer::ThreadPolicy_var threadPolicy;
      PortableServer::POA_var rootPoa;

      /// ORB init
      orb_ = ORB_init (argc, const_cast<char **> (argv));
      if (is_nil(orb_)) {
	std::string msg ("failed to initialize ORB");
	hppDout (error, msg.c_str ());
	throw std::runtime_error (msg.c_str ());
      }
      /// ORB init
      obj = orb_->resolve_initial_references("RootPOA");

      /// Create thread policy

      //
      // Make the CORBA object single-threaded to avoid GUI krash
      //
      // Create a sigle threaded policy object
      rootPoa = PortableServer::POA::_narrow(obj);

      if (inMultiThread) {
	threadPolicy = rootPoa->create_thread_policy
	  (PortableServer::ORB_CTRL_MODEL);
      }
      else {
	threadPolicy = rootPoa->create_thread_policy
	  (PortableServer::MAIN_THREAD_MODEL);
      }
      /// Duplicate thread policy
      PolicyList policyList;
      policyList.length(1);
      policyList[0] = PortableServer::ThreadPolicy::_duplicate(threadPolicy);

      try {
        poa_ = rootPoa->create_POA
          ("child", PortableServer::POAManager::_nil(), policyList);
      } catch (PortableServer::POA::AdapterAlreadyExists& /*e*/) {
        poa_ = rootPoa->find_POA ("child", false);
      }

      // Destroy policy object
      threadPolicy->destroy();
    }

    void Server::parseArguments (int argc, const char* argv[])
    {
      mainContextId_ = "corbaserver";

      for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--name") == 0) {
          if (i < argc - 1) {
            mainContextId_ = argv[i+1];
            ++i;
          }
          else              usage(argv[0]);
          std::cout << "Server main context: " << mainContextId_ << std::endl;
        } else if (strcmp(argv[i], "--help") == 0) {
          usage(argv[0]);
        } else if (strcmp(argv[i], "--single-thread") == 0) {
          multiThread_ = false;
          std::cout << "Switched to single thread mode." << std::endl;
        } else if (strcmp(argv[i], "--multi-thread") == 0) {
          std::cout << "Switched to multi-thread mode." << std::endl;
          multiThread_ = true;
        }
      }
    }

    void Server::startCorbaServer()
    {
      // Create Server interface
      tools_ = new Tools ();
      tools_->setServer (this);
      /*servantId_ = */ poa_->activate_object(tools_);

      CosNaming::NamingContext_var rootContext;
      Object_var localObj;
      CosNaming::Name contextName;

      try {
        // Obtain a reference to the root context of the Name service:
        localObj = orb_->resolve_initial_references("NameService");
      } catch(const CORBA::Exception& e) {
        hppCorbaDout (error, "CORBA::Exception " << e._name() << ":" 
            << e._rep_id() << " : failed to get the name service");
        return;
      }

      try {
        // Narrow the reference returned.
        rootContext = CosNaming::NamingContext::_narrow(localObj);
        if( is_nil(rootContext) ) {
          hppCorbaDout (error, "Failed to narrow the root naming context.");
          return;
        }
      }
      catch(CORBA::ORB::InvalidName& ex) {
        // This should not happen!
        hppCorbaDout (error, "Service required is invalid [does not exist].");
        return;
      } catch(const CORBA::Exception& e) {
        hppCorbaDout (error, "CORBA::Exception " << e._name() << ":" 
            << e._rep_id() << " : failed to narrow the root naming context");
        return;
      }

      try {
        // Bind a context called "hpp" to the root context:
        localObj = tools_->_this();

        contextName.length(1);
        contextName[0].id   = (const char*) "hpp";    // string copied
        contextName[0].kind = (const char*) "tools"; // string copied
        // Note on kind: The kind field is used to indicate the type
        // of the object. This is to avoid conventions such as that used
        // by files (name.type -- e.g. hpp.ps = postscript etc.)

        try {
          rootContext->bind(contextName, localObj);
        }
        catch(CosNaming::NamingContext::AlreadyBound& ex)
        {
          rootContext->rebind(contextName, localObj);
        }
      }
      catch(CORBA::COMM_FAILURE& ex) {
        hppCorbaDout (error, "Caught system exception COMM_FAILURE -- "
            "unable to contact the naming service.");
        return;
      }
      catch(CORBA::SystemException&) {
        hppCorbaDout (error, "Caught a SystemException while creating the context.");
        return;
      }
      tools_->_remove_ref();

      // Creation of main context
      createContext (mainContextId());
    }

    bool Server::createContext (const std::string& name)
    {
      if (contexts_.find (name) != contexts_.end())
        return false;

      getContext (name);
      return true;
    }

    core::ProblemSolverPtr_t Server::problemSolver ()
    {
      return problemSolverMap_->selected();
    }

    ProblemSolverMapPtr_t Server::problemSolverMap ()
    {
      return problemSolverMap_;
    }

    Server::Context& Server::getContext (const std::string& name)
    {
      if (contexts_.find (name) != contexts_.end()) {
        return contexts_[name];
      }

      Context context;
      context.main = ServerPluginPtr_t (new BasicServer (this));
      ProblemSolverMapPtr_t psm (new ProblemSolverMap (*problemSolverMap_));
      context.main->setProblemSolverMap (psm);
      context.main->startCorbaServer ("hpp", name);

      return contexts_.insert (std::make_pair(name, context)).first->second;
    }

    bool Server::loadPlugin (const std::string& contextName,
        const std::string& libFilename)
    {
      // Load the plugin
      std::string lib = core::plugin::findPluginLibrary (libFilename);

      typedef ::hpp::corbaServer::ServerPlugin* (*PluginFunction_t) (Server*);

      // Clear old errors
      const char* error = dlerror ();
      //void* library = dlopen(lib.c_str(), RTLD_NOW|RTLD_GLOBAL);
      void* library = dlopen(lib.c_str(), RTLD_NOW);
      error = dlerror ();
      if (error != NULL) {
        hppDout (error, "Error loading library " << lib << ": " << error);
        return false;
      }
      if (library == NULL) {
        // uncaught error ?
        return false;
      }

      PluginFunction_t function = reinterpret_cast<PluginFunction_t>(dlsym(library, "createServerPlugin"));
      error = dlerror ();
      if (error != NULL) {
        hppDout (error, "Error loading library " << lib << ":\n" << error);
        return false;
      }
      if (function == NULL) {
        hppDout (error, "Symbol createServerPlugin of (correctly loaded) library " << lib << " is NULL.");
        return false;
      }

      // Get the context.
      Context& context = getContext (contextName);

      ServerPluginPtr_t plugin (function(this));
      if (!plugin) return false;
      const std::string name = plugin->name();
      if (context.plugins.find (name) != context.plugins.end()) {
        hppDout (info, "Plugin " << lib << " already loaded.");
        return false;
      }
      context.plugins[name] = plugin;
      plugin->setProblemSolverMap (context.main->problemSolverMap());
      plugin->startCorbaServer ("hpp", contextName);

      // I don't think we should do that because the symbols should not be removed...
      // dlclose (library);

      return true;
    }

    /// \brief If CORBA requests are pending, process them
    int Server::processRequest (bool loop)
    {
      if (loop)
	{
	  hppDout (info, "start processing CORBA requests for ever.");
	  orb_->run();
	}
      else
	{
	  if (orb_->work_pending())
	    orb_->perform_work();
	}
      return 0;
    }

    void Server::requestShutdown (bool wait)
    {
      orb_->shutdown (wait);
    }

    PortableServer::Servant Server::getServant (ServantKey servantKey) const
    {
      ServantKeyToServantMap_t::const_iterator _servant
        = servantKeyToServantMap_.find (servantKey);
      if (_servant == servantKeyToServantMap_.end())
        return NULL;
      return _servant->second;
    }

    void Server::addServantKeyAndServant (ServantKey servantKey, PortableServer::Servant servant)
    {
      typedef std::pair<ServantToServantKeyMap_t::iterator, bool> Ret_t;
      Ret_t ret = servantToServantKeyMap_.insert (std::make_pair(servant, servantKey));
      if (!ret.second) // Object not added because it already exists
      {
        std::cerr << "An servant object was lost." << std::endl;
        ret.first->second = servantKey;
      }

      typedef std::pair<ServantKeyToServantMap_t::iterator, bool> Ret2_t;
      Ret2_t ret2 = servantKeyToServantMap_.insert (std::make_pair(servantKey, servant));
      if (!ret2.second)
        ret2.first->second = servant;
    }

    void Server::removeServant (PortableServer::Servant servant)
    {
      ServantToServantKeyMap_t::iterator _it = servantToServantKeyMap_.find(servant);
      if (_it == servantToServantKeyMap_.end()) return;
      servantKeyToServantMap_.erase (_it->second);
      servantToServantKeyMap_.erase (_it);
    }

  } // end of namespace corbaServer.
} // end of namespace hpp.
