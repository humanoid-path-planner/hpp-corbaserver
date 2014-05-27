// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_SERVER_PRIVATE_HH
# define HPP_CORBASERVER_SERVER_PRIVATE_HH
# include "hpp/corbaserver/fwd.hh"

# include "robot.impl.hh"
# include "obstacle.impl.hh"
# include "problem.impl.hh"
# include "precomputation.impl.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      class Server
      {
      public:
	~Server ();

	/// \brief Create and activate the Corba servers.
	void createAndActivateServers (corbaServer::Server* server);

      private:
	CORBA::ORB_var orb_;
	PortableServer::POA_var poa_;

	/// \brief Implementation of object hpp::Robot
	Robot* robotServant_;
	/// \brief Implementation of object hpp::Obstacle
	Obstacle* obstacleServant_;
	/// \brief Implementation of object hpp::Problem.
	Problem* problemServant_;
	/// \brief Implementation of object hpp::Precomputation.
	Precomputation* precomputationServant_;

	/// \brief It seems that we need to store this object to
	/// deactivate the server.
	PortableServer::ObjectId* robotServantid_;
	PortableServer::ObjectId* obstacleServantid_;
	PortableServer::ObjectId* problemServantid_;
	PortableServer::ObjectId* precomputationServantid_;

	/// \brief Corba context.
	CosNaming::NamingContext_var hppContext_;

	/// \brief Create context.
	void createHppContext ();

	/// \brief Store objects in Corba name service.
	void bindObjectToName
	(CORBA::Object_ptr objref, CosNaming::Name objectName);

	/// \brief Deactivate and destroy servers
	///
	/// Destroying active servers raises a Corba exception.
	void deactivateAndDestroyServers ();

	friend class corbaServer::Server;
      };

    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_SERVER_PRIVATE_HH
