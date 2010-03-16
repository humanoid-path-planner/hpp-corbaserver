// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_OBSTACLE_IMPL_HH
# define HPP_CORBASERVER_OBSTACLE_IMPL_HH
# include <map>
# include <string>

# include <KineoKCDModel/kppKCDPolyhedron.h>

# include <hppCore/hppPlanner.h>
# include <hpp/corbaserver/obstacle.stub.hh>

/// \brief Implementation CORBA interface Obstacle
namespace hpp
{
  namespace corbaServer
  {
    class Server; //FIXME: move to fwd.hh.

    namespace impl
    {
      using CORBA::Boolean;
      using CORBA::Double;
      using CORBA::Short;
      using CORBA::SystemException;
      using CORBA::ULong;

      class Obstacle : public virtual POA_hpp::Obstacle
      {
      public:
	/// \brief store pointer to ChppPlanner object.
	Obstacle (ChppciServer *inHppciServer);

	/// \brief Comment in interface hppCorbaServer::Obstacle::setObstacles
	Short setObstacles (const char* inListName);

	/// \brief Comment in interface hppCorbaServer::Obstacle::addObstacle
	Short addObstacle (const char* inPolyhedronName);

	/// \brief add an obstacle at the given position.
	virtual Short addObstacleConfig
	(const char* inPolyName, const Configuration& cfg)
	  throw(SystemException);
  
	/// \brief Move obstacle to specified configuration.
	virtual Short
	moveObstacleConfig (const char* inPolyName, const Configuration& cfg)
	  throw(SystemException);
  
	/// \brief Comment in interface Obstacle::createCollisionList.
	virtual Short
	createCollisionList (const char* inListName) throw (SystemException);
  
	/// \brief Comment in interface Obstacle::addPolyToCollList.
	virtual Short
	addPolyToCollList (const char* inListName, const char* inPolyhedronName)
	  throw(SystemException);
  
	/// \brief Comment in interface Obstacle::createPolyhedron.
	virtual Short
	createPolyhedron (const char* inPolyhedronName)
	  throw (SystemException);

	/// \brief Comment in Obstacle::createBox.
	virtual Short createBox(const char* inBoxName, Double x, 
				       Double y, Double z)
	  throw(SystemException);

	/// \brief Comment in interface Obstacle::addPoint.
	virtual Short 
	addPoint(const char* inPolyhedronName, Double x, 
		 Double y, Double z) throw(SystemException);

	/// \brief Comment in interface Obstacle::addTriangle.
	virtual Short 
	addTriangle(const char* inPolyhedronName, ULong pt1, ULong pt2, ULong pt3)
	  throw(SystemException);

	/// \brief Comment in interface Obstacle::loadModelLoaderObstacle.
	virtual Short 
	loadModelLoaderObstacle(const char* inPolyName, const char* inFilename,
				const char* inOpenHrpPrefix)
	  throw(SystemException);

	/// \brief Comment in interface Obstacle::setVisible.
	virtual Short 
	setVisible(const char* inPolyname, Boolean inVisible)
	  throw(SystemException);

	/// \brief Comment in interface Obstacle::setTransparent.
	virtual Short 
	setTransparent(const char* inPolyname, Boolean inTransparent)
	  throw(SystemException);

      private:
	/// \brief map of collision lists in construction.
	std::map<std::string, std::vector<CkcdObjectShPtr> > collisionListMap;

	/// \brief map of polyhedra in construction.
	std::map<std::string, CkppKCDPolyhedronShPtr> polyhedronMap;

	/// \brief Pointer to the ChppciServer owning this object..
	Server* server_;

	/// \brief Pointer to hppPlanner object of hppciServer.
	ChppPlanner* planner_;
      };

    } // end of namespace implementation.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_OBSTACLE_HH
