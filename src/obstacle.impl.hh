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

# include <hpp/core/planner.hh>
# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/obstacle.stub.hh>

/// \brief Implement CORBA interface ``Obstacle''.
namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      class Obstacle : public virtual POA_hpp::Obstacle
      {
      public:
	Obstacle (corbaServer::Server* server);

	Short setObstacles (const char* listName);
	Short addObstacle (const char* polyhedronName);

	virtual Short
	addObstacleConfig
	(const char* polyName, const Configuration& cfg)
	  throw (SystemException);

	virtual Short
	moveObstacleConfig (const char* polyName, const Configuration& cfg)
	  throw (SystemException);

	virtual Short
	createCollisionList
	(const char* listName)
	  throw (SystemException);

	virtual Short
	addPolyToCollList
	(const char* listName, const char* polyhedronName)
	  throw (SystemException);

	virtual Short
	createPolyhedron (const char* polyhedronName)
	  throw (SystemException);

	virtual Short createBox
	(const char* boxName, Double x, Double y, Double z)
	  throw(SystemException);

	virtual Short
	addPoint
	(const char* polyhedronName, Double x, Double y, Double z)
	  throw(SystemException);

	virtual Short
	addTriangle
	(const char* polyhedronName, ULong pt1, ULong pt2, ULong pt3)
	  throw(SystemException);

	virtual Short
	loadModelLoaderObstacle
	(const char* polyName, const char* filename, const char* openHrpPrefix)
	  throw(SystemException);

	/// \brief Comment in interface Obstacle::setVisible.
	virtual Short
	setVisible(const char* polyname, Boolean visible)
	  throw(SystemException);

	/// \brief Comment in interface Obstacle::setTransparent.
	virtual Short
	setTransparent(const char* polyname, Boolean transparent)
	  throw(SystemException);

      private:
	/// \brief map of collision lists in construction.
	std::map<std::string, std::vector<CkcdObjectShPtr> > collisionListMap;

	/// \brief map of polyhedra in construction.
	std::map<std::string, CkppKCDPolyhedronShPtr> polyhedronMap;

	/// \brief Pointer to the ChppciServer owning this object.
	corbaServer::Server* server_;

	/// \brief Pointer to hppPlanner object of hppciServer.
	ChppPlanner* planner_;
      };

    } // end of namespace implementation.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_OBSTACLE_HH
