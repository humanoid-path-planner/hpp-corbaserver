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

#include <hpp/fcl/math/vec_3f.h>

# include <hpp/core/problem-solver.hh>
# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/obstacle.hh>
# include <hpp/corbaserver/object-map.hh>

/// \brief Implement CORBA interface ``Obstacle''.
namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      class Obstacle : public virtual POA_hpp::corbaserver::Obstacle
      {
      public:
	Obstacle (corbaServer::Server* server);

	virtual void loadObstacleModel (const char* package,
					const char* filename,
					const char* prefix)
	  throw (hpp::Error);

	virtual void removeObstacleFromJoint
	(const char* objectName, const char* jointName, Boolean collision,
	 Boolean distance) throw (hpp::Error);

        virtual void cutObstacle (const char* objectName, const floatSeq& aabb)
        throw (Error);

	virtual	void
	addObstacle (const char* polyhedronName, Boolean collision,
		     Boolean distance)
	  throw (hpp::Error);

	virtual void
	moveObstacle (const char* polyName, const Transform_ cfg)
	  throw (hpp::Error);

	virtual void getObstaclePosition (const char* objectName,
					  Transform_ cfg)
	  throw (hpp::Error);

	virtual Names_t* getObstacleNames (bool collision, bool distance)
	  throw (hpp::Error);

	virtual void
	createPolyhedron (const char* polyhedronName)
	  throw (hpp::Error);

	virtual void createBox
	(const char* boxName, Double x, Double y, Double z)
	  throw(hpp::Error);

	virtual ULong
	addPoint
	(const char* polyhedronName, Double x, Double y, Double z)
	  throw(hpp::Error);

	virtual ULong
	addTriangle
	(const char* polyhedronName, ULong pt1, ULong pt2, ULong pt3)
	  throw(hpp::Error);


      private:
        CollisionObjectPtr_t getObstacleByName (const char* name);

        /// Map of object in construction.
        ObjectMap objectMap_;

	/// \brief Pointer to the hpp::corbaServer::Server owning this object.
	corbaServer::Server* server_;

	/// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
	core::ProblemSolverPtr_t problemSolver();
      };

    } // end of namespace implementation.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_OBSTACLE_HH
