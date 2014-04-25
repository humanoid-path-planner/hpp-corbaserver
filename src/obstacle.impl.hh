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

# include <hpp/core/problem-solver.hh>
# include <hpp/corbaserver/fwd.hh>
# include <obstacle.hh>

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

	virtual void loadObstacleModel (const char* package,
					 const char* filename)
	  throw (hpp::Error);

	virtual	void
	addObstacle (const char* polyhedronName, Boolean collision,
		     Boolean distance)
	  throw (hpp::Error);

	virtual void
	moveObstacle (const char* polyName, const Configuration& cfg)
	  throw (hpp::Error);


	virtual void getObstaclePosition (const char* objectName,
					   Configuration& cfg)
	  throw (hpp::Error);

	virtual void
	createPolyhedron (const char* polyhedronName)
	  throw (hpp::Error);

	virtual void createBox
	(const char* boxName, Double x, Double y, Double z)
	  throw(hpp::Error);

	virtual Short
	addPoint
	(const char* polyhedronName, Double x, Double y, Double z)
	  throw(hpp::Error);

	virtual Short
	addTriangle
	(const char* polyhedronName, ULong pt1, ULong pt2, ULong pt3)
	  throw(hpp::Error);


      private:
	typedef std::map <std::string, std::vector <fcl::Vec3f> > VertexMap_t;
	typedef std::map <std::string, std::vector <fcl::Triangle> >
	TriangleMap_t;

	typedef std::map <std::string, BasicShapePtr_t> ShapeMap_t;
	/// Map of polyhedra in construction.
	VertexMap_t vertexMap_;
	TriangleMap_t triangleMap_;
	/// Map of basic shapes
	ShapeMap_t shapeMap_;
    CollisionObjectPtr_t getObstacleByName (const char* name);

	/// \brief Pointer to the hpp::corbaServer::Server owning this object.
	corbaServer::Server* server_;

	/// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
	core::ProblemSolverPtr_t problemSolver_;
      };

    } // end of namespace implementation.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_OBSTACLE_HH
