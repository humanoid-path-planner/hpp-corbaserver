// Copyright (C) 2014 by Florent Lamiraux, Andreas Orthey, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#pragma once

# include <vector>
# include <hpp/corbaserver/fwd.hh>
# include <precomputation.hh>
# include <robot.hh>
# include "common.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      /// \brief A capsule point represents the center of the top or bottom
      ///  part of the cylinder included in the capsule representation. 
      ///
      /// It contains R^3 coordinates, the length of the cylinder and the radius
      /// of the halfspheres, which are located at the top and bottom of the
      /// cylinder, with the capsule point being its center. Given the two capsule
      /// points of a capsule, we can compute every other point on the capsule
      /// surface. Additionally, we save the jacobian of the associated joint,
      /// such that we can use it for optimization purposes.

      struct CapsulePoint {
        double x,y,z;
        double radius;
        double length;
        hpp::model::JointJacobian_t J;
      };

      struct ProjectedCapsulePoint {
        double y,z;
        hpp::model::JointJacobian_t J;
        uint idx;
        bool operator <(const ProjectedCapsulePoint &rhs) const {
                return y < rhs.y || (y == rhs.y && z < rhs.z);
        }
      };

      /// \brief Implement CORBA interface ``Precomputation''.
      class Precomputation : public virtual POA_hpp::corbaserver::Precomputation
      {
      public:

	Precomputation (corbaServer::Server* server);

        /// \brief returns the points of the convex hull of the projected
        /// capsules
        virtual hpp::floatSeq* getConvexHullCapsules () throw (hpp::Error);

        /// \brief compute the gradient wrt to the outer convex hull points and
        /// its associated jacobians
        virtual hpp::floatSeq* getGradient () throw (hpp::Error);

        /// \brief get volume of the surface area, inscribed in the convex hull
        /// of the projected capsule points
        virtual double getVolume () throw (hpp::Error);

        /// \brief Use the current configuration and project it down until it is
        ///  irreducible up to a threshold
        virtual hpp::floatSeq* projectUntilIrreducible () throw (hpp::Error);

	virtual void setCurrentConfiguration
	(const hpp::floatSeq& dofArray) throw (hpp::Error);


      private:
        /// \brief Compute q = q + lambda*q', i.e. one update step of gradient
        // descent
        virtual hpp::floatSeq* updateConfiguration(const hpp::floatSeq &qq, double lambda) throw (hpp::Error);

        /// \brief Parse capsule points from the robot geometry and return them
        ///  in a vector
	virtual std::vector<CapsulePoint> parseCapsulePoints () throw (hpp::Error);

        /// \brief Project Capsule Points onto ZY Plane including outer points
        virtual std::vector<ProjectedCapsulePoint> projectCapsulePointsOnYZPlane (const std::vector<CapsulePoint> &capsVec);
        virtual std::vector<ProjectedCapsulePoint> computeConvexHullFromProjectedCapsulePoints (const std::vector<ProjectedCapsulePoint> &capsVec);

        virtual void computeProjectedConvexHullFromCurrentConfiguration () throw (hpp::Error);

        //
        /// \brief Convert capsule point vector to hpp::floatSeq 
        virtual hpp::floatSeq* capsulePointsToFloatSeq (const std::vector<CapsulePoint> &capsVector) 
         throw (hpp::Error);
        virtual hpp::floatSeq* capsulePointsToFloatSeq (const std::vector<ProjectedCapsulePoint> &capsVector) 
         throw (hpp::Error);

      private:
	/// \brief Pointer to the Server owning this object
	corbaServer::Server* server_;

	/// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
	/// Instantiated at construction.
	core::ProblemSolverPtr_t problemSolver_;

        std::vector<ProjectedCapsulePoint> cvxCaps_;
      };
    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.
