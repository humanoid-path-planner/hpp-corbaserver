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

# include "precomputation.impl.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace precomputation
    {
      namespace convexhull
      {
        /// \brief Implementation of Andrew's monotone chain 2D convex hull algorithm.
        /// Asymptotic complexity: O(n log n).
        /// Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz machine.
        /// Original source code from
        /// http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain#C.2B.2B

        using hpp::corbaServer::impl::ProjectedCapsulePoint;
        double cross(const ProjectedCapsulePoint &O, const ProjectedCapsulePoint &A, const ProjectedCapsulePoint &B);
         
        // \brief Returns a list of points on the convex hull in counter-clockwise order.
        // Note: the last point in the returned list is the same as the first one.
        std::vector<ProjectedCapsulePoint> convex_hull(std::vector<ProjectedCapsulePoint> P);

      }// end of namespace convexhull
    } // end of namespace precomputation
  } // end of namespace corbaServer.
} // end of namespace hpp.
