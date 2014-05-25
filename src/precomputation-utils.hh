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

namespace hpp
{
  namespace corbaServer
  {
    namespace precomputation
    {
      namespace convexhull
      {
        typedef double coord_t;   // coordinate type
        typedef double coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2
         
        struct Point {
                coord_t x, y;
                uint idx;
                bool operator <(const Point &p) const {
                        return x < p.x || (x == p.x && y < p.y);
                }
        };

        coord2_t cross(const Point &O, const Point &A, const Point &B);
         
        // \brief Returns a list of points on the convex hull in counter-clockwise order.
        // Note: the last point in the returned list is the same as the first one.
        std::vector<Point> convex_hull(std::vector<Point> P);

        /// \brief Wrapper for convex hull computation on capsules
        ///     with capsules in the format < <x,y,z,radius,length>, ...>
        std::vector<std::vector<double> > computeConvexHullFromCapsulePoints( 
                 std::vector<std::vector<double> > &capsules);

      }// end of namespace convexhull
    } // end of namespace precomputation
  } // end of namespace corbaServer.
} // end of namespace hpp.
