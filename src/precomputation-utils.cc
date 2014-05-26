// Copyright (C) 2014 by Florent Lamiraux, Andreas Orthey, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.


#include <algorithm>
#include <vector>
#include "precomputation-utils.hh"

// --------------------------------------------------------------------
// Implementation of Andrew's monotone chain 2D convex hull algorithm.
// Asymptotic complexity: O(n log n).
// Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz machine.
namespace hpp
{
  namespace corbaServer
  {
    namespace precomputation
    {
      namespace convexhull
      {
        using namespace std;
         
        double cross(const ProjectedCapsulePoint &O, const ProjectedCapsulePoint &A, const ProjectedCapsulePoint &B)
        {
          return (A.y - O.y) * (B.z - O.z) - (A.z - O.z) * (B.y - O.y);
        }
         
        // Returns a list of points on the convex hull in counter-clockwise order.
        // Note: the last point in the returned list is the same as the first one.
        std::vector<ProjectedCapsulePoint> convex_hull(std::vector<ProjectedCapsulePoint> P)
        {
          uint n = P.size();
          uint k = 0;
          std::vector<ProjectedCapsulePoint> H(2*n);
         
          // Sort points lexicographically
          sort(P.begin(), P.end());
         
          // Build lower hull
          for (int i = 0; i < n; ++i) {
            while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
            H[k++] = P[i];
          }
         
          // Build upper hull
          for (int i = n-2, t = k+1; i >= 0; i--) {
            while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
            H[k++] = P[i];
          }
         
          H.resize(k);
          return H;
        }

      }// end of namespace convexhull
    } // end of namespace precomputation
  } // end of namespace corbaServer.
} // end of namespace hpp.
