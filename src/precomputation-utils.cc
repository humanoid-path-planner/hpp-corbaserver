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
         
        // 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
        // Returns a positive value, if OAB makes a counter-clockwise turn,
        // negative for clockwise turn, and zero if the points are collinear.
        coord2_t cross(const Point &O, const Point &A, const Point &B)
        {
                return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
        }
         
        // Returns a list of points on the convex hull in counter-clockwise order.
        // Note: the last point in the returned list is the same as the first one.
        vector<Point> convex_hull(vector<Point> P)
        {
                int n = P.size(), k = 0;
                vector<Point> H(2*n);
         
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

        //wrapper for convex hull on capsules
        std::vector<std::vector<double> > computeConvexHullFromCapsulePoints( 
                 std::vector<std::vector<double> > &capsules)
        {
          std::vector<convexhull::Point> P;
          std::vector<std::vector<double> > hull_vec;
          for(uint i=0; i<capsules.size(); i++){
            convexhull::Point pt;
            pt.x = capsules.at(i).at(0);
            pt.y = capsules.at(i).at(1);
            pt.idx = i;
            P.push_back(pt);
          }

          std::vector<convexhull::Point> hull_pts = convexhull::convex_hull(P);

          for(uint i=0; i<hull_pts.size(); i++){
            std::vector<double> pt;
            pt.push_back(hull_pts.at(i).x);
            pt.push_back(hull_pts.at(i).y);
            pt.push_back(hull_pts.at(i).idx);
            hull_vec.push_back(pt);
          }
          return hull_vec;
        }

      }// end of namespace convexhull
    } // end of namespace precomputation
  } // end of namespace corbaServer.
} // end of namespace hpp.
