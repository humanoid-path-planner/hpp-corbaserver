// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <iostream>

#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>

#include <hpp/util/debug.hh>
#include <hpp/model/fwd.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/urdf/util.hh>
#include <hpp/model/object-factory.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include "robot.impl.hh"
#include "tools.hh"
#include "precomputation.impl.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      Precomputation::Precomputation(corbaServer::Server* server) :
	server_(server), problemSolver_(server->problemSolver ())
      {
      }

      Short Precomputation::getNumberDof () throw (hpp::Error)
      {
	try {
          hppDout(notice, "Precomputation");
	  return (Short) problemSolver_->robot ()->numberDof ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }
      // --------------------------------------------------------------------
      // --------------------------------------------------------------------
      // --------------------------------------------------------------------
      // TODO: outsource the fast convexhull implementation and its wrapper! (ask florent where)
      // Implementation of Andrew's monotone chain 2D convex hull algorithm.
        // Asymptotic complexity: O(n log n).
        // Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz machine.
namespace convexhull{
        #include <algorithm>
        #include <vector>
        using namespace std;
         
        typedef double coord_t;   // coordinate type
        typedef double coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2
         
        struct Point {
                coord_t x, y;
                uint idx;
                bool operator <(const Point &p) const {
                        return x < p.x || (x == p.x && y < p.y);
                }
        };
         
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
}//namespace convexhull

//###################################################################

      hpp::floatSeq* Precomputation::gradientConfigurationWrtProjection (const hpp::floatSeq& dofArray) 
        throw (hpp::Error){
          //###################################################################
          //## set new configuration, so that capsules have the right transform
          //###################################################################
          //outsourced to python file
                //DevicePtr_t robot = problemSolver_->robot ();
                //robot->setCurrentConfig(dofArray);
          return gradientConfigurationWrtProjection ();
      }

      hpp::floatSeq* Precomputation::gradientConfigurationWrtProjection () 
        throw (hpp::Error)
      {
        // set new configuration, compute projected points of capsules, get convex projected hull, 
        // compute jacobian of outer points of convex hull
        try {

          //###################################################################
          //## compute projected points of capsules
          //###################################################################
          using namespace std;
          using namespace fcl;
          DevicePtr_t robot = problemSolver_->robot ();
          std::vector<std::vector<double> > capsulePoints;
          std::vector<hpp::model::JointJacobian_t > capsuleJacobian;

          //nameSeq *innerObjectSeq = 0x0;
          JointVector_t jointVec = robot->getJointVector();
          std::stringstream stream;

          for(uint i = 0; i<jointVec.size(); i++){
            JointPtr_t joint = jointVec.at(i);
            const hpp::model::JointJacobian_t jjacobian = joint->jacobian();
            BodyPtr_t body = joint->linkedBody();
            if (!body) {
              //stream << "JOINT has no body" << endl;
              continue;
            }
            ObjectVector_t objects = body->innerObjects (model::COLLISION);
            for (std::size_t i=0; i<objects.size(); i++) {
              CollisionObjectPtr_t object = objects[i];
              fcl::CollisionObjectPtr_t fco = object->fcl();
              const fcl::NODE_TYPE nodeType = fco->getNodeType();
              const fcl::OBJECT_TYPE objectType = fco->getObjectType();
              //safety check the right node type:
              // geometry -- capsules
              if(objectType != OT_GEOM){
                std::ostringstream oss ("OBJECT_TYPE ");
                oss << objectType << " not handled by function.";
                throw hpp::Error (oss.str ().c_str ());
              }
              if(nodeType != GEOM_CAPSULE){
                std::ostringstream oss ("NODE_TYPE ");
                oss << nodeType << " not handled by function.";
                throw hpp::Error (oss.str ().c_str ());
              }
              const fcl::Capsule *capsule = static_cast<const fcl::Capsule*>(fco->getCollisionGeometry());
              double length = capsule->lz;
              double radius = capsule->radius;
              //-----------------------------------------------
              //compute the outer points of the
              //capsule: ( x1 -----o----- x2 )
              //here, o depicts the center of the
              //capsule, which is given by aabb_center
              fcl::Vec3f center = capsule->aabb_center;
              fcl::Transform3f T = fco->getTransform();

              //create points all along the axis of
              //the capsule and project them under the
              //given transformation
              double l = -length/2;
              double dl = length/2;
              while( l<=length/2 ){
                fcl::Vec3f x(0,0,l);
                fcl::Transform3f Tx;
                Tx.setTranslation(x);
                Tx = T*Tx;
                x = Tx.getTranslation();
                std::vector<double> pt;
                pt.push_back(x[1]);
                pt.push_back(x[2]);
                pt.push_back(radius);
                pt.push_back(length);
                capsulePoints.push_back(pt);
                capsuleJacobian.push_back(jjacobian);
                //compute outer points of capsule points 
                // which are at a distance of radius away
                double t_step = M_PI/6;
                for(double theta = 0; theta <= 2*M_PI; theta+=t_step){
                  std::vector<double> pt_out;
                  pt_out.push_back(cos(theta)*radius+x[1]);
                  pt_out.push_back(sin(theta)*radius+x[2]);
                  pt_out.push_back(radius);
                  pt_out.push_back(length);
                  capsulePoints.push_back(pt);
                  capsuleJacobian.push_back(jjacobian);
                }
                l = l+dl;
              }
            }
          }

          //###################################################################
          //## compute convex hull from 2d capsule points (y,z)
          //###################################################################
          std::vector<std::vector<double> > hullPoints;
          std::vector<hpp::model::JointJacobian_t > hullJacobian;

          hullPoints = convexhull::computeConvexHullFromCapsulePoints(capsulePoints);

          for(uint i=0; i<hullPoints.size(); i++){
            uint idx = uint(hullPoints.at(i).at(2));
            hullJacobian.push_back(capsuleJacobian.at(idx));
          }
          //###################################################################
          //## compute gradient of configuration q from outer jacobians
          //###################################################################
          //JointJacobian_t is Eigen::Matrix<double, 6, Eigen::Dynamic> 
          vector_t qgrad(robot->numberDof());
          qgrad.setZero();

          hppDout(notice, "gradient computation from outer jacobians");
          for(uint i=0; i<hullJacobian.size(); i++){
            vector_t cvx_pt_eigen(6);
            cvx_pt_eigen << 0,hullPoints.at(i).at(0),hullPoints.at(i).at(1),0,0,0;
            vector_t qi = hullJacobian.at(i).transpose()*cvx_pt_eigen;
            qgrad = qgrad - 0.1*qi;
          }
          hppDout(notice, "convert gradient to floatSeq and return");

          hpp::floatSeq* q_proj = new hpp::floatSeq;
          q_proj->length (robot->numberDof());
          for(uint i=0;i<robot->numberDof();i++){
            (*q_proj)[i] = qgrad[i];
          }
          hppDout(notice, stream.str());
          return q_proj;
        } catch (const std::exception& exc) {
          hppDout (error, exc.what ());
          throw hpp::Error (exc.what ());
        }
      }

      hpp::floatSeq* Precomputation::projectConfigurationUntilIrreducible (const hpp::floatSeq& dofArray) 
        throw (hpp::Error)
      {
        // configuration, get convex projected hull, compute jacobian of outer points,
        // get complete gradient, do gradient descent into (0,0) direction of each point

        return this->gradientConfigurationWrtProjection(dofArray);
      }

      hpp::floatSeq* Precomputation::computeVolume () throw (hpp::Error)
      {
        return this->parseCapsulePoints();
      }

      hpp::floatSeq* Precomputation::parseCapsulePoints () throw (hpp::Error)
      {
	try {
                using namespace std;
                using namespace fcl;
                hppDout (notice, "Computing Volume");
                DevicePtr_t robot = problemSolver_->robot ();
                hpp::floatSeq* capsPos = new hpp::floatSeq;
                std::vector<double> capsulePoints;

          //      nameSeq *innerObjectSeq = 0x0;
                JointVector_t jointVec = robot->getJointVector();
                std::stringstream stream;

                for(uint i = 0; i<jointVec.size(); i++){
                        //-----------------------------------------------
                        JointPtr_t joint = jointVec.at(i);
                        //-----------------------------------------------
                        BodyPtr_t body = joint->linkedBody();
                        if (!body) {
                                stream << "JOINT has no body" << endl;
                                continue;
                        }
                        stream << "JOINT " << joint->name () << endl;
                        stream << "BODY " << body->name () << endl;
                        //-----------------------------------------------
                        ObjectVector_t objects = body->innerObjects (model::COLLISION);
                        if (objects.size() > 0) {
                                std::size_t nbObjects = objects.size();
                                for (std::size_t iObject=0; iObject < nbObjects; iObject++) {
                                        //-----------------------------------------------
                                        CollisionObjectPtr_t object = objects[iObject];
                                        std::string geometryName = object->name();
                                        stream << "OBJECT " << geometryName << endl;
                                        //-----------------------------------------------
                                        fcl::CollisionObjectPtr_t fco = object->fcl();
                                        //-----------------------------------------------
                                        const fcl::NODE_TYPE nodeType = fco->getNodeType();
                                        const fcl::OBJECT_TYPE objectType = fco->getObjectType();
                                        //safety check the right node type:
                                        // geometry -- capsules
                                        if(objectType != OT_GEOM){
                                                hppDout(error, "OBJECT_TYPE " << objectType << " not handled by function");
                                        }
                                        if(nodeType != GEOM_CAPSULE){
                                                hppDout(error, "NODE_TYPE " << nodeType << " not handled by function");
                                        }
                                        const fcl::Capsule *capsule = static_cast<const fcl::Capsule*>(fco->getCollisionGeometry());

                                        double length = capsule->lz;
                                        double radius = capsule->radius;
                                        //-----------------------------------------------
                                        //compute the outer points of the
                                        //capsule: ( x1 -----o----- x2 )
                                        //here, o depicts the center of the
                                        //capsule, which is given by aabb_center
                                        fcl::Vec3f center = capsule->aabb_center;
                                        fcl::Transform3f T = fco->getTransform();

                                        //create points all along the axis of
                                        //the capsule and project them under the
                                        //given transformation
                                        double l = -length/2;
                                        double dl = length/2;
                                        while( l<=length/2 ){
                                                fcl::Vec3f x(0,0,l);
                                                fcl::Transform3f Tx;
                                                Tx.setTranslation(x);
                                                Tx = T*Tx;
                                                x = Tx.getTranslation();
                                                capsulePoints.push_back(x[0]);
                                                capsulePoints.push_back(x[1]);
                                                capsulePoints.push_back(x[2]);
                                                capsulePoints.push_back(radius);
                                                capsulePoints.push_back(length);
                                                l = l+dl;
                                        }
                                        //-----------------------------------------------
                                        //fcl::AABB aabb = fco->getAABB();
                                        //stream << aabb.width() << " " << aabb.height() << endl;
                                }
                        }
                }
                capsPos->length (capsulePoints.size());
                for(uint i=0;i<capsulePoints.size();i++){
                        (*capsPos)[i] = capsulePoints.at(i);
                }

                hppDout(notice, stream.str());

                return capsPos;
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.
