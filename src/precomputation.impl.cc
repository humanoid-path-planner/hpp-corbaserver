// Copyright (C) 2014 by Florent Lamiraux, Andreas Orthey, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <iostream>
#include <cmath>

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
#include "precomputation-utils.hh"


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

      hpp::floatSeq* Precomputation::getConvexHullCapsules () throw (hpp::Error)
      {
        return capsulePointsToFloatSeq(cvxCaps_);
      }
      void Precomputation::setCurrentConfiguration(const hpp::floatSeq &dofArray) throw (hpp::Error)
      {
	std::size_t configDim = (std::size_t)dofArray.length();
	std::vector<double> dofVector;
	try {
	  DevicePtr_t robot = problemSolver_->robot ();
	  std::size_t deviceDim = robot->configSize ();
	  vector_t config; config.resize (configDim);
	  for (std::size_t iDof = 0; iDof < configDim; iDof++) {
	    config [iDof] = dofArray[iDof];
	  }
	  if(configDim != deviceDim){
	    hppDout (notice, "config dimension: " <<configDim
		   <<",  deviceDim "<<deviceDim);
	    throw hpp::Error ("dofVector Does not match");
	  }
	  // Create a config for robot initialized with dof vector.
	  problemSolver_->robot ()->currentConfiguration (config);
	  problemSolver_->robot ()->computeForwardKinematics ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      hpp::floatSeq* Precomputation::updateConfiguration (const hpp::floatSeq &dofArray, double lambda) throw (hpp::Error){
      }
      hpp::floatSeq* Precomputation::projectUntilIrreducible () throw (hpp::Error)
      {
        double epsilon = 0.001; //convergence threshold
        double error = 1;
        double lambda = 0.1; //update value
        //while( error > epsilon ){
          //this->computeProjectedConvexHullFromCurrentConfiguration ();
          //floatSeq* qq = this->getGradient();
          //floatSeq* q = this->updateConfiguration(qq, lambda);
          //this->setCurrentConfiguration(*q);
          //this->computeProjectedConvexHullFromCurrentConfiguration ();
        //}
      }

      double Precomputation::getVolume () throw (hpp::Error)
      {
        //assume that hull are points on a convex hull, which are sorted
        //counter-clockwise.
        using namespace Eigen;
        Vector2f x1(cvxCaps_.at(0).y, cvxCaps_.at(0).z);
        double volume = 0;
        //compute volume by iterating over all embeded triangles
        for(int i=0; i<cvxCaps_.size()-2; i++){
          Vector2f x2(cvxCaps_.at(i+1).y, cvxCaps_.at(i+1).z);
          Vector2f x3(cvxCaps_.at(i+2).y, cvxCaps_.at(i+2).z);
          Vector2f v12 = x2-x1;
          Vector2f v13 = x3-x1;
          double b = v12.norm();
          Vector2f h_vec = v13 - v13.dot(v12)*v12;
          double h = h_vec.norm();
          double d = 0.5*b*h;
          volume += d;
        }
        return volume;
      }

      void Precomputation::computeProjectedConvexHullFromCurrentConfiguration () 
        throw (hpp::Error)
      {
	std::vector<CapsulePoint> caps = parseCapsulePoints();
        std::vector<ProjectedCapsulePoint> projCaps = projectCapsulePointsOnYZPlane(caps);
        cvxCaps_.clear();
        cvxCaps_ = computeConvexHullFromProjectedCapsulePoints(projCaps);
      }

      hpp::floatSeq* Precomputation::getGradient() 
        throw (hpp::Error)
      {
        //Compute gradient wrt outer jacobians
        DevicePtr_t robot = problemSolver_->robot ();
        //JointJacobian_t is Eigen::Matrix<double, 6, Eigen::Dynamic> 
        vector_t qgrad(robot->numberDof());
        qgrad.setZero();

        hppDout(notice, "gradient computation from outer jacobians");
        for(uint i=0; i<cvxCaps_.size(); i++){
          vector_t cvx_pt_eigen(6);
          cvx_pt_eigen << 0,cvxCaps_.at(i).y,cvxCaps_.at(i).z,0,0,0;
          vector_t qi = cvxCaps_.at(i).J.transpose()*cvx_pt_eigen;
          qgrad = qgrad + qi;
        }
        hppDout(notice, "convert gradient to floatSeq and return");
        hpp::floatSeq* q_proj = new hpp::floatSeq;
        q_proj->length (robot->numberDof());
        for(uint i=0;i<robot->numberDof();i++){
          (*q_proj)[i] = qgrad[i];
        }
        return q_proj;
      }

      hpp::floatSeq* Precomputation::capsulePointsToFloatSeq (const std::vector<ProjectedCapsulePoint> &capsVector) 
         throw (hpp::Error)
      {
	try {
          hpp::floatSeq* capsFloatSeq = new hpp::floatSeq;
          capsFloatSeq->length (3*capsVector.size());
          int ctr = 0;
          for(uint i=0;i<capsVector.size();i++){
            (*capsFloatSeq)[ctr++] = 0;
            (*capsFloatSeq)[ctr++] = capsVector.at(i).y;
            (*capsFloatSeq)[ctr++] = capsVector.at(i).z;
          }
          return capsFloatSeq;
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }
      hpp::floatSeq* Precomputation::capsulePointsToFloatSeq (const std::vector<CapsulePoint> &capsVector) 
         throw (hpp::Error)
      {
	try {
          hpp::floatSeq* capsFloatSeq = new hpp::floatSeq;
          capsFloatSeq->length (3*capsVector.size());
          int ctr = 0;
          for(uint i=0;i<capsVector.size();i++){
            (*capsFloatSeq)[ctr++] = capsVector.at(i).x;
            (*capsFloatSeq)[ctr++] = capsVector.at(i).y;
            (*capsFloatSeq)[ctr++] = capsVector.at(i).z;
          }
          return capsFloatSeq;
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      std::vector<ProjectedCapsulePoint> Precomputation::computeConvexHullFromProjectedCapsulePoints (const std::vector<ProjectedCapsulePoint> &capsVec) 
      {
        using namespace hpp::corbaServer::precomputation::convexhull;
        return convex_hull(capsVec);
      }

      std::vector<ProjectedCapsulePoint> Precomputation::projectCapsulePointsOnYZPlane (const std::vector<CapsulePoint> &capsVec) 
      {
        std::vector<ProjectedCapsulePoint> projCapsulePointsYZPlane;
        for(uint i=0; i<capsVec.size(); i++){
          ProjectedCapsulePoint p;
          p.y = capsVec.at(i).y;
          p.z = capsVec.at(i).z;
          p.J = capsVec.at(i).J;
          projCapsulePointsYZPlane.push_back(p);

          // orthogonal projection by just removing the x-component of the
          // point.
          // we still need to compute the outer points, which are located at
          // radius from the capsule point on the YZ plane. We will compute only
          // a finite number of outer points, which is equal to an approximation
          // of the circle by an inner polygon. Change the t_step size to get
          // a better approximation.
          double t_step = M_PI/6;
          double yP = capsVec.at(i).y;
          double zP = capsVec.at(i).z;
          double radius = capsVec.at(i).radius;
          for(double theta = 0; theta <= 2*M_PI; theta+=t_step){
            ProjectedCapsulePoint outerPoint;
            outerPoint.y = cos(theta)*radius + yP;
            outerPoint.z = sin(theta)*radius + zP;
            outerPoint.J = capsVec.at(i).J;
            projCapsulePointsYZPlane.push_back(outerPoint);
          }
        }
        return projCapsulePointsYZPlane;
      }

      std::vector<CapsulePoint> Precomputation::parseCapsulePoints () throw (hpp::Error)
      {
	try {
          using namespace std;
          using namespace fcl;
          DevicePtr_t robot = problemSolver_->robot ();
          std::vector<CapsulePoint> capsuleVec;
          JointVector_t jointVec = robot->getJointVector();
          std::stringstream stream; //DEBUG stream

          for(uint i=0; i<jointVec.size(); i++){
            //-----------------------------------------------
            JointPtr_t joint = jointVec.at(i);
            const hpp::model::JointJacobian_t jjacobian = joint->jacobian();
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
                //x1,x2 are the center points of the top 
                //and bottom disc of the
                //cylinder, respectively.

                fcl::Vec3f center = capsule->aabb_center;
                fcl::Transform3f T = fco->getTransform();

                //create points all along the axis of
                //the capsule and project them under the
                //given transformation
                double l = -length/2;
                while(l <= length/2){
                  fcl::Vec3f x(0,0,l);
                  fcl::Transform3f Tx;
                  Tx.setTranslation(x);
                  Tx = T*Tx;
                  x = Tx.getTranslation();
                  CapsulePoint p;
                  p.x = x[0];
                  p.y = x[1];
                  p.z = x[2];
                  p.radius = radius;
                  p.length = length;
                  p.J = jjacobian;
                  capsuleVec.push_back(p);
                  l += length;
                  hppDout(notice, l);
                }
                //-----------------------------------------------
                //fcl::AABB aabb = fco->getAABB();
                //stream << aabb.width() << " " << aabb.height() << endl;
              }//iterate inner objects
            }
          }//iterate joints
          //IF DEBUG
          hppDout(notice, stream.str());
          return capsuleVec;

	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.
