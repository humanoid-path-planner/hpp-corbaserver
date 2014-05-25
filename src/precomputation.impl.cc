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

using namespace hpp::corbaServer::precomputation::convexhull;

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

          hullPoints = computeConvexHullFromCapsulePoints(capsulePoints);

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
