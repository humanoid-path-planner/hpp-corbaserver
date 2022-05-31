// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_ROBOT_IMPL_HH
#define HPP_CORBASERVER_ROBOT_IMPL_HH
#include <hpp/fcl/BVH/BVH_model.h>

#include <map>
#include <string>
// # include <hpp/pinocchio/object-factory.hh>
#include "hpp/corbaserver/fwd.hh"
#include "hpp/corbaserver/object-map.hh"
#include "hpp/corbaserver/robot-idl.hh"
#include "hpp/core/problem-solver.hh"

namespace hpp {
namespace corbaServer {
namespace impl {
using CORBA::Long;
/**
   Robots and obstacles are stored in object core::Planner.

   The kinematic part of a robot is stored in a
   CkppDeviceComponent object (see KineoWorks documentation).

   \li To each \em joint is attached a \em body (CkwsBody).

   \li Each \em body contains a list of CkcdObject (derived into
   CkppKCDPolyhedron).

   \li A \em polyhedron is defined by a set of \em vertices and a
   set of \em facets.

   Obstacles are stored in collision lists (CkcdCollisionList)
   composed of polyhedra (CkppKCDPolyhedron).
*/

/// \brief Implementation of corba interface hpp::Robot.
///
/// The construction of a
class Robot : public virtual POA_hpp::corbaserver::Robot {
 public:
  Robot();

  void setServer(ServerPlugin* server) { server_ = server; }

  virtual void createRobot(const char* robotName);

  virtual void loadRobotModel(const char* robotName, const char* rootJointType,
                              const char* urdfName, const char* srdfName);

  virtual void loadHumanoidModel(const char* robotName,
                                 const char* rootJointType,
                                 const char* urdfName, const char* srdfName);

  virtual void loadRobotModelFromString(const char* robotName,
                                        const char* rootJointType,
                                        const char* urdfString,
                                        const char* srdfString);

  virtual void loadHumanoidModelFromString(const char* robotName,
                                           const char* rootJointType,
                                           const char* urdfString,
                                           const char* srdfString);

  virtual char* getRobotName();

  virtual Long getConfigSize();

  virtual Long getNumberDof();

  virtual void appendJoint(const char* parentName, const char* jointName,
                           const char* jointType, const Transform_ pos);

  virtual Names_t* getJointNames();
  virtual Names_t* getJointTypes();
  virtual Names_t* getAllJointNames();
  virtual Names_t* getChildJointNames(const char* jointName);
  virtual char* getParentJointName(const char* jointName);
  virtual hpp::floatSeq* getJointConfig(const char* jointName);
  virtual void setJointConfig(const char* jointName, const floatSeq& cfg);
  virtual char* getJointType(const char* jointName);
  virtual floatSeq* jointIntegrate(const floatSeq& jointCfg,
                                   const char* jointName, const floatSeq& dq,
                                   bool saturate);
  virtual hpp::floatSeqSeq* getCurrentTransformation(const char* jointName);
  virtual Transform__slice* getJointPositionInParentFrame(
      const char* jointName);
  virtual Transform__slice* getJointPosition(const char* jointName);
  virtual TransformSeq* getJointsPosition(const floatSeq& q,
                                          const Names_t& jointNames);
  virtual floatSeq* getJointVelocity(const char* jointName);
  virtual floatSeq* getJointVelocityInLocalFrame(const char* jointName);

  virtual Transform__slice* getRootJointPosition();

  virtual void setRootJointPosition(const Transform_ position);

  virtual void setJointPositionInParentFrame(const char* jointName,
                                             const Transform_ position);

  virtual Long getJointNumberDof(const char* jointName);
  virtual Long getJointConfigSize(const char* jointName);

  virtual hpp::floatSeq* getJointBounds(const char* jointName);
  virtual void setJointBounds(const char* jointName,
                              const hpp::floatSeq& jointBound);

  virtual Transform__slice* getLinkPosition(const char* linkName);

  virtual TransformSeq* getLinksPosition(const floatSeq& q,
                                         const Names_t& linkName);

  virtual Names_t* getLinkNames(const char* jointName);

  virtual void setDimensionExtraConfigSpace(ULong dimension);

  virtual ULong getDimensionExtraConfigSpace();

  virtual void setExtraConfigSpaceBounds(const hpp::floatSeq& bounds);

  virtual void setCurrentConfig(const hpp::floatSeq& dofArray);

  virtual hpp::floatSeq* shootRandomConfig();
  virtual hpp::floatSeq* getCurrentConfig();

  virtual void setCurrentVelocity(const hpp::floatSeq& qDot);
  virtual hpp::floatSeq* getCurrentVelocity();

  virtual Names_t* getJointInnerObjects(const char* bodyName);

  virtual Names_t* getJointOuterObjects(const char* bodyName);

  virtual void getObjectPosition(const char* objectName, Transform_ cfg);

  virtual void isConfigValid(const hpp::floatSeq& dofArray, Boolean& validity,
                             CORBA::String_out report);

  virtual void distancesToCollision(hpp::floatSeq_out distances,
                                    Names_t_out innerObjects,
                                    Names_t_out outerObjects,
                                    hpp::floatSeqSeq_out innerPoints,
                                    hpp::floatSeqSeq_out outerPoints);

  virtual void autocollisionCheck(hpp::boolSeq_out collide);

  virtual void autocollisionPairs(Names_t_out innerObjects,
                                  Names_t_out outerObjects, boolSeq_out active);

  virtual void setAutoCollision(const char* innerObject,
                                const char* outerObject, bool active);

  virtual Double getMass();

  virtual hpp::floatSeq* getCenterOfMass();

  virtual hpp::floatSeq* getCenterOfMassVelocity();

  virtual hpp::floatSeqSeq* getJacobianCenterOfMass();

  virtual void createPolyhedron(const char* polyhedronName);

  virtual void createBox(const char* name, Double x, Double y, Double z);

  virtual void createSphere(const char* name, Double radius);

  virtual void createCylinder(const char* name, Double radius, Double length);

  virtual ULong addPoint(const char* polyhedronName, Double x, Double y,
                         Double z);

  virtual ULong addTriangle(const char* polyhedronName, ULong pt1, ULong pt2,
                            ULong pt3);

  virtual void addObjectToJoint(const char* jointName, const char* objectName,
                                const Transform_ config);

  virtual void addPartialCom(const char* comName, const Names_t& jointNames);

  virtual hpp::floatSeq* getPartialCom(const char* comName);

  virtual hpp::floatSeqSeq* getJacobianPartialCom(const char* comName);

  virtual hpp::floatSeq* getVelocityPartialCom(const char* comName);

  hpp::pinocchio_idl::CenterOfMassComputation_ptr getCenterOfMassComputation(
      const char* name);

  virtual floatSeq* getRobotAABB();

 private:
  CollisionObjectConstPtr_t getObjectByName(const char* name);

  ObjectMap objectMap_;

  /// \brief Pointer to the ServerPlugin owning this object.
  ServerPlugin* server_;

  /// \brief Pointer to hpp::core::ProblemSolver object of ServerPlugin.
  core::ProblemSolverPtr_t problemSolver();
};
}  // end of namespace impl.
}  // end of namespace corbaServer.
}  // end of namespace hpp.

#endif
