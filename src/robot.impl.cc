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

#include <iostream>

#include <pinocchio/fwd.hpp>

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>

#include <hpp/pinocchio/fwd.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/frame.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/pinocchio/liegroup-element.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/distance-between-objects.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/corbaserver/server-plugin.hh>
#include "robot.impl.hh"
#include "tools.hh"

#include "hpp/pinocchio_idl/robots-fwd.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      using pinocchio::Model;
      using pinocchio::Data;
      using pinocchio::JointIndex;
      using pinocchio::FrameIndex;

      namespace
      {
        using CORBA::Long;
        using CORBA::ULong;
        const ::pinocchio::FrameType JOINT_FRAME = (::pinocchio::FrameType)(::pinocchio::JOINT | ::pinocchio::FIXED_JOINT);

	static void localSetJointBounds(const JointPtr_t& joint,
					vector_t jointBounds)
	{
          static const value_type inf = std::numeric_limits<value_type>::infinity();
          if (jointBounds.size() %2 == 1) {
            HPP_THROW(std::invalid_argument, "Expect a vector of even size");
          }
          for (std::size_t i = 0; i < (std::size_t)jointBounds.size() / 2; i++) {
            value_type& vMin = jointBounds[2*i  ];
            value_type& vMax = jointBounds[2*i+1];
            if (vMin > vMax) {
              vMin = -inf;
              vMax =  inf;
            }
          }
          Eigen::Map<vector_t, Eigen::Unaligned, Eigen::InnerStride<2> >
            lower (&jointBounds[0], ( jointBounds.size() + 1 ) / 2 );
          Eigen::Map<vector_t, Eigen::Unaligned, Eigen::InnerStride<2> >
            upper (&jointBounds[1], ( jointBounds.size()     ) / 2 );
          joint->lowerBounds (lower);
          joint->upperBounds (upper);
	}

	static floatSeq* localGetJointBounds(const JointPtr_t& joint)
	{
	  std::size_t jointNbDofs = joint->configSize ();
          floatSeq* ret = new floatSeq;
          ret->length ((ULong) (2*jointNbDofs));
          for (std::size_t iDof=0; iDof<jointNbDofs; iDof++) {
            if (joint->isBounded (iDof)) {
	      (*ret) [(ULong) (2*iDof)] = joint->lowerBound (iDof);
	      (*ret) [(ULong) (2*iDof + 1)] = joint->upperBound (iDof);
            } else {
	      (*ret) [(ULong) (2*iDof)] = 1;
	      (*ret) [(ULong) (2*iDof + 1)] = 0;
            }
          }
          return ret;
	}
      } // end of namespace.

      // --------------------------------------------------------------------

      Robot::Robot() :
	server_(NULL)
      {
      }

      // --------------------------------------------------------------------

      core::ProblemSolverPtr_t Robot::problemSolver()
      {
        return server_->problemSolver();
      }

      // --------------------------------------------------------------------

      void Robot::createRobot(const char* inRobotName)
      {
	std::string robotName (inRobotName);
        DevicePtr_t robot (problemSolver()->createRobot (robotName));
        problemSolver ()->robot (robot);
      }

      // --------------------------------------------------------------------

      char* Robot::getRobotName ()
      {
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
        return c_str (robot->name());
      }

      // --------------------------------------------------------------------

      void Robot::loadRobotModel(const char* robotName,
                                 const char* rootJointType,
                                 const char* urdfName,
                                 const char* srdfName)
      {
	try {
	  pinocchio::DevicePtr_t device
            (problemSolver ()->createRobot (robotName));
	  hpp::pinocchio::urdf::loadModel (device, 0, "",
                                           std::string (rootJointType),
                                           std::string (urdfName),
                                           std::string (srdfName));
	  // Add device to the planner
	  problemSolver()->robot (device);
	  problemSolver()->robot ()->controlComputation
	    (hpp::pinocchio::JOINT_POSITION);
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::loadHumanoidModel(const char* robotName,
                                    const char* rootJointType,
                                    const char* urdfName,
                                    const char* srdfName)
      {
	try {
	  pinocchio::HumanoidRobotPtr_t robot
	    (pinocchio::HumanoidRobot::create (robotName));
	  hpp::pinocchio::urdf::loadModel (robot, 0, "",
                                           std::string (rootJointType),
                                           std::string (urdfName),
                                           std::string (srdfName));
          hpp::pinocchio::urdf::setupHumanoidRobot (robot);
          // Add robot to the planner
	  problemSolver()->robot (robot);
	  problemSolver()->robot ()->controlComputation
	    (hpp::pinocchio::JOINT_POSITION);
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::loadRobotModelFromString(
          const char* robotName,
          const char* rootJointType,
          const char* urdfString,
          const char* srdfString)
      {
	try {
	  pinocchio::DevicePtr_t device (problemSolver ()->createRobot
                                         (robotName));
	  hpp::pinocchio::urdf::loadModelFromString (
              device, 0, "",
              std::string (rootJointType),
              std::string (urdfString),
              std::string (srdfString));
	  // Add device to the planner
	  problemSolver()->robot (device);
	  problemSolver()->robot ()->controlComputation
	    (hpp::pinocchio::JOINT_POSITION);
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::loadHumanoidModelFromString(
          const char* robotName,
          const char* rootJointType,
          const char* urdfString,
          const char* srdfString)
      {
	try {
	  pinocchio::HumanoidRobotPtr_t robot
	    (pinocchio::HumanoidRobot::create (robotName));
          hpp::pinocchio::urdf::loadModelFromString (
              robot, 0, "",
              std::string (rootJointType),
              std::string (urdfString),
              std::string (srdfString));
          hpp::pinocchio::urdf::setupHumanoidRobot (robot);
          // Add robot to the planner
	  problemSolver()->robot (robot);
	  problemSolver()->robot ()->controlComputation
	    (hpp::pinocchio::JOINT_POSITION);
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Long Robot::getConfigSize ()
      {
	try {
	  return (Long) getRobotOrThrow(problemSolver())->configSize ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Long Robot::getNumberDof ()
      {
	try {
          return (Long) getRobotOrThrow(problemSolver())->numberDof ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      template <typename T> void addJoint (Model& model,
          const JointIndex parent,
          const Transform3f& placement,
          const std::string& name)
      {
        JointIndex jid = model.addJoint (parent, T(), placement, name);
        model.addJointFrame (jid, -1);
      }

      void Robot::appendJoint (
          const char* parentName,
          const char* jointName,
          const char* jointType,
          const Transform_ pos)
      {
        using namespace pinocchio;
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
        Model& model = robot->model();

	std::string pn(parentName);
	std::string jn(jointName);
	std::string jt(jointType);

        // Find parent joint
        JointIndex pid = 0;
        if (!pn.empty()) {
          if (!model.existJointName(pn))
            throw Error (("Joint " + pn + " not found.").c_str());
          pid = model.getJointId (pn);
        }

	// Check that joint of this name does not already exist.
        if (model.existJointName(jn))
          throw Error (("Joint " + jn + " already exists.").c_str());

	// Fill position matrix
	Transform3f posMatrix = toTransform3f(pos);

	// Determine type of joint.
	if (jt == "FreeFlyer")
          addJoint <JointCollection::JointModelFreeFlyer> (model, pid, posMatrix, jn);
        else if (jt == "Planar")
          addJoint <JointCollection::JointModelPlanar> (model, pid, posMatrix, jn);
	else if (jt == "RX")
          addJoint <JointCollection::JointModelRX> (model, pid, posMatrix, jn);
	else if (jt == "RUBX")
          addJoint <JointCollection::JointModelRUBX> (model, pid, posMatrix, jn);
	else if (jt == "RY")
          addJoint <JointCollection::JointModelRY> (model, pid, posMatrix, jn);
	else if (jt == "RUBY")
          addJoint <JointCollection::JointModelRUBY> (model, pid, posMatrix, jn);
	else if (jt == "RZ")
          addJoint <JointCollection::JointModelRZ> (model, pid, posMatrix, jn);
	else if (jt == "RUBZ")
          addJoint <JointCollection::JointModelRUBZ> (model, pid, posMatrix, jn);
	else if (jt == "PX")
          addJoint <JointCollection::JointModelPX> (model, pid, posMatrix, jn);
	else if (jt == "PY")
          addJoint <JointCollection::JointModelPY> (model, pid, posMatrix, jn);
	else if (jt == "PZ")
          addJoint <JointCollection::JointModelPZ> (model, pid, posMatrix, jn);
	else if (jt == "Translation")
          addJoint <JointCollection::JointModelTranslation> (model, pid, posMatrix, jn);
	else {
	  std::ostringstream oss;
	  oss << "Joint type \"" << jt << "\" does not exist.";
	  hppDout (error, oss.str ());
	  throw hpp::Error (oss.str ().c_str ());
	}

          robot->createData();
      }

      // --------------------------------------------------------------------

      Names_t* Robot::getJointNames ()
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          const Model& model = robot->model();
          return toNames_t ((++model.names.begin()), model.names.end());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Names_t* Robot::getJointTypes ()
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          const Model& model = robot->model();
          std::vector<std::string> types (model.names.size()-1);
          for (std::size_t i = 0; i < types.size(); ++i)
            types[i] = model.joints[i+1].shortname();
          return toNames_t (types.begin(), types.end());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Names_t* Robot::getAllJointNames ()
      {
	try {
	  DevicePtr_t robot = problemSolver()->robot ();
          if (!robot)
            return new Names_t (0, 0, Names_t::allocbuf (0));
	  // Compute number of real urdf joints
          const Model& model = robot->model();
          std::vector<std::string> jns;
          for(std::size_t i = 0; i < model.frames.size(); ++i)
            jns.push_back(model.frames[i].name);
          return toNames_t (jns.begin(), jns.end());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Names_t* Robot::getChildJointNames (const char* jointName)
      {
	try {
	  DevicePtr_t robot = problemSolver()->robot ();
          if (!robot) return new Names_t (0, 0, Names_t::allocbuf (0));
	  // Compute number of real urdf joints
          JointPtr_t j = robot->getJointByName (std::string (jointName));
	  ULong size = (ULong) j->numberChildJoints ();
	  char** nameList = Names_t::allocbuf(size);
	  Names_t *jointNames = new Names_t (size, size, nameList);
	  for (std::size_t i = 0; i < size; ++i) {
	    const JointPtr_t joint = j->childJoint (i);
	    nameList [i] = c_str (joint->name());
	  }
	  return jointNames;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      char* Robot::getParentJointName (const char* jointName)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          const std::string jn (jointName);
          const Frame f = robot->getFrameByName(jn);

          char* name;
          if (f.isRootFrame()) {
            name = CORBA::string_dup("");
          } else {
            // TODO if f.parentFrame is not of type JOINT or FIXED_JOINT
            // we should continue to loop on the parent.
            name = c_str (f.parentFrame().name());
          }
          return name;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------
      Transform__slice* Robot::getRootJointPosition ()
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  Frame root = robot->rootFrame ();
          return toHppTransform (root.positionInParentFrame());
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setRootJointPosition (const Transform_ position)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  Transform3f t3f (toTransform3f (position));
	  robot->rootFrame().positionInParentFrame(t3f);
          robot->computeForwardKinematics ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setJointPositionInParentFrame (const char* jointName, const Transform_ position)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          frame.positionInParentFrame(toTransform3f(position));
          robot->computeForwardKinematics ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getJointConfig(const char* jointName)
      {
	try {
	  // Get robot in hppPlanner object.
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          if (frame.isFixed())
            return new hpp::floatSeq();
	  JointPtr_t joint = frame.joint();
          if (!joint)
            return new hpp::floatSeq();
          vector_t config = robot->currentConfiguration ();
          size_type ric = joint->rankInConfiguration ();
	  size_type dim = joint->configSize ();
          return vectorToFloatSeq(config.segment(ric, dim));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      char* Robot::getJointType(const char* jointName)
      {
	try {
	  // Get robot in hppPlanner object.
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          std::string name;
          if (frame.isFixed())
            return CORBA::string_dup("anchor");
	  JointPtr_t joint = frame.joint();
          if (!joint)
            return CORBA::string_dup("anchor");
          return c_str(joint->jointModel().shortname());
        } catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setJointConfig(const char* jointName, const hpp::floatSeq& q)
      {
	try {
	  // Get robot in hppPlanner object.
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  JointPtr_t joint = robot->getJointByName (jointName);
	  if (!joint) {
	    std::ostringstream oss ("Robot has no joint with name ");
	    oss  << jointName;
	    hppDout (error, oss.str ());
	    throw hpp::Error (oss.str ().c_str ());
	  }
          vector_t config = robot->currentConfiguration ();
          size_type ric = joint->rankInConfiguration ();
	  size_type dim = joint->configSize ();
          if (dim != (size_type) q.length ()) {
	    throw Error ("Wrong configuration dimension");
	  }
	  for(size_type i=0; i<dim; i++)
            config [ric + i] = q[(ULong) i];
          robot->currentConfiguration (config);
          robot->computeForwardKinematics ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      floatSeq* Robot::jointIntegrate(const floatSeq& jointCfg,
            const char* jointName, const floatSeq& dq, bool saturate)
      {
	try {
	  // Get robot in hppPlanner object.
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  JointPtr_t joint = robot->getJointByName (jointName);
	  if (!joint) {
	    std::ostringstream oss ("Robot has no joint with name ");
	    oss  << jointName;
	    hppDout (error, oss.str ());
	    throw hpp::Error (oss.str ().c_str ());
	  }
	  const size_type nq  = joint->configSize (),
	                  nv  = joint->numberDof ();
          if (nv != (size_type) dq.length ()) {
	    throw Error ("Wrong speed dimension");
	  }
          LiegroupElement q (floatSeqToVector (jointCfg, nq),
                             joint->configurationSpace());
          q += floatSeqToVector (dq, nv);
          if (saturate) {
            for (size_type i = 0; i < nq; ++i) {
              value_type ub = joint->upperBound(i);
              value_type lb = joint->lowerBound(i);
              q.vector()[i] = std::min(ub, std::max(lb, q.vector()[i]));
            }
          }
          return vectorToFloatSeq(q.vector());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeqSeq* Robot::getCurrentTransformation(const char* jointName)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  JointPtr_t joint = robot->getJointByName (jointName);
	  if (!joint) {
	    std::ostringstream oss ("Robot has no joint with name ");
	    oss  << jointName;
	    hppDout (error, oss.str ());
	    throw hpp::Error (oss.str ().c_str ());
	  }
	  
	  // join translation & rotation into homog. transformation 
	  const Transform3f& T = joint->currentTransformation ();
	  Eigen::Matrix< hpp::pinocchio::value_type, 4, 4 > trafo;
	  trafo.block<3,3>(0,0) = T.rotation();
	  trafo.block<3,1>(0,3) = T.translation();
	  trafo.block<1,4>(3,0) << 0, 0, 0, 1;
	  return matrixToFloatSeqSeq(trafo);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Transform__slice* Robot::getJointPosition(const char* jointName)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          return toHppTransform(frame.currentTransformation());
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      TransformSeq* Robot::getJointsPosition (const floatSeq& dofArray, const Names_t& jointNames)
      {
	try {
          using hpp::pinocchio::DeviceSync;
	  DevicePtr_t _robot = getRobotOrThrow(problemSolver());
          Configuration_t config = floatSeqToConfig (_robot, dofArray, true);
          DeviceSync robot (_robot);
          robot.currentConfiguration(config);
          robot.computeForwardKinematics();
	  robot.computeFramesForwardKinematics ();

          const Model& model (robot.model());
          const Data & data  (robot.data ());
          TransformSeq* transforms = new TransformSeq ();
          transforms->length (jointNames.length());
          std::string n;
          for (CORBA::ULong i = 0; i < jointNames.length (); ++i) {
            n = jointNames[i];
            if (!model.existFrame (n)) {
              HPP_THROW(Error, "Robot has no frame with name " << n);
            }
            FrameIndex joint = model.getFrameId(n);
            if (model.frames.size() <= (std::size_t)joint)
              HPP_THROW(Error, "Frame index of joint " << n << " out of bounds: " << joint);

            toHppTransform (data.oMf[joint], (*transforms)[i]);
          }
          return transforms;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
        }
      }

      // --------------------------------------------------------------------

      floatSeq* Robot::getJointVelocity(const char* jointName)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          vector_t w = frame.jacobian() * robot->currentVelocity();
          w.head<3>() = frame.currentTransformation().rotation() * w.head<3>();
          w.tail<3>() = frame.currentTransformation().rotation() * w.tail<3>();
          return vectorToFloatSeq(w);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      floatSeq* Robot::getJointVelocityInLocalFrame(const char* jointName)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          return vectorToFloatSeq(
              frame.jacobian() * robot->currentVelocity());
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Transform__slice* Robot::getJointPositionInParentFrame(const char* jointName)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          return toHppTransform (frame.positionInParentFrame());
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Long Robot::getJointNumberDof (const char* jointName)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          if (frame.isFixed()) return 0;
          JointPtr_t joint = frame.joint();
          return (joint ? (Long) joint->numberDof() : 0 );
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Long Robot::getJointConfigSize (const char* jointName)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          if (frame.isFixed()) return 0;
          JointPtr_t joint = frame.joint();
          return (joint ? (Long) joint->configSize() : 0 );
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      floatSeq* Robot::getJointBounds (const char* jointName)
      {
	try  {
	  // Get robot in ProblemSolver object.
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  // get joint
	  JointPtr_t joint = robot->getJointByName (jointName);
          if (!joint) {
            HPP_THROW(std::invalid_argument, "Joint " << jointName << " not found.");
          }
          return localGetJointBounds(joint);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setJointBounds (const char* jointName,
				  const floatSeq& jointBounds)
      {
	try  {
	  // Get robot in ProblemSolver object.
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());

	  // get joint
	  JointPtr_t joint = robot->getJointByName (jointName);
	  localSetJointBounds(joint, floatSeqToVector(jointBounds));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Transform__slice* Robot::getLinkPosition (const char* linkName)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          if (!robot->model().existBodyName (std::string(linkName))) {
            HPP_THROW(Error, "Robot has no link with name " << linkName);
          }
          FrameIndex body = robot->model().getBodyId(std::string(linkName));
          const ::pinocchio::Frame& frame = robot->model().frames[body];
          JointIndex joint = frame.parent;
          if (frame.type != ::pinocchio::BODY)
            HPP_THROW(Error, linkName << " is not a link");
          if (robot->model().joints.size() <= (std::size_t)joint)
            HPP_THROW(Error, "Joint index of link " << linkName << " out of bounds: " << joint);

          Transform3f T = robot->data().oMi[joint] * frame.placement;
          return toHppTransform (T);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
        }
      }

      // --------------------------------------------------------------------

      TransformSeq* Robot::getLinksPosition (const floatSeq& dofArray, const Names_t& linkNames)
      {
	try {
          using hpp::pinocchio::DeviceSync;
	  DevicePtr_t _robot = getRobotOrThrow(problemSolver());
          Configuration_t config = floatSeqToConfig (_robot, dofArray, true);
          DeviceSync robot (_robot);
          robot.currentConfiguration(config);
          robot.computeForwardKinematics();
	  robot.computeFramesForwardKinematics ();

          const Model& model (robot.model());
          const Data & data  (robot.data ());
          TransformSeq* transforms = new TransformSeq ();
          transforms->length (linkNames.length());
          std::string n;
          for (CORBA::ULong i = 0; i < linkNames.length (); ++i) {
            n = linkNames[i];
            if (!model.existBodyName (n)) {
              HPP_THROW(Error, "Robot has no link with name " << n);
            }
            FrameIndex body = model.getBodyId(n);
            const ::pinocchio::Frame& frame = model.frames[body];
            if (frame.type != ::pinocchio::BODY)
              HPP_THROW(Error, n << " is not a link");

            toHppTransform (data.oMf[body], (*transforms)[i]);
          }
          return transforms;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
        }
      }

      // --------------------------------------------------------------------

      Names_t* Robot::getLinkNames (const char* jointName)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          const Model& model = robot->model();
          Frame frame = robot->getFrameByName(jointName);

          std::vector<std::string> names;
          for (size_type i = 0; i < (size_type)model.frames.size(); ++i) {
            const ::pinocchio::Frame& f = model.frames[i];
            if (f.type == ::pinocchio::BODY && f.previousFrame == frame.index())
              names.push_back(f.name);
          }
          return toNames_t(names.begin(), names.end());
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setDimensionExtraConfigSpace (ULong dimension)
      {
	// Get robot in ProblemSolver object.
        try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          robot->setDimensionExtraConfigSpace (dimension);
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      ULong Robot::getDimensionExtraConfigSpace()
      {
  // Get robot in ProblemSolver object.
        ULong dim;
        try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          dim = (ULong)robot->extraConfigSpace().dimension();
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
        return dim;
      }

      // --------------------------------------------------------------------

      void Robot::setExtraConfigSpaceBounds
      (const hpp::floatSeq& bounds)
      {
	using hpp::pinocchio::ExtraConfigSpace;
	// Get robot in ProblemSolver object.
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          ExtraConfigSpace& extraCS = robot->extraConfigSpace ();
	  size_type nbBounds = (size_type)bounds.length();
	  size_type dimension = extraCS.dimension ();
	  if (nbBounds == 2*dimension) {
	    for (size_type iDof=0; iDof < dimension; ++iDof) {
	      double vMin = bounds [(ULong) (2*iDof)];
	      double vMax = bounds [(ULong) (2*iDof+1)];
	      if (vMin <= vMax) {
		/* Dof is actually bounded */
		extraCS.lower (iDof) = vMin;
		extraCS.upper (iDof) = vMax;
	      }
	      else {
		/* Dof is not bounded */
		extraCS.lower (iDof) = -std::numeric_limits<double>::infinity();
		extraCS.upper (iDof) = +std::numeric_limits<double>::infinity();
	      }
	    }
	  } else {
	    std::ostringstream oss ("Expected list of ");
	    oss << 2*dimension << "float, got " << nbBounds << ".";
	    throw hpp::Error (oss.str ().c_str ());
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setCurrentConfig(const hpp::floatSeq& dofArray)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  // Create a config for robot initialized with dof vector.
	  Configuration_t config = floatSeqToConfig (robot, dofArray, true);
	  robot->currentConfiguration (config);
	  robot->computeForwardKinematics ();
	  robot->computeFramesForwardKinematics ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::shootRandomConfig ()
      {
	try {
          core::ProblemPtr_t p = problemSolver()->problem();
          if (p) {
            Configuration_t q;
            p->configurationShooter()->shoot(q);
            return vectorToFloatSeq(q);
          } else {
            throw Error ("No problem in the ProblemSolver");
          }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getCurrentConfig()
      {
	try {
	  // Get robot in hppPlanner object.
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  vector_t config = robot->currentConfiguration ();
          assert(robot->configSize () == config.size());
          return vectorToFloatSeq (config);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getCurrentVelocity()
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  vector_t qDot = robot->currentVelocity ();
          assert(robot->numberDof () == qDot.size());
          return vectorToFloatSeq (qDot);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setCurrentVelocity(const hpp::floatSeq& qDot)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  robot->currentVelocity(floatSeqToVector (qDot, robot->numberDof()));
	  robot->computeForwardKinematics ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Names_t* Robot::getJointInnerObjects (const char* jointName)
      {
	Names_t *innerObjectSeq = 0x0;
	JointPtr_t joint;
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  joint = robot->getJointByName (std::string (jointName));
	  if (!joint) {
	    hppDout (error, "No joint with name " << jointName);
	    innerObjectSeq = new Names_t (0);
	    return innerObjectSeq;
	  }
	  BodyPtr_t body = joint->linkedBody ();
	  if (!body) {
	    hppDout (error, "Joint " << jointName << " has no body.");
	    innerObjectSeq = new Names_t (0);
	    return innerObjectSeq;
	  }

	  if (body->nbInnerObjects() > 0) {
	    // Allocate result now that the size is known.
	    ULong size = (ULong)body->nbInnerObjects();
	    char** nameList = Names_t::allocbuf(size);
	    innerObjectSeq = new Names_t(size, size, nameList);
	    for (size_type i=0; i < body->nbInnerObjects(); i++) {
	      CollisionObjectConstPtr_t object = body->innerObjectAt(i);
	      nameList[i] = c_str (object->name ());
	    }
	  } else {
	    innerObjectSeq = new Names_t (0);
	  }
	  return innerObjectSeq;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Names_t* Robot::getJointOuterObjects (const char* jointName)
      {
	Names_t *outerObjectSeq = 0x0;
	JointPtr_t joint;

	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  joint = robot->getJointByName (std::string (jointName));
	  if (!joint) {
	    hppDout (error, "No joint with name " << jointName);
	    outerObjectSeq = new Names_t (0);
	    return outerObjectSeq;
	  }
	  BodyPtr_t body = joint->linkedBody ();
	  if (!body) {
	    hppDout (error, "Joint " << jointName << " has no body.");
	    outerObjectSeq = new Names_t (0);
	    return outerObjectSeq;
	  }

	  if (body->nbOuterObjects() > 0) {
	    // Allocate result now that the size is known.
	    ULong size = (ULong)body->nbOuterObjects();
	    char** nameList = Names_t::allocbuf(size);
	    outerObjectSeq = new Names_t(size, size, nameList);
            for (size_type i=0; i < body->nbOuterObjects(); i++) {
	      CollisionObjectConstPtr_t object = body->outerObjectAt(i);
              nameList[i] = c_str (object->name ());
	    }
	  } else {
	    outerObjectSeq = new Names_t (0);
	  }
	  return outerObjectSeq;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      CollisionObjectConstPtr_t Robot::getObjectByName (const char* name)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  for (size_type i = 0; i < robot->nbObjects(); ++i) {
            CollisionObjectConstPtr_t object = robot->objectAt(i);
	    if (object->name () == name) {
	      return object;
	    }
	  }
	  throw std::runtime_error ("robot has no object with name " +
				    std::string (name));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::getObjectPosition (const char* objectName, Transform_ cfg)
      {
	try {
	  CollisionObjectConstPtr_t object = getObjectByName (objectName);
	  Transform3f transform = object->getTransform ();
	  toHppTransform (transform, cfg);
	  return;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::isConfigValid (const hpp::floatSeq& dofArray,
				 Boolean& validity, CORBA::String_out report)
      {
        try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  Configuration_t config = floatSeqToConfig (robot, dofArray, true);
	  core::ValidationReportPtr_t validationReport;
	  validity = problemSolver()->problem ()->configValidations ()->validate
	    (config, validationReport);
	  if (validity) {
	    report = CORBA::string_dup ("");
	  } else {
	    std::ostringstream oss; oss << *validationReport;
	    report = CORBA::string_dup(oss.str ().c_str ());
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void
      Robot::distancesToCollision (hpp::floatSeq_out distances,
				   Names_t_out innerObjects,
				   Names_t_out outerObjects,
				   hpp::floatSeqSeq_out innerPoints,
				   hpp::floatSeqSeq_out outerPoints)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  const DistanceBetweenObjectsPtr_t& distanceBetweenObjects =
	    problemSolver()->distanceBetweenObjects ();
	  robot->computeDistances ();
          const pinocchio::GeomModel& geomModel (robot->geomModel());
          const pinocchio::GeomData & geomData  (robot->geomData ());

	  distanceBetweenObjects->computeDistances ();
	  const CollisionPairs_t & cps = distanceBetweenObjects->collisionPairs ();
	  const DistanceResults_t& drs = distanceBetweenObjects->distanceResults ();

          const std::size_t nbAutoCol = geomModel.collisionPairs.size();
	  const std::size_t nbObsCol = drs.size ();
          const std::size_t nbDistPairs = nbAutoCol + nbObsCol;

          hpp::core::vector_t dists (nbDistPairs);
          std::vector<std::string> innerObj (nbDistPairs);
          std::vector<std::string> outerObj (nbDistPairs);
          hpp::core::matrix_t innerPts (nbDistPairs, 3);
          hpp::core::matrix_t outerPts (nbDistPairs, 3);

          for (std::size_t i = 0; i < nbAutoCol; ++i)
          {
            const ::pinocchio::CollisionPair& cp (geomModel.collisionPairs[i]);
	    dists    [i] = geomData.distanceResults[i].min_distance;
	    innerObj [i] = geomModel.geometryObjects[cp.first ].name;
	    outerObj [i] = geomModel.geometryObjects[cp.second].name;

            innerPts.row(i) = geomData.distanceResults[i].nearest_points[0];
            outerPts.row(i) = geomData.distanceResults[i].nearest_points[1];
	  }
	  for (std::size_t i = 0; i < nbObsCol; ++i)
          {
	    dists    [nbAutoCol + i] = drs[i].min_distance;
	    innerObj [nbAutoCol + i] = cps[i].first ->name();
	    outerObj [nbAutoCol + i] = cps[i].second->name();

            innerPts.row(nbAutoCol + i) = drs[i].nearest_points[0];
            outerPts.row(nbAutoCol + i) = drs[i].nearest_points[1];
	  }
	  distances = vectorToFloatSeq(dists);
	  innerObjects = toNames_t (innerObj.begin(), innerObj.end());
	  outerObjects = toNames_t (outerObj.begin(), outerObj.end());
	  innerPoints = matrixToFloatSeqSeq (innerPts);
	  outerPoints = matrixToFloatSeqSeq (outerPts);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void
      Robot::autocollisionCheck (hpp::boolSeq_out collide)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  robot->collisionTest (false);
          const pinocchio::GeomModel& geomModel (robot->geomModel());
          const pinocchio::GeomData & geomData  (robot->geomData ());

          const std::size_t nbColPairs = geomModel.collisionPairs.size();

          boolSeq* col_ptr = new boolSeq ();
          col_ptr->length ((ULong) nbColPairs);
          collide = col_ptr;

          for (std::size_t i = 0; i < nbColPairs; ++i)
	    (*col_ptr) [(ULong)i] = geomData.collisionResults[i].isCollision();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void
      Robot::autocollisionPairs (Names_t_out innerObjects,
                                 Names_t_out outerObjects,
                                 boolSeq_out active)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          const pinocchio::GeomModel& geomModel (robot->geomModel());
          const pinocchio::GeomData & geomData  (robot->geomData ());

          const std::size_t nbAutoCol = geomModel.collisionPairs.size();

          std::vector<std::string> innerObj (nbAutoCol);
          std::vector<std::string> outerObj (nbAutoCol);

          boolSeq* active_ptr = new boolSeq ();
          active_ptr->length ((ULong) nbAutoCol);
          active = active_ptr;

          for (std::size_t i = 0; i < nbAutoCol; ++i)
          {
            const ::pinocchio::CollisionPair& cp (geomModel.collisionPairs[i]);
	    innerObj [i] = geomModel.geometryObjects[cp.first ].name;
	    outerObj [i] = geomModel.geometryObjects[cp.second].name;
	    (*active_ptr) [(ULong)i] = geomData.activeCollisionPairs[i];
	  }
	  innerObjects = toNames_t (innerObj.begin(), innerObj.end());
	  outerObjects = toNames_t (outerObj.begin(), outerObj.end());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void
      Robot::setAutoCollision (const char* innerObject,
                               const char* outerObject,
                               bool active)
      {
	try {
          core::ProblemSolverPtr_t ps = problemSolver();
	  DevicePtr_t robot = getRobotOrThrow(ps);

          const pinocchio::GeomModel& geomModel (robot->geomModel());
          pinocchio::GeomData & geomData  (robot->geomData ());

          if (!geomModel.existGeometryName(innerObject))
            HPP_THROW(Error, "Object " << innerObject << " not found");
          if (!geomModel.existGeometryName(outerObject))
            HPP_THROW(Error, "Object " << outerObject << " not found");
          pinocchio::GeomIndex idxI = geomModel.getGeometryId(innerObject);
          pinocchio::GeomIndex idxO = geomModel.getGeometryId(outerObject);
          ::pinocchio::PairIndex pid = geomModel.findCollisionPair(::pinocchio::CollisionPair(idxI, idxO));

          if (pid >= geomModel.collisionPairs.size()) {
            HPP_THROW(Error, "Collision pair (" << innerObject << ", " << outerObject << ") not found");
          }

          if (active)
            geomData.activateCollisionPair(pid);
          else
            geomData.deactivateCollisionPair(pid);

          ps->initValidations();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Double Robot::getMass ()
      {
	try {
	  return getRobotOrThrow(problemSolver())->mass ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getCenterOfMass ()
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          pinocchio::Computation_t flags = robot->computationFlag();
          if (!(flags&pinocchio::COM)) {
            robot->controlComputation (pinocchio::COMPUTE_ALL);
            robot->computeForwardKinematics ();
            robot->controlComputation (flags);
          }
	  return vectorToFloatSeq (robot->positionCenterOfMass());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getCenterOfMassVelocity ()
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          pinocchio::Computation_t flags = robot->computationFlag();
          if (!(flags&pinocchio::COM) || (flags&pinocchio::JACOBIAN)) {
            robot->controlComputation (pinocchio::COMPUTE_ALL);
            robot->computeForwardKinematics ();
            robot->controlComputation (flags);
          }
          return vectorToFloatSeq (
              robot->jacobianCenterOfMass() * robot->currentVelocity());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeqSeq* Robot::getJacobianCenterOfMass ()
      {
	try {
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
          pinocchio::Computation_t flags = robot->computationFlag();
          if (!(flags&pinocchio::COM) || (flags&pinocchio::JACOBIAN)) {
            robot->controlComputation (pinocchio::COMPUTE_ALL);
            robot->computeForwardKinematics ();
            robot->controlComputation (flags);
          }
	  const ComJacobian_t& jacobian = robot->jacobianCenterOfMass ();
	  return matrixToFloatSeqSeq (jacobian);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::createPolyhedron(const char* polyhedronName)
      {
        objectMap_.createPolyhedron (polyhedronName);
      }

      // --------------------------------------------------------------------

      void Robot::createBox(const char* name, Double x, Double y, Double z)
      {
        objectMap_.createBox (name, x, y, z);
      }

      // --------------------------------------------------------------------

      void Robot::createSphere (const char* name, Double radius)
      {
        objectMap_.createSphere (name, radius);
      }

      // --------------------------------------------------------------------

      void Robot::createCylinder (const char* name, Double radius, Double length)
      {
        objectMap_.createCylinder (name, radius, length);
      }

      // --------------------------------------------------------------------

      ULong Robot::addPoint(const char* polyhedronName, Double x, Double y,
			    Double z)
      {
	return static_cast<Long> (
            objectMap_.addPoint (polyhedronName, x, y, z));
      }

      // --------------------------------------------------------------------

      ULong Robot::addTriangle(const char* polyhedronName,
			       ULong pt1, ULong pt2, ULong pt3)
      {
	return static_cast<ULong> (
            objectMap_.addTriangle (polyhedronName, pt1, pt2, pt3));
      }

      // --------------------------------------------------------------------

      void Robot::addObjectToJoint (const char* jointName,
          const char* objectName,
          const Transform_ pos)
      {
        using namespace pinocchio;
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
        Model& model = robot->model();
        GeomModel& geomModel = robot->geomModel();

        std::string jn(jointName);
        std::string on(objectName);

        // Find parent joint
        if (!model.existJointName(jn))
          throw Error (("Joint " + jn + " not found.").c_str());
        JointIndex jid = model.getJointId (jn);

        Transform3f placement = toTransform3f (pos);

        try {
          CollisionGeometryPtr_t geometry = objectMap_.geometry (objectName);

          FrameIndex bid = model.addBodyFrame (on, jid, placement);
          GeometryObject geom (on,
              bid, jid,
              geometry, placement);
          GeomIndex idx = geomModel.addGeometryObject (geom, model);
          for (GeomIndex i = 0; i < idx; ++i)
          {
            if (geomModel.geometryObjects[i].parentJoint != jid)
                geomModel.addCollisionPair(::pinocchio::CollisionPair(i,idx));
          }

          robot->createData();
          robot->createGeomData();
          problemSolver()->resetProblem();
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      void Robot::addPartialCom (const char* comName, const Names_t& jointNames)
      {
        try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          pinocchio::CenterOfMassComputationPtr_t comc =
            pinocchio::CenterOfMassComputation::create (robot);

          for (ULong i=0; i<jointNames.length (); ++i) {
            std::string name (jointNames[i]);
            JointPtr_t j = robot->getJointByName (name);
            if (!j) throw hpp::Error ("One joint not found.");
            comc->add (j);
          }
          problemSolver()->centerOfMassComputations.add (std::string (comName), comc);
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      hpp::floatSeq* Robot::getPartialCom (const char* comName)
      {
	try {
          if (!problemSolver()->centerOfMassComputations.has(comName)) {
            HPP_THROW(std::invalid_argument, "Partial COM " << comName << " not found.");
          }
          pinocchio::CenterOfMassComputationPtr_t comc =
            problemSolver()->centerOfMassComputations.get(comName);
          comc->compute (hpp::pinocchio::COM);
	  return vectorToFloatSeq (comc->com());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      hpp::floatSeq* Robot::getVelocityPartialCom (const char* comName)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          if (!problemSolver()->centerOfMassComputations.has(comName)) {
            HPP_THROW(std::invalid_argument, "Partial COM " << comName << " not found.");
          }
          pinocchio::CenterOfMassComputationPtr_t comc =
            problemSolver()->centerOfMassComputations.get(comName);
          comc->compute (hpp::pinocchio::JACOBIAN);
          return vectorToFloatSeq (comc->jacobian() * robot->currentVelocity());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      hpp::floatSeqSeq* Robot::getJacobianPartialCom (const char* comName)
      {
	try {
          if (!problemSolver()->centerOfMassComputations.has(comName)) {
            HPP_THROW(std::invalid_argument, "Partial COM " << comName << " not found.");
          }
          pinocchio::CenterOfMassComputationPtr_t comc =
            problemSolver()->centerOfMassComputations.get(comName);
          comc->compute (hpp::pinocchio::JACOBIAN);
	  return matrixToFloatSeqSeq (comc->jacobian());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      hpp::pinocchio_idl::CenterOfMassComputation_ptr Robot::getCenterOfMassComputation
        (const char* name)
      {
        const std::string cn (name);
        core::ProblemSolverPtr_t ps = problemSolver();
        if (!ps->centerOfMassComputations.has(cn))
          throw Error (("CenterOfMassComputation " + cn + " not found").c_str());

        hpp::pinocchio_idl::CenterOfMassComputation_var d = makeServantDownCast <pinocchio_impl::CenterOfMassComputation>
          (server_->parent(), ps->centerOfMassComputations.get(cn));
        return d._retn();
      }

      floatSeq* Robot::getRobotAABB()
      {
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
        try {
          fcl::AABB aabb = robot->computeAABB();
          vector_t v (6);
          v.head<3>() = aabb.min_;
          v.tail<3>() = aabb.max_;
          return vectorToFloatSeq(v);
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }
    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.
