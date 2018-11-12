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

#include <hpp/core/problem.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/distance-between-objects.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/corbaserver/server.hh>
#include "robot.impl.hh"
#include "tools.hh"

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

      Robot::Robot(corbaServer::Server* server) :
	server_(server)
      {
      }

      // --------------------------------------------------------------------

      core::ProblemSolverPtr_t Robot::problemSolver()
      {
        return server_->problemSolver();
      }

      // --------------------------------------------------------------------

      void Robot::createRobot(const char* inRobotName)
	throw (hpp::Error)
      {
	std::string robotName (inRobotName);
        DevicePtr_t robot (problemSolver()->createRobot (robotName));
        problemSolver ()->robot (robot);
      }

      // --------------------------------------------------------------------

      char* Robot::getRobotName () throw (hpp::Error)
      {
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
        const std::string& str = robot->name();
        char* name = new char[str.length ()+1];
        strcpy (name, str.c_str ());
        return name;
      }

      // --------------------------------------------------------------------

      void Robot::loadRobotModel(const char* robotName,
				 const char* rootJointType,
				 const char* packageName,
				 const char* modelName,
				 const char* urdfSuffix,
				 const char* srdfSuffix) throw (hpp::Error)
      {
	try {
	  pinocchio::DevicePtr_t device
            (problemSolver ()->createRobot (robotName));
	  hpp::pinocchio::urdf::loadRobotModel (device,
					    std::string (rootJointType),
					    std::string (packageName),
					    std::string (modelName),
					    std::string (urdfSuffix),
					    std::string (srdfSuffix));
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
				     const char* packageName,
				     const char* modelName,
				     const char* urdfSuffix,
				     const char* srdfSuffix) throw (hpp::Error)
      {
	try {
	  pinocchio::HumanoidRobotPtr_t robot
	    (pinocchio::HumanoidRobot::create (robotName));
	  hpp::pinocchio::urdf::loadRobotModel (robot,
					        std::string (rootJointType),
					        std::string (packageName),
					        std::string (modelName),
					        std::string (urdfSuffix),
					        std::string (srdfSuffix));
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
          const char* srdfString) throw (hpp::Error)
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
          const char* srdfString) throw (hpp::Error)
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

      Long Robot::getConfigSize () throw (hpp::Error)
      {
	try {
	  return (Long) getRobotOrThrow(problemSolver())->configSize ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Long Robot::getNumberDof () throw (hpp::Error)
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
	throw (hpp::Error)
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

      Names_t* Robot::getJointNames () throw (hpp::Error)
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

      Names_t* Robot::getJointTypes () throw (hpp::Error)
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

      Names_t* Robot::getAllJointNames () throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = problemSolver()->robot ();
          if (!robot)
            return new Names_t (0, 0, Names_t::allocbuf (0));
	  // Compute number of real urdf joints
          const Model& model = robot->model();
          std::vector<std::string> jns;
          for(std::size_t i = 0; i < model.frames.size(); ++i) {
            if(    model.frames[i].type == ::pinocchio::JOINT
                || model.frames[i].type == ::pinocchio::FIXED_JOINT)
              jns.push_back(model.frames[i].name);
          }
	  return toNames_t (jns.begin(), jns.end());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Names_t* Robot::getChildJointNames (const char* jointName)
        throw (hpp::Error)
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
	    std::string name = joint->name ();
	    nameList [i] =
	      (char*) malloc (sizeof(char)*(name.length ()+1));
	    strcpy (nameList [i], name.c_str ());
	  }
	  return jointNames;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      char* Robot::getParentJointName (const char* jointName)
        throw (hpp::Error)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          const std::string jn (jointName);
          const Frame f = robot->getFrameByName(jn);

          char* name;
          if (f.isRootFrame()) {
            name = CORBA::string_dup("");
          } else {
            const std::string str = f.parentFrame().name();
            name = CORBA::string_dup(str.c_str());
          }
          return name;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------
      Transform__slice* Robot::getRootJointPosition () throw (hpp::Error)
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
	throw (hpp::Error)
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
	throw (hpp::Error)
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
	throw (hpp::Error)
      {
	try {
	  // Get robot in hppPlanner object.
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          if (frame.isFixed())
            return new hpp::floatSeq();
	  Joint joint = frame.joint();
          vector_t config = robot->currentConfiguration ();
          size_type ric = joint.rankInConfiguration ();
	  size_type dim = joint.configSize ();
          return vectorToFloatSeq(config.segment(ric, dim));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      char* Robot::getJointType(const char* jointName) throw (hpp::Error)
      {
	try {
	  // Get robot in hppPlanner object.
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          std::string name;
          if (frame.isFixed())
            return CORBA::string_dup("anchor");
	  Joint joint = frame.joint();
          return CORBA::string_dup(joint.jointModel().shortname().c_str());
        } catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setJointConfig(const char* jointName, const hpp::floatSeq& q)
	throw (hpp::Error)
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

      void Robot::jointIntegrate(const char* jointName, const hpp::floatSeq& dq)
	throw (hpp::Error)
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
	  const size_type nv  = joint->numberDof (),
                          riv = joint->rankInVelocity ();
          if (nv != (size_type) dq.length ()) {
	    throw Error ("Wrong speed dimension");
	  }
          vector_t dqAll (vector_t::Zero(robot->numberDof ()));
          dqAll .segment(riv, nv) = floatSeqToVector (dq, nv);

          vector_t config (robot->currentConfiguration ());
          pinocchio::integrate<true, pinocchio::RnxSOnLieGroupMap>
            (robot, config, dqAll, config);

          robot->currentConfiguration (config);
          robot->computeForwardKinematics ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeqSeq* Robot::getCurrentTransformation(const char* jointName)
	throw (hpp::Error)
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
	throw (hpp::Error)
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

      floatSeq* Robot::getJointVelocity(const char* jointName)
	throw (hpp::Error)
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
	throw (hpp::Error)
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
	throw (hpp::Error)
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

      Long Robot::getJointNumberDof (const char* jointName) throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          if (frame.isFixed()) return 0;
          else                 return (Long) frame.joint().numberDof();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Long Robot::getJointConfigSize (const char* jointName) throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          if (frame.isFixed()) return 0;
          else                 return (Long) frame.joint().configSize();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      floatSeq* Robot::getJointBounds (const char* jointName)
	throw (hpp::Error)
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
	throw (hpp::Error)
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
	throw (hpp::Error)
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

      TransformSeq* Robot::getLinksPosition (const Names_t& linkNames)
	throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          const Model& model (robot->model());
          const Data & data  (robot->data ());
          TransformSeq* transforms = new TransformSeq ();
          transforms->length (linkNames.length());
          Transform3f T;
          std::string n;
          for (CORBA::ULong i = 0; i < linkNames.length (); ++i) {
            n = linkNames[i];
            if (!model.existBodyName (n)) {
              HPP_THROW(Error, "Robot has no link with name " << n);
            }
            FrameIndex body = model.getBodyId(n);
            const ::pinocchio::Frame& frame = model.frames[body];
            JointIndex joint = frame.parent;
            if (frame.type != ::pinocchio::BODY)
              HPP_THROW(Error, n << " is not a link");
            if (model.joints.size() <= (std::size_t)joint)
              HPP_THROW(Error, "Joint index of link " << n << " out of bounds: " << joint);

            T = data.oMi[joint] * frame.placement;
            toHppTransform (T, (*transforms)[i]);
          }
          return transforms;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
        }
      }

      // --------------------------------------------------------------------

      Names_t* Robot::getLinkNames (const char* jointName) throw (hpp::Error)
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
	throw (hpp::Error)
      {
	// Get robot in ProblemSolver object.
        try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          robot->setDimensionExtraConfigSpace (dimension);
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // --------------------------------------------------------------------

      void Robot::setExtraConfigSpaceBounds
      (const hpp::floatSeq& bounds) throw (hpp::Error)
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
	throw (hpp::Error)
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

      hpp::floatSeq* Robot::shootRandomConfig () throw (hpp::Error)
      {
	try {
          core::ProblemPtr_t p = problemSolver()->problem();
          if (p) {
            ConfigurationPtr_t q = p->configurationShooter()->shoot();
            return vectorToFloatSeq(*q);
          } else {
            throw Error ("No problem in the ProblemSolver");
          }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getCurrentConfig() throw (hpp::Error)
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

      hpp::floatSeq* Robot::getCurrentVelocity() throw (hpp::Error)
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
	throw (hpp::Error)
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
	throw (hpp::Error)
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
	      std::string geomName = object->name();
	      nameList[i] = (char*)malloc(sizeof(char)*(geomName.length()+1));
	      strcpy(nameList[i], geomName.c_str());
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
	throw (hpp::Error)
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
	      std::string geomName = object->name();
	      nameList[i] = (char*)malloc(sizeof(char)*(geomName.length()+1));
	      strcpy(nameList[i], geomName.c_str());
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
	throw (hpp::Error)
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
	throw (hpp::Error)
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
	throw (hpp::Error)
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
	throw (hpp::Error)
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
	throw (hpp::Error)
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
	throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());

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

          geomData.activateCollisionPair(pid, active);

          problemSolver()->initPathValidation();
          problemSolver()->problem()->resetConfigValidations();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Double Robot::getMass () throw (hpp::Error)
      {
	try {
	  return getRobotOrThrow(problemSolver())->mass ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getCenterOfMass () throw (hpp::Error)
      {
	try {
	  return vectorToFloatSeq (
              getRobotOrThrow(problemSolver())->positionCenterOfMass ());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getCenterOfMassVelocity () throw (hpp::Error)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          return vectorToFloatSeq (
              robot->jacobianCenterOfMass() * robot->currentVelocity());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeqSeq* Robot::getJacobianCenterOfMass () throw (hpp::Error)
      {
	try {
	  const ComJacobian_t& jacobian =
	    getRobotOrThrow(problemSolver())->jacobianCenterOfMass ();
	  return matrixToFloatSeqSeq (jacobian);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::createPolyhedron(const char* polyhedronName)
	throw (hpp::Error)
      {
        objectMap_.createPolyhedron (polyhedronName);
      }

      // --------------------------------------------------------------------

      void Robot::createBox(const char* name, Double x, Double y, Double z)
	throw (hpp::Error)
      {
        objectMap_.createBox (name, x, y, z);
      }

      // --------------------------------------------------------------------

      void Robot::createSphere (const char* name, Double radius)
	throw (hpp::Error)
      {
        objectMap_.createSphere (name, radius);
      }

      // --------------------------------------------------------------------

      void Robot::createCylinder (const char* name, Double radius, Double length)
	throw (hpp::Error)
      {
        objectMap_.createCylinder (name, radius, length);
      }

      // --------------------------------------------------------------------

      ULong Robot::addPoint(const char* polyhedronName, Double x, Double y,
			    Double z)
	throw (hpp::Error)
      {
	return static_cast<Long> (
            objectMap_.addPoint (polyhedronName, x, y, z));
      }

      // --------------------------------------------------------------------

      ULong Robot::addTriangle(const char* polyhedronName,
			       ULong pt1, ULong pt2, ULong pt3)
	throw (hpp::Error)
      {
	return static_cast<ULong> (
            objectMap_.addTriangle (polyhedronName, pt1, pt2, pt3));
      }

      // --------------------------------------------------------------------

      void Robot::addObjectToJoint (const char* jointName,
          const char* objectName,
          const Transform_ pos)
        throw (hpp::Error)
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
          GeomIndex idx = geomModel.ngeoms;
          geomModel.addGeometryObject (geom, model);
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
        throw (hpp::Error)
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

      hpp::floatSeq* Robot::getPartialCom (const char* comName) throw (hpp::Error)
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
        throw (hpp::Error)
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
          

      hpp::floatSeqSeq* Robot::getJacobianPartialCom (const char* comName) throw (hpp::Error)
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

      floatSeq* Robot::getRobotAABB()
        throw (hpp::Error)
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
