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

#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>

#include <hpp/pinocchio/fwd.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/frame.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/urdf/util.hh>
// #include <hpp/pinocchio/object-factory.hh>
// #include <hpp/pinocchio/object-iterator.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>
#include <hpp/pinocchio/configuration.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/basic-configuration-shooter.hh>
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
      namespace
      {
        typedef se3::FrameIndex FrameIndex;
        const se3::FrameType JOINT_FRAME = (se3::FrameType)(se3::JOINT | se3::FIXED_JOINT);

	static void localSetJointBounds(const JointPtr_t& joint,
					vector_t jointBounds)
	{
          static const value_type inf = std::numeric_limits<value_type>::infinity();
          if (jointBounds.size() %2 == 1) {
            HPP_THROW(std::invalid_argument, "Expect a vector of even size");
          }
          for (std::size_t i = 0; i < jointBounds.size() / 2; i++) {
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
          ret->length ((CORBA::ULong) (2*jointNbDofs));
          for (std::size_t iDof=0; iDof<jointNbDofs; iDof++) {
            if (joint->isBounded (iDof)) {
	      (*ret) [(CORBA::ULong) (2*iDof)] = joint->lowerBound (iDof);
	      (*ret) [(CORBA::ULong) (2*iDof + 1)] = joint->upperBound (iDof);
            } else {
	      (*ret) [(CORBA::ULong) (2*iDof)] = 1;
	      (*ret) [(CORBA::ULong) (2*iDof + 1)] = 0;
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

//DEPREC      // --------------------------------------------------------------------
//DEPREC
//DEPREC      void Robot::createRobot(const char* inRobotName)
//DEPREC	throw (hpp::Error)
//DEPREC      {
//DEPREC	std::string robotName (inRobotName);
//DEPREC	// Check that no robot of this name already exists.
//DEPREC	if (robotMap_.count (robotName) != 0) {
//DEPREC	  std::ostringstream oss ("robot ");
//DEPREC	  oss << robotName << " already exists.";
//DEPREC	  hppDout (error, oss.str ());
//DEPREC	  throw hpp::Error (oss.str ().c_str ());
//DEPREC	}
//DEPREC	// Try to create a robot.
//DEPREC	DevicePtr_t robot = Device_t::create (robotName);
//DEPREC	// Store robot in map.
//DEPREC	robotMap_ [robotName] = robot;
//DEPREC      }
//DEPREC
//DEPREC      // --------------------------------------------------------------------
//DEPREC
//DEPREC      void Robot::setRobot(const char* inRobotName) throw (hpp::Error)
//DEPREC      {
//DEPREC	std::string robotName (inRobotName);
//DEPREC	// Check that robot of this name exists.
//DEPREC	if (robotMap_.count (robotName) != 1) {
//DEPREC	  std::ostringstream oss ("robot ");
//DEPREC	  oss << robotName << " does not exists.";
//DEPREC	  hppDout (error, oss.str ());
//DEPREC	  throw Error (oss.str ().c_str ());
//DEPREC	}
//DEPREC	DevicePtr_t robot = robotMap_ [robotName];
//DEPREC	// Create a new problem with this robot.
//DEPREC	problemSolver()->robot (robot);
//DEPREC      }

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

//DEPREC      void Robot::setRobotRootJoint(const char* inRobotName,
//DEPREC				     const char* inJointName)
//DEPREC	throw (hpp::Error)
//DEPREC      {
//DEPREC	std::string robotName(inRobotName);
//DEPREC
//DEPREC	// Check that robot of this name exists.
//DEPREC	if (robotMap_.count (robotName) != 1) {
//DEPREC	  std::ostringstream oss ("robot ");
//DEPREC	  oss << robotName << " does not exists.";
//DEPREC	  hppDout (error, oss.str ());
//DEPREC	  throw hpp::Error (oss.str ().c_str ());
//DEPREC	}
//DEPREC	DevicePtr_t robot = robotMap_ [robotName];
//DEPREC	JointPtr_t joint = getJointByName (inJointName);
//DEPREC
//DEPREC	robot->rootJoint (joint);
//DEPREC      }

      // --------------------------------------------------------------------

      void Robot::loadRobotModel(const char* robotName,
				 const char* rootJointType,
				 const char* packageName,
				 const char* modelName,
				 const char* urdfSuffix,
				 const char* srdfSuffix) throw (hpp::Error)
      {
	try {
	  pinocchio::DevicePtr_t device (pinocchio::Device::create (robotName));
	  hpp::pinocchio::urdf::loadRobotModel (device,
					    std::string (rootJointType),
					    std::string (packageName),
					    std::string (modelName),
					    std::string (urdfSuffix),
					    std::string (srdfSuffix));
	  // Add device to the planner
	  problemSolver()->robot (device);
	  problemSolver()->robot ()->controlComputation
	    (pinocchio::Device::JOINT_POSITION);
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
	  hpp::pinocchio::urdf::loadHumanoidModel (robot,
					       std::string (rootJointType),
					       std::string (packageName),
					       std::string (modelName),
					       std::string (urdfSuffix),
					       std::string (srdfSuffix));
	  // Add robot to the planner
	  problemSolver()->robot (robot);
	  problemSolver()->robot ()->controlComputation
	    (pinocchio::Device::JOINT_POSITION);
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Short Robot::getConfigSize () throw (hpp::Error)
      {
	try {
	  return (Short) getRobotOrThrow(problemSolver())->configSize ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Short Robot::getNumberDof () throw (hpp::Error)
      {
	try {
          return (Short) getRobotOrThrow(problemSolver())->numberDof ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

//DEPREC      void Robot::createJoint (const char* jointName,
//DEPREC			        const char* jointType,
//DEPREC				const Double* pos,
//DEPREC				const jointBoundSeq& jointBounds)
//DEPREC	throw (hpp::Error)
//DEPREC      {
//DEPREC	std::string jn(jointName);
//DEPREC	std::string jt(jointType);
//DEPREC
//DEPREC	// Check that joint of this name does not already exist.
//DEPREC	if (jointMap_.count(jn) != 0) {
//DEPREC	  std::ostringstream oss ("joint ");
//DEPREC	  oss << jn << " already exists.";
//DEPREC	  hppDout (error, oss.str ());
//DEPREC	  throw hpp::Error (oss.str ().c_str ());
//DEPREC	}
//DEPREC	JointPtr_t joint;
//DEPREC	// Fill position matrix
//DEPREC	Transform3f posMatrix;
//DEPREC	hppTransformToTransform3f (pos, posMatrix);
//DEPREC
//DEPREC	// Determine type of joint.
//DEPREC	if (jt == "anchor") {
//DEPREC	  joint = objectFactory_.createJointAnchor (posMatrix);
//DEPREC	}
//DEPREC	else if (jt == "SO3") {
//DEPREC	  joint =
//DEPREC	    objectFactory_.createJointSO3 (posMatrix);
//DEPREC	}
//DEPREC	else if (jt == "bounded-rotation") {
//DEPREC	  joint = objectFactory_.createBoundedJointRotation (posMatrix);
//DEPREC	}
//DEPREC	else if (jt == "unbounded-rotation") {
//DEPREC	  joint = objectFactory_.createUnBoundedJointRotation (posMatrix);
//DEPREC	}
//DEPREC	else if (jt == "translation") {
//DEPREC	  joint =
//DEPREC	    objectFactory_.createJointTranslation (posMatrix);
//DEPREC	}
//DEPREC	else if (jt == "translation2") {
//DEPREC	  joint =
//DEPREC	    objectFactory_.createJointTranslation2 (posMatrix);
//DEPREC	}
//DEPREC	else if (jt == "translation3") {
//DEPREC	  joint =
//DEPREC	    objectFactory_.createJointTranslation3 (posMatrix);
//DEPREC	}
//DEPREC	else if (jt == "rotation") {
//DEPREC	  throw hpp::Error ("joint type \"rotation\" is not supported anymore,"
//DEPREC			    "please choose between \"bounded-rotation\" "
//DEPREC			    "and \"unbounded-rotation\"");
//DEPREC	}
//DEPREC	else {
//DEPREC	  std::ostringstream oss ("joint type");
//DEPREC	  oss << jt << " does not exist.";
//DEPREC	  hppDout (error, oss.str ());
//DEPREC	  throw hpp::Error (oss.str ().c_str ());
//DEPREC	}
//DEPREC	// Set the bounds of the joint
//DEPREC	// Bound joint if needed.
//DEPREC	localSetJointBounds(joint, jointBounds);
//DEPREC	BodyPtr_t body = objectFactory_.createBody ();
//DEPREC	body->name (jn);
//DEPREC	joint->setLinkedBody (body);
//DEPREC	// Store joint in jointMap_.
//DEPREC	jointMap_[jn] = joint;
//DEPREC	joint->name (jn);
//DEPREC      }
//DEPREC
//DEPREC      // --------------------------------------------------------------------
//DEPREC
//DEPREC      void Robot::addJoint(const char* inParentName,
//DEPREC			    const char* inChildName)
//DEPREC	throw (hpp::Error)
//DEPREC      {
//DEPREC	JointPtr_t parentJoint = getJointByName (inParentName);
//DEPREC	JointPtr_t childJoint = getJointByName (inChildName);
//DEPREC	parentJoint->addChildJoint (childJoint);
//DEPREC      }

      // --------------------------------------------------------------------

      Names_t* Robot::getJointNames () throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  // Compute number of real urdf joints
	  ULong size = 0;
	  JointVector_t jointVector = robot->getJointVector ();
	  for (JointVector_t::const_iterator it = jointVector.begin ();
	       it != jointVector.end (); it++) {
	    if ((*it)->numberDof () != 0) size ++;
	  }
	  char** nameList = Names_t::allocbuf(size);
	  Names_t *jointNames = new Names_t (size, size, nameList);
	  std::size_t rankInConfig = 0;
	  for (size_type i = 0; i < jointVector.size (); ++i) {
	    const JointPtr_t joint = jointVector [i];
	    std::string name = joint->name ();
	    std::size_t dimension = joint->numberDof ();
	    if (dimension != 0) {
	      nameList [rankInConfig] =
		(char*) malloc (sizeof(char)*(name.length ()+1));
	      strcpy (nameList [rankInConfig], name.c_str ());
	      ++rankInConfig;
	    }
	  }
	  return jointNames;
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
          const se3::Model& model = robot->model();
          std::vector<std::string> jns;
          for(std::size_t i = 0; i < model.frames.size(); ++i) {
            if(    model.frames[i].type == se3::JOINT
                || model.frames[i].type == se3::FIXED_JOINT)
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
	  JointVector_t jointVector = robot->getJointVector ();
	  ULong size = (CORBA::ULong) j->numberChildJoints ();
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
          const se3::Model& model = robot->model();
          if (!model.existFrame(jn, JOINT_FRAME)) {
            HPP_THROW(std::invalid_argument, jn << " not found.");
          }
          se3::FrameIndex fid = model.getFrameId(jn, JOINT_FRAME);
          char* name;
          if (fid == 0) {
            name = CORBA::string_dup("");
          } else {
            se3::FrameIndex pfid = model.frames[fid].previousFrame;
            assert (pfid >= 0 && pfid < model.frames.size());

            const std::string& str = model.frames[pfid].name;
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
          return vectorToFloatseq(config.segment(ric, dim));
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
            config [ric + i] = q[(CORBA::ULong) i];
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
	                  nq  = joint->configSize (),
                          ric = joint->rankInConfiguration (),
                          riv = joint->rankInVelocity ();
          if (nv != (size_type) dq.length ()) {
	    throw Error ("Wrong speed dimension");
	  }
          vector_t dqAll (vector_t::Zero(robot->numberDof ()));
          dqAll .segment(riv, nv) = floatSeqToVector (dq, nv);

          vector_t config (robot->currentConfiguration ());
          config.segment(ric, nq) = joint->jointModel().integrate (config, dqAll);

          pinocchio::saturate (robot, config);

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

      Short Robot::getJointNumberDof (const char* jointName) throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          if (frame.isFixed()) return 0;
          else                 return (Short) frame.joint().numberDof();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Short Robot::getJointConfigSize (const char* jointName) throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          Frame frame = robot->getFrameByName(jointName);
          if (frame.isFixed()) return 0;
          else                 return (Short) frame.joint().configSize();
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
          se3::FrameIndex body = robot->model().getBodyId(std::string(linkName));
          const se3::Frame& frame = robot->model().frames[body];
          se3::JointIndex joint = frame.parent;
          if (frame.type != se3::BODY)
            HPP_THROW(Error, linkName << " is not a link");
          if (robot->model().joints.size() < (size_type)joint)
            HPP_THROW(Error, "Joint index of link " << linkName << " out of bounds: " << joint);

          Transform3f T = robot->data().oMi[joint] * frame.placement;
          return toHppTransform (T);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
        }
      }

      // --------------------------------------------------------------------

      Names_t* Robot::getLinkNames (const char* jointName) throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          const se3::Model& model = robot->model();
          Frame frame = robot->getFrameByName(jointName);

          std::vector<std::string> names;
          for (size_type i = 0; i < model.frames.size(); ++i) {
            const se3::Frame& f = model.frames[i];
            if (f.type == se3::BODY && f.previousFrame == frame.index())
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
	      double vMin = bounds [(CORBA::ULong) (2*iDof)];
	      double vMax = bounds [(CORBA::ULong) (2*iDof+1)];
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

      static pinocchio::Configuration_t
      dofArrayToConfig (const core::ProblemSolverPtr_t& problemSolver,
			const hpp::floatSeq& dofArray)
      {
	size_type configDim = (size_type)dofArray.length();
	std::vector<double> dofVector;
	// Get robot
        DevicePtr_t robot = getRobotOrThrow(problemSolver);
	size_type deviceDim = robot->configSize ();
	// Fill dof vector with dof array.
	Configuration_t config; config.resize (configDim);
	for (size_type iDof = 0; iDof < configDim; iDof++) {
	  config [iDof] = dofArray[(CORBA::ULong) iDof];
	}
	// fill the vector by zero
	hppDout (info, "config dimension: " <<configDim
		 <<",  deviceDim "<<deviceDim);
	if(configDim != deviceDim){
	  throw hpp::Error ("dofVector Does not match");
	}
	return config;
      }

      // --------------------------------------------------------------------

      void Robot::setCurrentConfig(const hpp::floatSeq& dofArray)
	throw (hpp::Error)
      {
	try {
	  Configuration_t config = dofArrayToConfig (problemSolver(), dofArray);
	  // Create a config for robot initialized with dof vector.
	  getRobotOrThrow(problemSolver())->currentConfiguration (config);
	  getRobotOrThrow(problemSolver())->computeForwardKinematics ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::shootRandomConfig () throw (hpp::Error)
      {
	try {
	  hpp::floatSeq *dofArray = 0x0;
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  hpp::core::BasicConfigurationShooterPtr_t shooter
			= core::BasicConfigurationShooter::create (robot);
	  ConfigurationPtr_t configuration = shooter->shoot();

	  size_type deviceDim = robot->configSize ();
	  dofArray = new hpp::floatSeq();
	  dofArray->length ((CORBA::ULong) deviceDim);
	  for(size_type i=0; i<deviceDim; i++)
	    (*dofArray)[(CORBA::ULong) i] = (*configuration) [i];
	  return dofArray;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getComPosition () throw (hpp::Error)
      {
	hpp::floatSeq *dofArray;
	try {
	  // Get robot in hppPlanner object.
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  const vector3_t& com = robot->positionCenterOfMass ();
	  dofArray = new hpp::floatSeq();
	  dofArray->length(3);
	  for(size_type i=0; i<3; i++)
	    (*dofArray)[(CORBA::ULong) i] = com [i];
	  return dofArray;
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
	  size_type deviceDim = robot->configSize ();
          assert(deviceDim == config.size());
          return vectorToFloatseq (config);
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

	  ObjectVector_t objects = body->innerObjects ();
	  if (objects.size() > 0) {
	    std::size_t nbObjects = objects.size();
	    // Allocate result now that the size is known.
	    ULong size = (ULong)nbObjects;
	    char** nameList = Names_t::allocbuf(size);
	    innerObjectSeq = new Names_t(size, size, nameList);
	    ObjectVector_t::const_iterator itObj = objects.begin ();
	    for (std::size_t iObject=0; iObject < nbObjects; iObject++) {
	      CollisionObjectConstPtr_t object = *itObj;
	      std::string geometryName = object->name();
	      nameList[iObject] =
		(char*)malloc(sizeof(char)*(geometryName.length()+1));
	      strcpy(nameList[iObject], geometryName.c_str());
	      ++itObj;
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

	  ObjectVector_t objects = body->outerObjects ();
	  if (objects.size() > 0) {
	    std::size_t nbObjects = objects.size();
	    // Allocate result now that the size is known.
	    ULong size = (ULong)nbObjects;
	    char** nameList = Names_t::allocbuf(size);
	    outerObjectSeq = new Names_t(size, size, nameList);
	    ObjectVector_t::const_iterator itObj = objects.begin ();
	    for (std::size_t iObject=0; iObject < nbObjects; iObject++) {
	      CollisionObjectConstPtr_t object = *itObj;
	      std::string geometryName = object->name();
	      nameList[iObject] =
		(char*)malloc(sizeof(char)*(geometryName.length()+1));
	      strcpy(nameList[iObject], geometryName.c_str());
	      ++itObj;
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
	using pinocchio::DeviceObjectVector;
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
          const DeviceObjectVector& objs = robot->objectVector();
	  for (DeviceObjectVector::const_iterator it = objs.begin();
	       it != objs.end(); ++it) {
	    if ((*it)->name () == name) {
	      return *it;
	    }
	  }
	  throw std::runtime_error ("robot has no object with name " +
				    std::string (name));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      JointPtr_t Robot::getJointByName (const char* name)
      {
	JointMap_t::const_iterator it = jointMap_.find (std::string (name));
	if (it != jointMap_.end ()) {
	  return it->second;
	}
	const DevicePtr_t robot (problemSolver()->robot ());
	if (robot) {
	  return robot->getJointByName (std::string (name));
	}
	std::string msg1 ("No robot is loaded and no joint with name ");
	std::string msg2 (" stored in local map");
	throw Error ((msg1 + std::string (name) + msg2).c_str ());
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
	  Configuration_t config = dofArrayToConfig (problemSolver(), dofArray);
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
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
            const se3::CollisionPair& cp (geomModel.collisionPairs[i]);
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
	  distances = vectorToFloatseq(dists);
	  innerObjects = toNames_t (innerObj.begin(), innerObj.end());
	  outerObjects = toNames_t (outerObj.begin(), outerObj.end());
	  innerPoints = matrixToFloatSeqSeq (innerPts);
	  outerPoints = matrixToFloatSeqSeq (outerPts);
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
	hpp::floatSeq* res = new hpp::floatSeq;
	try {
	  vector3_t com = getRobotOrThrow(problemSolver())->positionCenterOfMass ();
	  res->length (3);
	  (*res) [0] = com [0]; (*res) [1] = com [1]; (*res) [2] = com [2];
	  return res;
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

//DEPREC      // --------------------------------------------------------------------
//DEPREC
//DEPREC      void Robot::createPolyhedron(const char* inPolyhedronName)
//DEPREC	throw (hpp::Error)
//DEPREC      {
//DEPREC	std::string polyhedronName(inPolyhedronName);
//DEPREC
//DEPREC	// Check that polyhedron does not already exist.
//DEPREC	if (vertexMap_.find(polyhedronName) != vertexMap_.end ()) {
//DEPREC	  std::ostringstream oss ("polyhedron ");
//DEPREC	  oss << polyhedronName << " already exists.";
//DEPREC	  throw hpp::Error (oss.str ().c_str ());
//DEPREC	}
//DEPREC	vertexMap_ [polyhedronName] = std::vector <fcl::Vec3f> ();
//DEPREC	triangleMap_ [polyhedronName] = std::vector <fcl::Triangle> ();
//DEPREC      }
//DEPREC
//DEPREC
//DEPREC      // --------------------------------------------------------------------
//DEPREC
//DEPREC      void Robot::createBox(const char* name, Double x, Double y, Double z)
//DEPREC	throw (hpp::Error)
//DEPREC      {
//DEPREC	std::string shapeName(name);
//DEPREC	// Check that object does not already exist.
//DEPREC	if (vertexMap_.find(shapeName) != vertexMap_.end () ||
//DEPREC	    shapeMap_.find (shapeName) != shapeMap_.end ()) {
//DEPREC	  std::ostringstream oss ("object ");
//DEPREC	  oss << shapeName << " already exists.";
//DEPREC	  throw hpp::Error (oss.str ().c_str ());
//DEPREC	}
//DEPREC	BasicShapePtr_t box (new fcl::Box (x, y, z));
//DEPREC	shapeMap_ [shapeName] = box;
//DEPREC      }
//DEPREC
//DEPREC      // --------------------------------------------------------------------
//DEPREC
//DEPREC      void Robot::createSphere (const char* name, Double radius)
//DEPREC	throw (hpp::Error)
//DEPREC      {
//DEPREC	if (vertexMap_.find(name) != vertexMap_.end () ||
//DEPREC	    shapeMap_.find (name) != shapeMap_.end ()) {
//DEPREC	  std::ostringstream oss ("object ");
//DEPREC	  oss << name << " already exists.";
//DEPREC	  throw hpp::Error (oss.str ().c_str ());
//DEPREC	}
//DEPREC	BasicShapePtr_t sphere (new fcl::Sphere (radius));
//DEPREC	shapeMap_ [name] = sphere;
//DEPREC      }
//DEPREC
//DEPREC      // --------------------------------------------------------------------
//DEPREC
//DEPREC      Short Robot::addPoint(const char* polyhedronName, Double x, Double y,
//DEPREC			    Double z)
//DEPREC	throw (hpp::Error)
//DEPREC      {
//DEPREC	// Check that polyhedron exists.
//DEPREC	VertexMap_t::iterator itVertex = vertexMap_.find (polyhedronName);
//DEPREC	if (itVertex == vertexMap_.end ()) {
//DEPREC	  std::ostringstream oss ("polyhedron ");
//DEPREC	  oss << polyhedronName << " does not exist.";
//DEPREC	  throw hpp::Error (oss.str ().c_str ());
//DEPREC	}
//DEPREC	itVertex->second.push_back (fcl::Vec3f (x, y, z));
//DEPREC	return static_cast<Short> (vertexMap_.size ());
//DEPREC      }
//DEPREC
//DEPREC      // --------------------------------------------------------------------
//DEPREC
//DEPREC      Short Robot::addTriangle(const char* polyhedronName,
//DEPREC			       ULong pt1, ULong pt2, ULong pt3)
//DEPREC	throw (hpp::Error)
//DEPREC      {
//DEPREC	// Check that polyhedron exists.
//DEPREC	TriangleMap_t::iterator itTriangle = triangleMap_.find (polyhedronName);
//DEPREC	if (itTriangle == triangleMap_.end ()) {
//DEPREC	  std::ostringstream oss ("polyhedron ");
//DEPREC	  oss << polyhedronName << " does not exist.";
//DEPREC	  throw hpp::Error (oss.str ().c_str ());
//DEPREC	}
//DEPREC	itTriangle->second.push_back (fcl::Triangle (pt1, pt2, pt3));
//DEPREC	return static_cast<Short> (triangleMap_.size ());
//DEPREC      }
//DEPREC
//DEPREC      // --------------------------------------------------------------------
//DEPREC
//DEPREC      void Robot::addObjectToJoint (const char* jointName,
//DEPREC				     const char* objectName,
//DEPREC				     const Double* inConfig)
//DEPREC	throw (hpp::Error)
//DEPREC      {
//DEPREC	try {
//DEPREC	  JointPtr_t joint = getJointByName (jointName);
//DEPREC	  CollisionGeometryPtr_t geometry;
//DEPREC	  // Check that polyhedron exists.
//DEPREC	  VertexMap_t::const_iterator itVertex = vertexMap_.find(objectName);
//DEPREC	  if (itVertex != vertexMap_.end ()) {
//DEPREC	    PolyhedronPtr_t polyhedron = PolyhedronPtr_t (new Polyhedron_t);
//DEPREC	    int res = polyhedron->beginModel ();
//DEPREC	    if (res != fcl::BVH_OK) {
//DEPREC	      std::ostringstream oss ("fcl BVHReturnCode = ");
//DEPREC	      oss << res;
//DEPREC	      throw hpp::Error (oss.str ().c_str ());
//DEPREC	    }
//DEPREC	    polyhedron->addSubModel (itVertex->second,
//DEPREC				     triangleMap_ [objectName]);
//DEPREC	    polyhedron->endModel ();
//DEPREC	    geometry = polyhedron;
//DEPREC	  } else {
//DEPREC	    ShapeMap_t::const_iterator itShape = shapeMap_.find (objectName);
//DEPREC	    if (itShape != shapeMap_.end ()) {
//DEPREC	      geometry = itShape->second;
//DEPREC	    }
//DEPREC	  }
//DEPREC	  if (!geometry) {
//DEPREC	    std::ostringstream oss ("object ");
//DEPREC	    oss << objectName << " does not exist.";
//DEPREC	    throw hpp::Error (oss.str ().c_str ());
//DEPREC	  }
//DEPREC
//DEPREC	  Transform3f pos;
//DEPREC	  hppTransformToTransform3f (inConfig, pos);
//DEPREC	  CollisionObjectPtr_t collisionObject
//DEPREC	    (CollisionObject_t::create (geometry, pos, objectName));
//DEPREC	  joint->linkedBody ()->addInnerObject(collisionObject, true, true);
//DEPREC	} catch (const std::exception& exc) {
//DEPREC	  throw hpp::Error (exc.what ());
//DEPREC	}
//DEPREC      }

      void Robot::addPartialCom (const char* comName, const Names_t& jointNames)
        throw (hpp::Error)
      {
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
        pinocchio::CenterOfMassComputationPtr_t comc =
          pinocchio::CenterOfMassComputation::create (robot);

        for (CORBA::ULong i=0; i<jointNames.length (); ++i) {
          std::string name (jointNames[i]);
          JointPtr_t j = robot->getJointByName (name);
          if (!j) throw hpp::Error ("One joint not found.");
          comc->add (j);
        }
        problemSolver()->addCenterOfMassComputation (std::string (comName), comc);
      }
    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.
