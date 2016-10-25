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

#include <hpp/util/debug.hh>
#include <hpp/model/fwd.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/urdf/util.hh>
#include <hpp/model/object-factory.hh>
#include <hpp/model/object-iterator.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/center-of-mass-computation.hh>
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
	using hpp::corbaserver::jointBoundSeq;

	static void localSetJointBounds(const JointPtr_t& joint,
					const jointBoundSeq& jointBounds)
	{
	  std::size_t nbJointBounds = (std::size_t)jointBounds.length();
	  std::size_t jointNbDofs = joint->configSize ();
	  if (nbJointBounds == 2*jointNbDofs) {
	    for (std::size_t iDof=0; iDof<jointNbDofs; iDof++) {
	      double vMin = jointBounds[(CORBA::ULong) (2*iDof)];
	      double vMax = jointBounds[(CORBA::ULong) (2*iDof+1)];
	      if (vMin <= vMax) {
		/* Dof is actually bounded */
		joint->isBounded(iDof, true);
		joint->lowerBound(iDof, vMin);
		joint->upperBound(iDof, vMax);
	      }
	      else {
		/* Dof is not bounded */
		joint->isBounded(iDof, false);
	      }
	    }
	  } else {
	    std::ostringstream oss ("Expected list of ");
	    oss << 2*jointNbDofs << "float, got " << nbJointBounds << ".";
	    throw hpp::Error (oss.str ().c_str ());
	  }
	}

	static jointBoundSeq* localGetJointBounds(const JointPtr_t& joint)
	{
	  std::size_t jointNbDofs = joint->configSize ();
          jointBoundSeq* ret = new jointBoundSeq;
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

        DevicePtr_t getRobotOrThrow (core::ProblemSolverPtr_t p)
        {
          DevicePtr_t robot = p->robot ();
          if (!robot) throw hpp::Error ("No robot in problem solver.");
          return robot;
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
	// Check that no robot of this name already exists.
	if (robotMap_.count (robotName) != 0) {
	  std::ostringstream oss ("robot ");
	  oss << robotName << " already exists.";
	  hppDout (error, oss.str ());
	  throw hpp::Error (oss.str ().c_str ());
	}
	// Try to create a robot.
	DevicePtr_t robot = Device_t::create (robotName);
	// Store robot in map.
	robotMap_ [robotName] = robot;
      }

      // --------------------------------------------------------------------

      void Robot::setRobot(const char* inRobotName) throw (hpp::Error)
      {
	std::string robotName (inRobotName);
	// Check that robot of this name exists.
	if (robotMap_.count (robotName) != 1) {
	  std::ostringstream oss ("robot ");
	  oss << robotName << " does not exists.";
	  hppDout (error, oss.str ());
	  throw Error (oss.str ().c_str ());
	}
	DevicePtr_t robot = robotMap_ [robotName];
	// Create a new problem with this robot.
	problemSolver()->robot (robot);
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

      void Robot::setRobotRootJoint(const char* inRobotName,
				     const char* inJointName)
	throw (hpp::Error)
      {
	std::string robotName(inRobotName);

	// Check that robot of this name exists.
	if (robotMap_.count (robotName) != 1) {
	  std::ostringstream oss ("robot ");
	  oss << robotName << " does not exists.";
	  hppDout (error, oss.str ());
	  throw hpp::Error (oss.str ().c_str ());
	}
	DevicePtr_t robot = robotMap_ [robotName];
	JointPtr_t joint = getJointByName (inJointName);

	robot->rootJoint (joint);
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
	  model::DevicePtr_t device (model::Device::create (robotName));
	  hpp::model::urdf::loadRobotModel (device,
					    std::string (rootJointType),
					    std::string (packageName),
					    std::string (modelName),
					    std::string (urdfSuffix),
					    std::string (srdfSuffix));
	  // Add device to the planner
	  problemSolver()->robot (device);
	  problemSolver()->robot ()->controlComputation
	    (model::Device::JOINT_POSITION);
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
	  model::HumanoidRobotPtr_t robot
	    (model::HumanoidRobot::create (robotName));
	  hpp::model::urdf::loadHumanoidModel (robot,
					       std::string (rootJointType),
					       std::string (packageName),
					       std::string (modelName),
					       std::string (urdfSuffix),
					       std::string (srdfSuffix));
	  // Add robot to the planner
	  problemSolver()->robot (robot);
	  problemSolver()->robot ()->controlComputation
	    (model::Device::JOINT_POSITION);
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

      void Robot::createJoint (const char* jointName,
			        const char* jointType,
				const Double* pos,
				const jointBoundSeq& jointBounds)
	throw (hpp::Error)
      {
	std::string jn(jointName);
	std::string jt(jointType);

	// Check that joint of this name does not already exist.
	if (jointMap_.count(jn) != 0) {
	  std::ostringstream oss ("joint ");
	  oss << jn << " already exists.";
	  hppDout (error, oss.str ());
	  throw hpp::Error (oss.str ().c_str ());
	}
	JointPtr_t joint;
	// Fill position matrix
	Transform3f posMatrix;
	hppTransformToTransform3f (pos, posMatrix);

	// Determine type of joint.
	if (jt == "anchor") {
	  joint = objectFactory_.createJointAnchor (posMatrix);
	}
	else if (jt == "SO3") {
	  joint =
	    objectFactory_.createJointSO3 (posMatrix);
	}
	else if (jt == "bounded-rotation") {
	  joint = objectFactory_.createBoundedJointRotation (posMatrix);
	}
	else if (jt == "unbounded-rotation") {
	  joint = objectFactory_.createUnBoundedJointRotation (posMatrix);
	}
	else if (jt == "translation") {
	  joint =
	    objectFactory_.createJointTranslation (posMatrix);
	}
	else if (jt == "translation2") {
	  joint =
	    objectFactory_.createJointTranslation2 (posMatrix);
	}
	else if (jt == "translation3") {
	  joint =
	    objectFactory_.createJointTranslation3 (posMatrix);
	}
	else if (jt == "rotation") {
	  throw hpp::Error ("joint type \"rotation\" is not supported anymore,"
			    "please choose between \"bounded-rotation\" "
			    "and \"unbounded-rotation\"");
	}
	else {
	  std::ostringstream oss ("joint type");
	  oss << jt << " does not exist.";
	  hppDout (error, oss.str ());
	  throw hpp::Error (oss.str ().c_str ());
	}
	// Set the bounds of the joint
	// Bound joint if needed.
	localSetJointBounds(joint, jointBounds);
	BodyPtr_t body = objectFactory_.createBody ();
	body->name (jn);
	joint->setLinkedBody (body);
	// Store joint in jointMap_.
	jointMap_[jn] = joint;
	joint->name (jn);
      }

      // --------------------------------------------------------------------

      void Robot::addJoint(const char* inParentName,
			    const char* inChildName)
	throw (hpp::Error)
      {
	JointPtr_t parentJoint = getJointByName (inParentName);
	JointPtr_t childJoint = getJointByName (inChildName);
	parentJoint->addChildJoint (childJoint);
      }

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
	  for (std::size_t i = 0; i < jointVector.size (); ++i) {
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
	  JointVector_t jointVector = robot->getJointVector ();
	  ULong size = (CORBA::ULong) jointVector.size ();
	  char** nameList = Names_t::allocbuf(size);
	  Names_t *jointNames = new Names_t (size, size, nameList);
	  for (std::size_t i = 0; i < jointVector.size (); ++i) {
	    const JointPtr_t joint = jointVector [i];
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
      Transform__slice* Robot::getRootJointPosition () throw (hpp::Error)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  JointPtr_t root = robot->rootJoint ();
	  if (!root) {
	    throw hpp::Error ("robot has no root joint");
	  }
	  const Transform3f& T = root->positionInParentFrame ();
	  double* res = new Transform_;
	  res [0] = T.getTranslation () [0];
	  res [1] = T.getTranslation () [1];
	  res [2] = T.getTranslation () [2];
	  res [3] = T.getQuatRotation () [0];
	  res [4] = T.getQuatRotation () [1];
	  res [5] = T.getQuatRotation () [2];
	  res [6] = T.getQuatRotation () [3];
	  return res;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setRootJointPosition (const Double* position)
	throw (hpp::Error)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  Transform3f t3f;
	  hppTransformToTransform3f (position, t3f);
	  robot->rootJointPosition (t3f);
          robot->computeForwardKinematics ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setJointPositionInParentFrame (const char* jointName, const Double* position)
	throw (hpp::Error)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  JointVector_t jv = robot->getJointVector ();
	  JointPtr_t joint (NULL);
          std::string n (jointName);
          for (JointVector_t::iterator it = jv.begin ();
              it != jv.end (); ++it) {
            if (n.compare ((*it)->name ()) == 0) {
              joint = *it;
              break;
            }
          }
	  if (!joint) {
	    std::ostringstream oss ("Robot has no joint with name ");
	    oss  << n;
	    hppDout (error, oss.str ());
	    throw hpp::Error (oss.str ().c_str ());
	  }
	  Transform3f t3f;
	  hppTransformToTransform3f (position, t3f);
	  joint->positionInParentFrame (t3f);
          robot->computeForwardKinematics ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getJointConfig(const char* jointName)
	throw (hpp::Error)
      {
	hpp::floatSeq *dofArray;
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
	  dofArray = new hpp::floatSeq();
	  dofArray->length((CORBA::ULong) dim);
	  for(std::size_t i=0; i< (std::size_t) dim; ++i)
	    (*dofArray)[(CORBA::ULong) i] = config [ric + i];
	  return dofArray;
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
	  size_type dim = joint->numberDof ();
          if (dim != (CORBA::ULong) dq.length ()) {
	    throw Error ("Wrong speed dimension");
	  }
          vector_t config = robot->currentConfiguration ();
          vector_t dqAll (robot->numberDof ());
          dqAll.setZero ();
          size_type ric = joint->rankInConfiguration ();
          size_type riv = joint->rankInVelocity ();
	  for(size_type i=0; i<dim; ++i)
            dqAll [riv + i] = dq[(CORBA::ULong) i];
          joint->configuration ()->integrate (config, dqAll, ric, riv, config);
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
	  
	  // create eigen::matrix from fcl transformation 
	  const Transform3f& T = joint->currentTransformation ();
	  Eigen::Matrix< hpp::model::value_type, 3, 1 > translation;
	  hpp::model::vector3_t fclTrans (T.getTranslation ());
	  hpp::model::toEigen (fclTrans, translation);

	  Eigen::Matrix< hpp::model::value_type, 3, 3 > rotation;
          hpp::model::matrix3_t fclRot (T.getRotation ());
	  hpp::model::toEigen (fclRot, rotation);
	 
	  // join translation & rotation into homog. transformation 
	  Eigen::Matrix< hpp::model::value_type, 4, 4 > trafo;
	  trafo.block<3,3>(0,0) = rotation;
	  trafo.block<3,1>(0,3) = translation;
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
	  JointPtr_t joint = robot->getJointByName (jointName);
	  if (!joint) {
	    std::ostringstream oss ("Robot has no joint with name ");
	    oss  << jointName;
	    hppDout (error, oss.str ());
	    throw hpp::Error (oss.str ().c_str ());
	  }
	  const Transform3f& T = joint->currentTransformation ();
	  double* res = new Transform_;
	  res [0] = T.getTranslation () [0];
	  res [1] = T.getTranslation () [1];
	  res [2] = T.getTranslation () [2];
	  res [3] = T.getQuatRotation () [0];
	  res [4] = T.getQuatRotation () [1];
	  res [5] = T.getQuatRotation () [2];
	  res [6] = T.getQuatRotation () [3];
	  return res;
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
	  JointPtr_t joint = robot->getJointByName (jointName);
	  if (!joint) {
	    std::ostringstream oss ("Robot has no joint with name ");
	    oss  << jointName;
	    hppDout (error, oss.str ());
	    throw hpp::Error (oss.str ().c_str ());
	  }
	  const Transform3f& T = joint->positionInParentFrame ();
	  double* res = new Transform_;
	  res [0] = T.getTranslation () [0];
	  res [1] = T.getTranslation () [1];
	  res [2] = T.getTranslation () [2];
	  res [3] = T.getQuatRotation () [0];
	  res [4] = T.getQuatRotation () [1];
	  res [5] = T.getQuatRotation () [2];
	  res [6] = T.getQuatRotation () [3];
	  return res;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Short Robot::getJointNumberDof (const char* jointName) throw (hpp::Error)
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
	  return (Short)joint->numberDof ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Short Robot::getJointConfigSize (const char* jointName) throw (hpp::Error)
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
	  return (Short) joint->configSize ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      jointBoundSeq* Robot::getJointBounds (const char* jointName)
	throw (hpp::Error)
      {
	try  {
	  // Get robot in ProblemSolver object.
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());

	  // get joint
	  JointPtr_t joint = robot->getJointByName (jointName);
	  return localGetJointBounds(joint);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setJointBounds (const char* jointName,
				  const jointBoundSeq& jointBounds)
	throw (hpp::Error)
      {
	try  {
	  // Get robot in ProblemSolver object.
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());

	  // get joint
	  JointPtr_t joint = robot->getJointByName (jointName);
	  localSetJointBounds(joint, jointBounds);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Transform__slice* Robot::getLinkPosition (const char* jointName)
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
	  Transform3f T = joint->currentTransformation () *
	    joint->linkInJointFrame ();
	  double* res = new Transform_;
	  res [0] = T.getTranslation () [0];
	  res [1] = T.getTranslation () [1];
	  res [2] = T.getTranslation () [2];
	  res [3] = T.getQuatRotation () [0];
	  res [4] = T.getQuatRotation () [1];
	  res [5] = T.getQuatRotation () [2];
	  res [6] = T.getQuatRotation () [3];
	  return res;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      char* Robot::getLinkName (const char* jointName) throw (hpp::Error)
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
	  std::string linkName = joint->linkName ();
	  char* res = new char [linkName.size () + 1];
	  strcpy (res, linkName.c_str ());
	  return res;
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

      Short Robot::getDimensionExtraConfigSpace()
  throw (hpp::Error)
      {
  // Get robot in ProblemSolver object.
        Short dim;
        try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          dim = robot->extraConfigSpace().dimension();
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
        return dim;
      }

      // --------------------------------------------------------------------

      void Robot::setExtraConfigSpaceBounds
      (const hpp::corbaserver::jointBoundSeq& bounds) throw (hpp::Error)
      {
	using hpp::model::ExtraConfigSpace;
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

      static model::Configuration_t
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
	hpp::floatSeq *dofArray;
	try {
	  // Get robot in hppPlanner object.
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  vector_t config = robot->currentConfiguration ();
	  size_type deviceDim = robot->configSize ();
	  dofArray = new hpp::floatSeq();
	  dofArray->length((CORBA::ULong) deviceDim);
	  for(size_type i=0; i<deviceDim; i++)
	    (*dofArray)[(CORBA::ULong) i] = config [i];
	  return dofArray;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Names_t* Robot::getJointInnerObjects (const char* jointName)
	throw (hpp::Error)
      {
	Names_t *innerObjectSeq = 0x0;
	JointPtr_t joint (0x0);
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

	  ObjectVector_t objects = body->innerObjects (model::COLLISION);
	  if (objects.size() > 0) {
	    std::size_t nbObjects = objects.size();
	    // Allocate result now that the size is known.
	    ULong size = (ULong)nbObjects;
	    char** nameList = Names_t::allocbuf(size);
	    innerObjectSeq = new Names_t(size, size, nameList);
	    ObjectVector_t::const_iterator itObj = objects.begin ();
	    for (std::size_t iObject=0; iObject < nbObjects; iObject++) {
	      CollisionObjectPtr_t object = *itObj;
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
	JointPtr_t joint (0x0);

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

	  ObjectVector_t objects = body->outerObjects (model::COLLISION);
	  if (objects.size() > 0) {
	    std::size_t nbObjects = objects.size();
	    // Allocate result now that the size is known.
	    ULong size = (ULong)nbObjects;
	    char** nameList = Names_t::allocbuf(size);
	    outerObjectSeq = new Names_t(size, size, nameList);
	    ObjectVector_t::const_iterator itObj = objects.begin ();
	    for (std::size_t iObject=0; iObject < nbObjects; iObject++) {
	      CollisionObjectPtr_t object = *itObj;
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

      CollisionObjectPtr_t Robot::getObjectByName (const char* name)
      {
	using model::ObjectIterator;
	try {
	  DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  for (ObjectIterator it = robot->objectIterator (model::COLLISION);
	       !it.isEnd (); ++it) {
	    if ((*it)->name () == name) {
	      return *it;
	    }
	  }
	  for (ObjectIterator it = robot->objectIterator (model::DISTANCE);
	       !it.isEnd (); ++it) {
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

      void Robot::getObjectPosition (const char* objectName, Double* cfg)
	throw (hpp::Error)
      {
	try {
	  CollisionObjectPtr_t object = getObjectByName (objectName);
	  Transform3f transform = object->getTransform ();
	  Transform3fTohppTransform (transform, cfg);
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
	  distanceBetweenObjects->computeDistances ();
	  const DistanceResults_t& dr1 = robot->distanceResults ();
	  const DistanceResults_t& dr2 =
	    distanceBetweenObjects->distanceResults ();
	  std::size_t nbDistPairs = dr1.size () + dr2.size ();
	  hpp::floatSeq* distances_ptr = new hpp::floatSeq ();
	  distances_ptr->length ((CORBA::ULong) nbDistPairs);
	  Names_t* innerObjects_ptr = new Names_t ();
	  innerObjects_ptr->length ((CORBA::ULong) nbDistPairs);
	  Names_t* outerObjects_ptr = new Names_t ();
	  outerObjects_ptr->length ((CORBA::ULong) nbDistPairs);
	  hpp::floatSeqSeq* innerPoints_ptr = new hpp::floatSeqSeq ();
	  innerPoints_ptr->length ((CORBA::ULong) nbDistPairs);
	  hpp::floatSeqSeq* outerPoints_ptr = new hpp::floatSeqSeq ();
	  outerPoints_ptr->length ((CORBA::ULong) nbDistPairs);
	  std::size_t distPairId = 0;
	  for (DistanceResults_t::const_iterator itDistance = dr1.begin ();
	       itDistance != dr1.end (); itDistance++) {
	    (*distances_ptr) [(CORBA::ULong) distPairId] =
	      itDistance->fcl.min_distance;
	    (*innerObjects_ptr) [(CORBA::ULong) distPairId] =
	      itDistance->innerObject->name ().c_str ();
	    (*outerObjects_ptr) [(CORBA::ULong) distPairId] =
	      itDistance->outerObject->name ().c_str ();
	    hpp::floatSeq pointBody_seq;
	    pointBody_seq.length (3);
	    hpp::floatSeq pointObstacle_seq;
	    pointObstacle_seq.length (3);
	    for (std::size_t j=0; j<3; ++j) {
	      pointBody_seq [(CORBA::ULong) j] =
		itDistance->fcl.nearest_points [0][j];
	      pointObstacle_seq [(CORBA::ULong) j] =
		itDistance->fcl.nearest_points [1][j];
	    }
	    (*innerPoints_ptr) [(CORBA::ULong) distPairId] = pointBody_seq;
	    (*outerPoints_ptr) [(CORBA::ULong) distPairId] = pointObstacle_seq;
	    ++distPairId;
	  }
	  for (DistanceResults_t::const_iterator itDistance = dr2.begin ();
	       itDistance != dr2.end (); itDistance++) {
	    (*distances_ptr) [(CORBA::ULong) distPairId] =
	      itDistance->fcl.min_distance;
	    (*innerObjects_ptr) [(CORBA::ULong) distPairId] =
	      itDistance->innerObject->name ().c_str ();
	    (*outerObjects_ptr) [(CORBA::ULong) distPairId] =
	      itDistance->outerObject->name ().c_str ();
	    hpp::floatSeq pointBody_seq;
	    pointBody_seq.length (3);
	    hpp::floatSeq pointObstacle_seq;
	    pointObstacle_seq.length (3);
	    for (std::size_t j=0; j<3; ++j) {
	      pointBody_seq [(CORBA::ULong) j] =
		itDistance->fcl.nearest_points [0][j];
	      pointObstacle_seq [(CORBA::ULong) j] =
		itDistance->fcl.nearest_points [1][j];
	    }
	    (*innerPoints_ptr) [(CORBA::ULong) distPairId] = pointBody_seq;
	    (*outerPoints_ptr) [(CORBA::ULong) distPairId] = pointObstacle_seq;
	    ++distPairId;
	  }
	  distances = distances_ptr;
	  innerObjects = innerObjects_ptr;
	  outerObjects = outerObjects_ptr;
	  innerPoints = innerPoints_ptr;
	  outerPoints = outerPoints_ptr;
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

      // --------------------------------------------------------------------

      void Robot::createPolyhedron(const char* inPolyhedronName)
	throw (hpp::Error)
      {
	std::string polyhedronName(inPolyhedronName);

	// Check that polyhedron does not already exist.
	if (vertexMap_.find(polyhedronName) != vertexMap_.end ()) {
	  std::ostringstream oss ("polyhedron ");
	  oss << polyhedronName << " already exists.";
	  throw hpp::Error (oss.str ().c_str ());
	}
	vertexMap_ [polyhedronName] = std::vector <fcl::Vec3f> ();
	triangleMap_ [polyhedronName] = std::vector <fcl::Triangle> ();
      }


      // --------------------------------------------------------------------

      void Robot::createBox(const char* name, Double x, Double y, Double z)
	throw (hpp::Error)
      {
	std::string shapeName(name);
	// Check that object does not already exist.
	if (vertexMap_.find(shapeName) != vertexMap_.end () ||
	    shapeMap_.find (shapeName) != shapeMap_.end ()) {
	  std::ostringstream oss ("object ");
	  oss << shapeName << " already exists.";
	  throw hpp::Error (oss.str ().c_str ());
	}
	BasicShapePtr_t box (new fcl::Box (x, y, z));
	shapeMap_ [shapeName] = box;
      }

      // --------------------------------------------------------------------

      void Robot::createSphere (const char* name, Double radius)
	throw (hpp::Error)
      {
	if (vertexMap_.find(name) != vertexMap_.end () ||
	    shapeMap_.find (name) != shapeMap_.end ()) {
	  std::ostringstream oss ("object ");
	  oss << name << " already exists.";
	  throw hpp::Error (oss.str ().c_str ());
	}
	BasicShapePtr_t sphere (new fcl::Sphere (radius));
	shapeMap_ [name] = sphere;
      }

      // --------------------------------------------------------------------

      Short Robot::addPoint(const char* polyhedronName, Double x, Double y,
			    Double z)
	throw (hpp::Error)
      {
	// Check that polyhedron exists.
	VertexMap_t::iterator itVertex = vertexMap_.find (polyhedronName);
	if (itVertex == vertexMap_.end ()) {
	  std::ostringstream oss ("polyhedron ");
	  oss << polyhedronName << " does not exist.";
	  throw hpp::Error (oss.str ().c_str ());
	}
	itVertex->second.push_back (fcl::Vec3f (x, y, z));
	return static_cast<Short> (vertexMap_.size ());
      }

      // --------------------------------------------------------------------

      Short Robot::addTriangle(const char* polyhedronName,
			       ULong pt1, ULong pt2, ULong pt3)
	throw (hpp::Error)
      {
	// Check that polyhedron exists.
	TriangleMap_t::iterator itTriangle = triangleMap_.find (polyhedronName);
	if (itTriangle == triangleMap_.end ()) {
	  std::ostringstream oss ("polyhedron ");
	  oss << polyhedronName << " does not exist.";
	  throw hpp::Error (oss.str ().c_str ());
	}
	itTriangle->second.push_back (fcl::Triangle (pt1, pt2, pt3));
	return static_cast<Short> (triangleMap_.size ());
      }

      // --------------------------------------------------------------------

      void Robot::addObjectToJoint (const char* jointName,
				     const char* objectName,
				     const Double* inConfig)
	throw (hpp::Error)
      {
	try {
	  JointPtr_t joint = getJointByName (jointName);
	  CollisionGeometryPtr_t geometry;
	  // Check that polyhedron exists.
	  VertexMap_t::const_iterator itVertex = vertexMap_.find(objectName);
	  if (itVertex != vertexMap_.end ()) {
	    PolyhedronPtr_t polyhedron = PolyhedronPtr_t (new Polyhedron_t);
	    int res = polyhedron->beginModel ();
	    if (res != fcl::BVH_OK) {
	      std::ostringstream oss ("fcl BVHReturnCode = ");
	      oss << res;
	      throw hpp::Error (oss.str ().c_str ());
	    }
	    polyhedron->addSubModel (itVertex->second,
				     triangleMap_ [objectName]);
	    polyhedron->endModel ();
	    geometry = polyhedron;
	  } else {
	    ShapeMap_t::const_iterator itShape = shapeMap_.find (objectName);
	    if (itShape != shapeMap_.end ()) {
	      geometry = itShape->second;
	    }
	  }
	  if (!geometry) {
	    std::ostringstream oss ("object ");
	    oss << objectName << " does not exist.";
	    throw hpp::Error (oss.str ().c_str ());
	  }

	  Transform3f pos;
	  hppTransformToTransform3f (inConfig, pos);
	  CollisionObjectPtr_t collisionObject
	    (CollisionObject_t::create (geometry, pos, objectName));
	  joint->linkedBody ()->addInnerObject(collisionObject, true, true);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Robot::addPartialCom (const char* comName, const Names_t& jointNames)
        throw (hpp::Error)
      {
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
        model::CenterOfMassComputationPtr_t comc =
          model::CenterOfMassComputation::create (robot);

        for (CORBA::ULong i=0; i<jointNames.length (); ++i) {
          std::string name (jointNames[i]);
          JointPtr_t j = robot->getJointByName (name);
          if (!j) throw hpp::Error ("One joint not found.");
          comc->add (j);
        }
        comc->computeMass ();
        problemSolver()->addCenterOfMassComputation (std::string (comName), comc);
      }
    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.
