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
#include <hpp/core/basic-configuration-shooter.hh>
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
	      double vMin = jointBounds[2*iDof];
	      double vMax = jointBounds[2*iDof+1];
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
      } // end of namespace.

      // --------------------------------------------------------------------

      Robot::Robot(corbaServer::Server* server) :
	server_(server), problemSolver_(server->problemSolver ())
      {
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
	problemSolver_->robot (robot);
      }

      // --------------------------------------------------------------------

      void Robot::setRobotRootJoint(const char* inRobotName,
				     const char* inJointName)
	throw (hpp::Error)
      {
	std::string robotName(inRobotName);
	std::string jointName(inJointName);

	// Check that robot of this name exists.
	if (robotMap_.count (robotName) != 1) {
	  std::ostringstream oss ("robot ");
	  oss << robotName << " does not exists.";
	  hppDout (error, oss.str ());
	  throw hpp::Error (oss.str ().c_str ());
	}
	// Check that joint of this name exists.
	if (jointMap_.count (jointName) != 1) {
	  std::ostringstream oss ("joint ");
	  oss << jointName << " does not exists.";
	  hppDout (error, oss.str ());
	  throw hpp::Error (oss.str ().c_str ());
	}
	DevicePtr_t robot = robotMap_ [robotName];
	JointPtr_t joint = jointMap_ [jointName];

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
	  problemSolver_->robot (device);
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
	  problemSolver_->robot (robot);
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Short Robot::getConfigSize () throw (hpp::Error)
      {
	try {
	  return (Short) problemSolver_->robot ()->configSize ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Short Robot::getNumberDof () throw (hpp::Error)
      {
	try {
	  return (Short) problemSolver_->robot ()->numberDof ();
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
	else if (jt == "rotation") {
	  joint = objectFactory_.createJointRotation (posMatrix);
	}
	else if (jt == "translation") {
	  joint =
	    objectFactory_.createJointTranslation (posMatrix);
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
	// Check that joint of this name exists.
	if (jointMap_.count(inParentName) != 1) {
	  std::ostringstream oss ("joint ");
	  oss << inParentName << " does not exist.";
	  throw hpp::Error (oss.str ().c_str ());
	}
	// Check that joint of this name does not already exist.
	if (jointMap_.count(inChildName) != 1) {
	  std::ostringstream oss ("joint ");
	  oss << inChildName << " does not exist.";
	  throw hpp::Error (oss.str ().c_str ());
	}
	JointPtr_t parentJoint = jointMap_ [inParentName];
	JointPtr_t childJoint = jointMap_ [inChildName];
	parentJoint->addChildJoint (childJoint);
      }

      // --------------------------------------------------------------------

      Names_t* Robot::getJointNames () throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = problemSolver_->robot ();
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

      void Robot::setRootJointPosition (const Double* position)
	throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = problemSolver_->robot ();
	  if (!robot) {
	    throw hpp::Error ("no robot");
	  }
	  Transform3f t3f;
	  hppTransformToTransform3f (position, t3f);
	  robot->rootJointPosition (t3f);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}

      }

      // --------------------------------------------------------------------

      Transform_slice* Robot::getJointPosition(const char* jointName)
	throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = problemSolver_->robot ();
	  if (!robot) {
	    throw hpp::Error ("no robot");
	  }
	  JointPtr_t joint = robot->getJointByName (jointName);
	  if (!joint) {
	    std::ostringstream oss ("Robot has no joint with name ");
	    oss  << jointName;
	    hppDout (error, oss.str ());
	    throw hpp::Error (oss.str ().c_str ());
	  }
	  const Transform3f& T = joint->currentTransformation ();
	  double* res = new Transform;
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
	  DevicePtr_t robot = problemSolver_->robot ();
	  if (!robot) {
	    throw hpp::Error ("no robot");
	  }
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
	  DevicePtr_t robot = problemSolver_->robot ();
	  if (!robot) {
	    throw hpp::Error ("no robot");
	  }
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

      void Robot::setJointBounds (const char* jointName,
				  const jointBoundSeq& jointBounds)
	throw (hpp::Error)
      {
	try  {
	  // Get robot in ProblemSolver object.
	  DevicePtr_t robot = problemSolver_->robot ();

	  // get joint
	  JointPtr_t joint = robot->getJointByName (jointName);
	  localSetJointBounds(joint, jointBounds);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setDimensionExtraConfigSpace (ULong dimension)
	throw (hpp::Error)
      {
	// Get robot in ProblemSolver object.
	DevicePtr_t robot = problemSolver_->robot ();
	if (!robot) throw hpp::Error ("Robot is not set");
	try {
	  robot->setDimensionExtraConfigSpace (dimension);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void Robot::setExtraConfigSpaceBounds
      (const hpp::corbaserver::jointBoundSeq& bounds) throw (hpp::Error)
      {
	using hpp::model::ExtraConfigSpace;
	// Get robot in ProblemSolver object.
	DevicePtr_t robot = problemSolver_->robot ();
	if (!robot) throw hpp::Error ("Robot is not set");
	try {
	  ExtraConfigSpace& extraCS = robot->extraConfigSpace ();
	  std::size_t nbBounds = (std::size_t)bounds.length();
	  std::size_t dimension = extraCS.dimension ();
	  if (nbBounds == 2*dimension) {
	    for (std::size_t iDof=0; iDof < dimension; ++iDof) {
	      double vMin = bounds [2*iDof];
	      double vMax = bounds [2*iDof+1];
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
	std::size_t configDim = (std::size_t)dofArray.length();
	std::vector<double> dofVector;
	try {
	  // Get robot in hppPlanner object.
	  DevicePtr_t robot = problemSolver_->robot ();
	  // by Yoshida 06/08/25
	  std::size_t deviceDim = robot->configSize ();
	  // Fill dof vector with dof array.
	  vector_t config; config.resize (configDim);
	  for (std::size_t iDof = 0; iDof < configDim; iDof++) {
	    config [iDof] = dofArray[iDof];
	  }
	  // fill the vector by zero
	  hppDout (info, "config dimension: " <<configDim
		   <<",  deviceDim "<<deviceDim);
	  if(configDim != deviceDim){
	    throw hpp::Error ("dofVector Does not match");
	  }
	  // Create a config for robot initialized with dof vector.
	  problemSolver_->robot ()->currentConfiguration (config);
	  problemSolver_->robot ()->computeForwardKinematics ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::shootRandomConfig () throw (hpp::Error)
      {
	try {
	  hpp::floatSeq *dofArray = 0x0;
	  DevicePtr_t robot = problemSolver_->robot ();
	  hpp::core::BasicConfigurationShooter shooter (robot);
	  ConfigurationPtr_t configuration = shooter.shoot();

	  std::size_t deviceDim = robot->configSize ();
	  dofArray = new hpp::floatSeq();
	  dofArray->length (deviceDim);
	  for(std::size_t i=0; i<deviceDim; i++)
	    (*dofArray)[i] = (*configuration) [i];
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
	  DevicePtr_t robot = problemSolver_->robot ();
	  vector_t config = robot->currentConfiguration ();
	  std::size_t deviceDim = robot->configSize ();
	  dofArray = new hpp::floatSeq();
	  dofArray->length(deviceDim);
	  for(std::size_t i=0; i<deviceDim; i++)
	    (*dofArray)[i] = config [i];
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
	  DevicePtr_t robot = problemSolver_->robot ();
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
	    for (std::size_t iObject=0; iObject < nbObjects; iObject++) {
	      CollisionObjectPtr_t object = objects[iObject];
	      std::string geometryName = object->name();
	      nameList[iObject] =
		(char*)malloc(sizeof(char)*(geometryName.length()+1));
	      strcpy(nameList[iObject], geometryName.c_str());
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
	  DevicePtr_t robot = problemSolver_->robot ();
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
	    for (std::size_t iObject=0; iObject < nbObjects; iObject++) {
	      CollisionObjectPtr_t object = objects[iObject];
	      std::string geometryName = object->name();
	      nameList[iObject] =
		(char*)malloc(sizeof(char)*(geometryName.length()+1));
	      strcpy(nameList[iObject], geometryName.c_str());
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

      void Robot::collisionTest (Boolean& validity) throw (hpp::Error)
      {
	validity = false;
	try {
	  DevicePtr_t robot = problemSolver_->robot ();
	  validity = robot->collisionTest ();
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
	  DevicePtr_t robot = problemSolver_->robot ();
	  robot->computeDistances ();
	  const DistanceResults_t& dr = robot->distanceResults ();
	  std::size_t nbDistPairs = dr.size ();
	  hpp::floatSeq* distances_ptr = new hpp::floatSeq ();
	  distances_ptr->length (nbDistPairs);
	  Names_t* innerObjects_ptr = new Names_t ();
	  innerObjects_ptr->length (nbDistPairs);
	  Names_t* outerObjects_ptr = new Names_t ();
	  outerObjects_ptr->length (nbDistPairs);
	  hpp::floatSeqSeq* innerPoints_ptr = new hpp::floatSeqSeq ();
	  innerPoints_ptr->length (nbDistPairs);
	  hpp::floatSeqSeq* outerPoints_ptr = new hpp::floatSeqSeq ();
	  outerPoints_ptr->length (nbDistPairs);
	  std::size_t distPairId = 0;
	  for (DistanceResults_t::const_iterator itDistance = dr.begin ();
	       itDistance != dr.end (); itDistance++) {
	    (*distances_ptr) [distPairId] = itDistance->fcl.min_distance;
	    (*innerObjects_ptr) [distPairId] =
	      itDistance->innerObject->name ().c_str ();
	    (*outerObjects_ptr) [distPairId] =
	      itDistance->outerObject->name ().c_str ();
	    hpp::floatSeq pointBody_seq;
	    pointBody_seq.length (3);
	    hpp::floatSeq pointObstacle_seq;
	    pointObstacle_seq.length (3);
	    for (std::size_t j=0; j<3; ++j) {
	      pointBody_seq [j] = itDistance->fcl.nearest_points [0][j];
	      pointObstacle_seq [j] = itDistance->fcl.nearest_points [1][j];
	    }
	    (*innerPoints_ptr) [distPairId] = pointBody_seq;
	    (*outerPoints_ptr) [distPairId] = pointObstacle_seq;
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
	  return problemSolver_->robot ()->mass ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getCenterOfMass () throw (hpp::Error)
      {
	hpp::floatSeq* res = new hpp::floatSeq;
	try {
	  vector3_t com = problemSolver_->robot ()->positionCenterOfMass ();
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
	hpp::floatSeqSeq* res = new hpp::floatSeqSeq;
	try {
	  const ComJacobian_t& jacobian =
	    problemSolver_->robot ()->jacobianCenterOfMass ();
	  res->length (jacobian.rows ());
	  for (size_type i=0; i<jacobian.rows (); ++i) {
	    hpp::floatSeq row; row.length (jacobian.cols ());
	    for (size_type j=0; j<jacobian.cols (); ++j) {
	      row [j] = jacobian (i, j);
	    }
	    (*res) [i] = row;
	  }
	  return res;
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
	  JointPtr_t joint = jointMap_ [jointName];
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

    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.
