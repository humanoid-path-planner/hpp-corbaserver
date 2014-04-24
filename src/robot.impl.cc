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
	static void localSetJointBounds(const JointPtr_t& joint,
					const hpp::jointBoundSeq& jointBounds)
	{
	  std::size_t nbJointBounds = (std::size_t)jointBounds.length();
	  std::size_t kwsJointNbDofs = joint->configSize ();
	  if (nbJointBounds == 2*kwsJointNbDofs) {
	    for (std::size_t iDof=0; iDof<kwsJointNbDofs; iDof++) {
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
	  }
	}
      } // end of namespace.

      // --------------------------------------------------------------------

      Robot::Robot(corbaServer::Server* server) :
	server_(server), problemSolver_(server->problemSolver ())
      {
      }

      // --------------------------------------------------------------------

      Short Robot::createRobot(const char* inRobotName)
	throw (SystemException)
      {
	std::string robotName (inRobotName);
	// Check that no robot of this name already exists.
	if (robotMap_.count (robotName) != 0) {
	  hppDout (error, ":createRobot: robot " << robotName <<
		   " already exists.");
	  return -1;
	}
	// Try to create a robot.
	DevicePtr_t robot = Device_t::create (robotName);
	// Store robot in map.
	robotMap_ [robotName] = robot;
	return 0;
      }

      // --------------------------------------------------------------------

      Short Robot::setRobot(const char* inRobotName) throw (SystemException)
      {
	std::string robotName (inRobotName);
	// Check that robot of this name exists.
	if (robotMap_.count (robotName) != 1) {
	  hppDout (error, ":setRobot: robot " << robotName <<
		   " does not exist.");
	  return -1;
	}
	DevicePtr_t robot = robotMap_ [robotName];
	// Create a new problem with this robot.
	problemSolver_->robot (robot);
	return 0;
      }

      // --------------------------------------------------------------------

      Short Robot::setRobotRootJoint(const char* inRobotName,
				     const char* inJointName)
	throw (SystemException)
      {
	std::string robotName(inRobotName);
	std::string jointName(inJointName);

	// Check that robot of this name exists.
	if (robotMap_.count (robotName) != 1) {
	  hppDout (error, ":setRobotRootJoint: robot " << robotName <<
		   " does not exist.");
	  return -1;
	}
	// Check that joint of this name exists.
	if (jointMap_.count (jointName) != 1) {
	  hppDout (error, ":setRobotRootJoint: joint " << jointName <<
		   " does not exist.");
	  return -1;
	}
	DevicePtr_t robot = robotMap_ [robotName];
	JointPtr_t joint = jointMap_ [jointName];

	robot->rootJoint (joint);
	return 0;
      }

      // --------------------------------------------------------------------

      Short Robot::loadRobotModel(const char* robotName,
				  const char* rootJointType,
				  const char* packageName,
				  const char* modelName,
				  const char* urdfSuffix,
				  const char* srdfSuffix)
      {
	try {
	  hpp::model::DevicePtr_t device = 
	    hpp::model::urdf::loadRobotModel (std::string (robotName),
					      std::string (rootJointType),
					      std::string (packageName),
					      std::string (modelName),
					      std::string (urdfSuffix),
					      std::string (srdfSuffix));
	  // Add device to the planner
	  problemSolver_->robot (device);
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
	return 0;
      }

      // --------------------------------------------------------------------

      Short Robot::loadHumanoidModel(const char* robotName,
				     const char* rootJointType,
				     const char* packageName,
				     const char* modelName,
				     const char* urdfSuffix,
				     const char* srdfSuffix)
      {
	try {
	  hpp::model::HumanoidRobotPtr_t device = 
	    hpp::model::urdf::loadHumanoidModel (std::string (robotName),
						 std::string (rootJointType),
						 std::string (packageName),
						 std::string (modelName),
						 std::string (urdfSuffix),
						 std::string (srdfSuffix));
	  // Add device to the planner
	  problemSolver_->robot (device);
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
	return 0;
      }

      // --------------------------------------------------------------------

      Short Robot::getConfigSize () throw (SystemException)
      {
	try {
	  return problemSolver_->robot ()->configSize ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
      }

      // --------------------------------------------------------------------

      Short Robot::getNumberDof () throw (SystemException)
      {
	try {
	  return problemSolver_->robot ()->numberDof ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
      }

      // --------------------------------------------------------------------

      Short Robot::createJoint (const char* jointName,
			        const char* jointType,
				const hpp::Configuration& pos,
				const hpp::jointBoundSeq& jointBounds)
	throw (SystemException)
      {
	std::string jn(jointName);
	std::string jt(jointType);

	// Check that joint of this name does not already exist.
	if (jointMap_.count(jn) != 0) {
	  hppDout (error, ":createJoint: joint " << jn  <<
		   " already exists.");
	  return -1;
	}
	JointPtr_t joint;
	// Fill position matrix
	Transform3f posMatrix;
	ConfigurationToTransform3f (pos, posMatrix);

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
	  hppDout (error, ":createJoint: joint type " << jt
		   << " does not exist.");
	  return -1;
	}
	// Check whether creation failed.
	if (!joint) {
	  hppDout (error, ":createJoint: failed to create joint " << jn);
	  return -1;
	}
	// Set the bounds of the joint
	// Bound joint if needed.
	localSetJointBounds(joint, jointBounds);
	BodyPtr_t body = objectFactory_.createBody ();
	joint->setLinkedBody (body);
	// Store joint in jointMap_.
	jointMap_[jn] = joint;
	joint->name (jn);
	return 0;
      }

      // --------------------------------------------------------------------

      Short Robot::addJoint(const char* inParentName,
			    const char* inChildName)
	throw (SystemException)
      {
	// Check that joint of this name exists.
	if (jointMap_.count(inParentName) != 1) {
	  hppDout (error, ":addJoint: joint " << inParentName  <<
		   " does not exist.");
	  return -1;
	}
	// Check that joint of this name does not already exist.
	if (jointMap_.count(inChildName) != 1) {
	  hppDout (error, ":addJoint: joint " << inChildName  <<
		   " does not exist.");
	  return -1;
	}
	JointPtr_t parentJoint = jointMap_ [inParentName];
	JointPtr_t childJoint = jointMap_ [inChildName];
	parentJoint->addChildJoint (childJoint);
	return 0;
      }

      // --------------------------------------------------------------------

      hpp::nameSeq* Robot::getJointNames ()
      {
	DevicePtr_t robot = problemSolver_->robot ();
	// Compute number of real urdf joints
	ULong size = 0;
	JointVector_t jointVector = robot->getJointVector ();
	for (JointVector_t::const_iterator it = jointVector.begin ();
	     it != jointVector.end (); it++) {
	  if ((*it)->numberDof () != 0) size ++;
	}
	char** nameList = hpp::nameSeq::allocbuf(size);
	hpp::nameSeq *jointNames = new hpp::nameSeq (size, size, nameList);
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
      }

      // --------------------------------------------------------------------

      Short Robot::getJointNumberDof (const char* jointName)
      {
	DevicePtr_t robot = problemSolver_->robot ();
	if (!robot) {
	  hppDout (error, "No robot");
	  return -1;
	}
	JointPtr_t joint = robot->getJointByName (jointName);
	if (!joint) {
	  hppDout (error, "Robot has no joint with name " << jointName);
	  return -1;
	}
	return joint->numberDof ();
      }
      
      // --------------------------------------------------------------------

      Short Robot::getJointConfigSize (const char* jointName)
      {
	DevicePtr_t robot = problemSolver_->robot ();
	if (!robot) {
	  hppDout (error, "No robot");
	  return -1;
	}
	JointPtr_t joint = robot->getJointByName (jointName);
	if (!joint) {
	  hppDout (error, "Robot has no joint with name " << jointName);
	  return -1;
	}
	return joint->configSize ();
      }

      // --------------------------------------------------------------------

      Short Robot::setJointBounds (UShort jointId,
				   const hpp::jointBoundSeq& jointBounds)
	throw (SystemException)
      {
	try  {
	  // Get robot in hppPlanner object.
	  DevicePtr_t robot = problemSolver_->robot ();

	  // get joint
	  JointVector_t jointVector = robot->getJointVector ();
	  JointPtr_t kwsJoint = jointVector[jointId];
	  localSetJointBounds(kwsJoint, jointBounds);
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
	return 0;
      }

      // --------------------------------------------------------------------

      Short Robot::setCurrentConfig(const hpp::floatSeq& dofArray)
	throw (SystemException)
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
	    hppDout (error, ":setCurrentConfig: dofVector Does not match");
	    return -1;
	  }
	  // Create a config for robot initialized with dof vector.
	  problemSolver_->robot ()->currentConfiguration (config);
	  problemSolver_->robot ()->computeForwardKinematics ();
	  return 0;
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
	return 0;
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getCurrentConfig() throw (SystemException)
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
	  hppDout (error, exc.what ());
	  dofArray = new hpp::floatSeq (1);
	  return dofArray;
	}
	return new hpp::floatSeq (1);
      }
      // --------------------------------------------------------------------

      hpp::nameSeq* Robot::getJointInnerObjects (const char* jointName)
      {
	hpp::nameSeq *innerObjectSeq = 0x0;
	JointPtr_t joint (0x0);
	try {
	  DevicePtr_t robot = problemSolver_->robot ();
	  joint = robot->getJointByName (std::string (jointName));
	  if (!joint) {
	    hppDout (error, "No joint with name " << jointName);
	    innerObjectSeq = new hpp::nameSeq (0);
	    return innerObjectSeq;
	  }
	  BodyPtr_t body = joint->linkedBody ();
	  if (!body) {
	    hppDout (error, "Joint " << jointName << " has no body.");
	    innerObjectSeq = new hpp::nameSeq (0);
	    return innerObjectSeq;
	  }

	  ObjectVector_t objects = body->innerObjects (model::COLLISION);
	  if (objects.size() > 0) {
	    std::size_t nbObjects = objects.size();
	    // Allocate result now that the size is known.
	    ULong size = (ULong)nbObjects;
	    char** nameList = hpp::nameSeq::allocbuf(size);
	    innerObjectSeq = new hpp::nameSeq(size, size, nameList);
	    for (std::size_t iObject=0; iObject < nbObjects; iObject++) {
	      CollisionObjectPtr_t object = objects[iObject];
	      std::string geometryName = object->name();
	      nameList[iObject] =
		(char*)malloc(sizeof(char)*(geometryName.length()+1));
	      strcpy(nameList[iObject], geometryName.c_str());
	    }
	  } else {
	    innerObjectSeq = new hpp::nameSeq (0);
	  }
	  return innerObjectSeq;
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  innerObjectSeq = new hpp::nameSeq (0);
	  return innerObjectSeq;
	}
      }

      // --------------------------------------------------------------------

      hpp::nameSeq* Robot::getJointOuterObjects (const char* jointName)
      {
	hpp::nameSeq *outerObjectSeq = 0x0;
	JointPtr_t joint (0x0);

	try {
	  DevicePtr_t robot = problemSolver_->robot ();
	  joint = robot->getJointByName (std::string (jointName));
	  if (!joint) {
	    hppDout (error, "No joint with name " << jointName);
	    outerObjectSeq = new hpp::nameSeq (0);
	    return outerObjectSeq;
	  }
	  BodyPtr_t body = joint->linkedBody ();
	  if (!body) {
	    hppDout (error, "Joint " << jointName << " has no body.");
	    outerObjectSeq = new hpp::nameSeq (0);
	    return outerObjectSeq;
	  }

	  ObjectVector_t objects = body->outerObjects (model::COLLISION);
	  if (objects.size() > 0) {
	    std::size_t nbObjects = objects.size();
	    // Allocate result now that the size is known.
	    ULong size = (ULong)nbObjects;
	    char** nameList = hpp::nameSeq::allocbuf(size);
	    outerObjectSeq = new hpp::nameSeq(size, size, nameList);
	    for (std::size_t iObject=0; iObject < nbObjects; iObject++) {
	      CollisionObjectPtr_t object = objects[iObject];
	      std::string geometryName = object->name();
	      nameList[iObject] =
		(char*)malloc(sizeof(char)*(geometryName.length()+1));
	      strcpy(nameList[iObject], geometryName.c_str());
	    }
	  } else {
	    outerObjectSeq = new hpp::nameSeq (0);
	  }
	  return outerObjectSeq;
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  outerObjectSeq = new hpp::nameSeq (0);
	  return outerObjectSeq;
	}
      }

      // --------------------------------------------------------------------

      Short Robot::collisionTest (Boolean& validity) throw (SystemException)
      {
	validity = false;
	try {
	  DevicePtr_t robot = problemSolver_->robot ();
	  validity = robot->collisionTest ();
	  return 0;
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	}
	return -1;
      }

      // --------------------------------------------------------------------

      Short
      Robot::distancesToCollision (hpp::floatSeq_out distances,
				   hpp::nameSeq_out innerObjects,
				   hpp::nameSeq_out outerObjects,
				   hpp::floatSeqSeq_out innerPoints,
				   hpp::floatSeqSeq_out outerPoints)
      {
	try {
	  DevicePtr_t robot = problemSolver_->robot ();
	  robot->computeDistances ();
	  const DistanceResults_t& dr = robot->distanceResults ();
	  std::size_t nbDistPairs = dr.size ();
	  hpp::floatSeq* distances_ptr = new hpp::floatSeq ();
	  distances_ptr->length (nbDistPairs);
	  hpp::nameSeq* innerObjects_ptr = new hpp::nameSeq ();
	  innerObjects_ptr->length (nbDistPairs);
	  hpp::nameSeq* outerObjects_ptr = new hpp::nameSeq ();
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
	  return 0;
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  distances = new hpp::floatSeq ();
	  innerObjects = new hpp::nameSeq ();
	  outerObjects = new hpp::nameSeq ();
	  innerPoints = new hpp::floatSeqSeq ();
	  outerPoints = new hpp::floatSeqSeq ();
	  return -1;
	}
	return 0;
      }

      // --------------------------------------------------------------------

      Double Robot::getMass ()
      {
	try {
	  return problemSolver_->robot ()->mass ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* Robot::getCenterOfMass ()
      {
	hpp::floatSeq* res = new hpp::floatSeq;
	try {
	  vector3_t com = problemSolver_->robot ()->positionCenterOfMass ();
	  res->length (3);
	  (*res) [0] = com [0]; (*res) [1] = com [1]; (*res) [2] = com [2];
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  res->length (0);
	}
	return res;
      }

      // --------------------------------------------------------------------

      hpp::floatSeqSeq* Robot::getJacobianCenterOfMass ()
      {
	hpp::floatSeqSeq* res = new hpp::floatSeqSeq;
	try {
	  const ComJacobian_t& jacobian =
	    problemSolver_->robot ()->jacobianCenterOfMass ();
	  res->length (jacobian.rows ());
	  for (std::size_t i=0; i<jacobian.rows (); ++i) {
	    hpp::floatSeq row; row.length (jacobian.cols ());
	    for (std::size_t j=0; j<jacobian.cols (); ++j) {
	      row [j] = jacobian (i, j);
	    }
	    (*res) [i] = row;
	  }
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  res->length (0);
	}
	return res;
      }

      // --------------------------------------------------------------------

      Short Robot::createPolyhedron(const char* inPolyhedronName)
	throw (SystemException)
      {
	std::string polyhedronName(inPolyhedronName);

	// Check that polyhedron does not already exist.
	if (vertexMap_.find(polyhedronName) != vertexMap_.end ()) {
	  hppDout (error, "polyhedron "	 << polyhedronName
		   << " already exists.");
	  return -1;
	}
	vertexMap_ [polyhedronName] = std::vector <fcl::Vec3f> ();
	triangleMap_ [polyhedronName] = std::vector <fcl::Triangle> ();
	return 0;
      }


      // --------------------------------------------------------------------

      Short Robot::createBox(const char* name,
			     Double x, Double y,
			     Double z)
	throw (SystemException)
      {
	std::string shapeName(name);
	// Check that object does not already exist.
	if (vertexMap_.find(shapeName) != vertexMap_.end () ||
	    shapeMap_.find (shapeName) != shapeMap_.end ()) {
	  hppDout (info, "object " << shapeName << " already exists.");
	  return -1;
	}
	BasicShapePtr_t box (new fcl::Box (x, y, z));
	shapeMap_ [shapeName] = box;
	return 0;
      }

      // --------------------------------------------------------------------

      Short Robot::createSphere (const char* name, Double radius)
	throw (SystemException)
      {
	if (vertexMap_.find(name) != vertexMap_.end () ||
	    shapeMap_.find (name) != shapeMap_.end ()) {
	  hppDout (info, "object " << name << " already exists.");
	  return -1;
	}
	BasicShapePtr_t sphere (new fcl::Sphere (radius));
	shapeMap_ [name] = sphere;
	return 0;

      }

      // --------------------------------------------------------------------

      Short Robot::addPoint(const char* polyhedronName, Double x, Double y,
			    Double z)
	throw (SystemException)
      {
	// Check that polyhedron exists.
	VertexMap_t::iterator itVertex = vertexMap_.find (polyhedronName);
	if (itVertex == vertexMap_.end ()) {
	  hppDout (error, "polyhedron " << polyhedronName
		   << " does not exist.");
	  return -1;
	}
	itVertex->second.push_back (fcl::Vec3f (x, y, z));
	return static_cast<Short> (vertexMap_.size ());
      }

      // --------------------------------------------------------------------

      Short Robot::addTriangle(const char* polyhedronName,
			       ULong pt1, ULong pt2, ULong pt3)
	throw (SystemException)
      {
	// Check that polyhedron exists.
	TriangleMap_t::iterator itTriangle = triangleMap_.find (polyhedronName);
	if (itTriangle == triangleMap_.end ()) {
	  hppDout (error, "polyhedron " << polyhedronName
		   << " does not exist.");
	  return -1;
	}

	itTriangle->second.push_back (fcl::Triangle (pt1, pt2, pt3));
	return static_cast<Short> (triangleMap_.size ());
      }

      // --------------------------------------------------------------------

      Short Robot::addObjectToJoint (const char* jointName,
				     const char* objectName,
				     const hpp::Configuration& inConfig)
	throw (SystemException)
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
	      hppDout (error, "fcl BVHReturnCode = " << res);
	      return -1;
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
	    hppDout (error, "Object " << objectName << " does not exist.");
	    return -1;
	  }

	  Transform3f pos;
	  ConfigurationToTransform3f (inConfig, pos);
	  CollisionObjectPtr_t collisionObject
	    (CollisionObject_t::create (geometry, pos, objectName));
	  joint->linkedBody ()->addInnerObject(collisionObject, true, true);
	  return 0;
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  return -1;
	}
      }

    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.
