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
#include "../../hpp-core/src/basic-configuration-shooter.hh"

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
	  return problemSolver_->robot ()->configSize ();
	} catch (const std::exception& exc) {
	  hppDout (error, exc.what ());
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      Short Robot::getNumberDof () throw (hpp::Error)
      {
	try {
	  return problemSolver_->robot ()->numberDof ();
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
	  return joint->numberDof ();
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
	  return joint->configSize ();
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
	  // Get robot in hppPlanner object.
	  DevicePtr_t robot = problemSolver_->robot ();

	  // get joint
	  JointPtr_t joint = robot->getJointByName (jointName);
	  localSetJointBounds(joint, jointBounds);
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
	  for (std::size_t i=0; i<jacobian.rows (); ++i) {
	    hpp::floatSeq row; row.length (jacobian.cols ());
	    for (std::size_t j=0; j<jacobian.cols (); ++j) {
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

      hpp::floatSeq* Robot::gradientConfigurationWrtProjection (const hpp::floatSeq& dofArray) 
        throw (hpp::Error){
          //###################################################################
          //## set new configuration, so that capsules have the right transform
          //###################################################################
          this->setCurrentConfig(dofArray);
          return gradientConfigurationWrtProjection ();
      }

      hpp::floatSeq* Robot::gradientConfigurationWrtProjection () 
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
              stream << "JOINT has no body" << endl;
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
      hpp::floatSeq* Robot::getRandomConfig() throw (hpp::Error){
        DevicePtr_t robot = problemSolver_->robot ();
        hpp::core::BasicConfigurationShooter confShooter(robot);
        //configurationptr_t is vector_t is eigen::matrix<double>
        ConfigurationPtr_t configuration = confShooter.shoot();
        hppDout(notice, "RANDOM CONF >> " << configuration.get());

        hpp::model::ConfigurationIn_t random_configuration = *configuration.get();
        hppDout(notice, "RANDOM CONF >> " << random_configuration);
        robot->currentConfiguration(random_configuration);
        return getCurrentConfig();
      }

      hpp::floatSeq* Robot::projectConfigurationUntilIrreducible (const hpp::floatSeq& dofArray) 
        throw (hpp::Error)
      {
        // configuration, get convex projected hull, compute jacobian of outer points,
        // get complete gradient, do gradient descent into (0,0) direction of each point

        return this->gradientConfigurationWrtProjection(dofArray);
      }

      hpp::floatSeq* Robot::computeVolume () throw (hpp::Error)
      {
        return this->parseCapsulePoints();
      }

      hpp::floatSeq* Robot::parseCapsulePoints () throw (hpp::Error)
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

      // --------------------------------------------------------------------

    } // end of namespace impl.
  } // end of namespace corbaServer.
} // end of namespace hpp.
