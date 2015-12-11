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
#include <sstream>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/algorithm/string.hpp>    

#include <hpp/util/debug.hh>
#include <hpp/util/portability.hh>

#include <hpp/fcl/shape/geometric_shapes.h>

#include <hpp/model/configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/locked-joint.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/comparison-type.hh>
#include <hpp/core/parser/roadmap-factory.hh>

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/distance-between-bodies.hh>
#include <hpp/constraints/position.hh>
#include <hpp/constraints/orientation.hh>
#include <hpp/constraints/transformation.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/com-between-feet.hh>
#include <hpp/constraints/relative-orientation.hh>
#include <hpp/constraints/relative-transformation.hh>
#include <hpp/constraints/relative-position.hh>
#include <hpp/constraints/convex-shape-contact.hh>
#include <hpp/constraints/static-stability.hh>
#include <hpp/constraints/configuration-constraint.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/model/body.hh>
#include <hpp/model/center-of-mass-computation.hh>

#include "problem.impl.hh"
#include "tools.hh"

using hpp::model::ObjectVector_t;
using hpp::model::CenterOfMassComputation;
using hpp::model::CenterOfMassComputationPtr_t;

using hpp::constraints::DistanceBetweenBodies;
using hpp::constraints::Orientation;
using hpp::constraints::OrientationPtr_t;
using hpp::constraints::Position;
using hpp::constraints::PositionPtr_t;
using hpp::constraints::RelativeOrientation;
using hpp::constraints::ComBetweenFeetPtr_t;
using hpp::constraints::ComBetweenFeet;
using hpp::constraints::RelativeComPtr_t;
using hpp::constraints::RelativeCom;
using hpp::constraints::RelativeOrientationPtr_t;
using hpp::constraints::RelativePosition;
using hpp::constraints::RelativePositionPtr_t;
using hpp::constraints::RelativeTransformation;
using hpp::constraints::Transformation;
using hpp::constraints::ConvexShapeContact;
using hpp::constraints::ConvexShapeContactPtr_t;
using hpp::constraints::StaticStability;
using hpp::constraints::StaticStabilityPtr_t;

using hpp::core::NumericalConstraint;

namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      static ConfigurationPtr_t floatSeqToConfig
      (hpp::core::ProblemSolverPtr_t problemSolver,
       const hpp::floatSeq& dofArray)
      {
	size_type configDim = (size_type)dofArray.length();
	ConfigurationPtr_t config (new Configuration_t (configDim));

	// Get robot in hppPlanner object.
	DevicePtr_t robot = problemSolver->robot ();

	// Compare size of input array with number of degrees of freedom of
	// robot.
	if (configDim != robot->configSize ()) {
	  hppDout (error, "robot nb dof=" << configDim <<
		   " is different from config size=" << robot->configSize());
	  throw std::runtime_error
	    ("robot nb dof is different from config size");
	}

	// Fill dof vector with dof array.
	for (size_type iDof=0; iDof < configDim; ++iDof) {
	  (*config) [iDof] = dofArray [(CORBA::ULong)iDof];
	}
	return config;
      }

      static vector3_t floatSeqToVector3 (const hpp::floatSeq& dofArray)
      {
	if (dofArray.length() != 3) {
	  std::ostringstream oss
	    ("Expecting vector of size 3, got vector of size");
	  oss << dofArray.length() << ".";
	  throw hpp::Error (oss.str ().c_str ());
	}
	// Fill dof vector with dof array.
	vector3_t result;
	for (unsigned int iDof=0; iDof < 3; ++iDof) {
	  result [iDof] = dofArray [iDof];
	}
	return result;
      }

      static vector_t floatSeqToVector (const hpp::floatSeq& dofArray)
      {
	// Fill dof vector with dof array.
	vector_t result (dofArray.length ());
	for (size_type iDof=0; iDof < result.size (); ++iDof) {
	  result [iDof] = dofArray [(CORBA::ULong)iDof];
	}
	return result;
      }

      Problem::Problem (corbaServer::Server* server)
	: server_ (server),
	  problemSolver_ (server->problemSolver ())
      {}

      // ---------------------------------------------------------------

      Names_t* Problem::getAvailable (const char* what) throw (hpp::Error)
      {
        std::string w (what);
        boost::algorithm::to_lower(w);
        typedef std::list <std::string> Ret_t;
        Ret_t ret;

        if (w == "pathoptimizer") {
          ret = problemSolver_->getKeys <core::PathOptimizerBuilder_t, Ret_t> ();
        } else if (w == "pathprojector") {
          ret = problemSolver_->getKeys <core::PathProjectorBuilder_t, Ret_t> ();
        } else if (w == "pathplanner") {
          ret = problemSolver_->getKeys <core::PathPlannerBuilder_t, Ret_t> ();
        } else if (w == "configurationshooter") {
          ret = problemSolver_->getKeys <core::ConfigurationShooterBuilder_t, Ret_t> ();
        } else if (w == "pathvalidation") {
          ret = problemSolver_->getKeys <core::PathValidationBuilder_t, Ret_t> ();
        } else if (w == "steeringmethod") {
          ret = problemSolver_->getKeys <core::SteeringMethodBuilder_t, Ret_t> ();
        } else if (w == "numericalconstraint") {
          ret = problemSolver_->getKeys <core::NumericalConstraintPtr_t, Ret_t> ();
        } else if (w == "type") {
          ret = boost::assign::list_of ("PathOptimizer") ("PathProjector")
            ("PathPlanner") ("ConfigurationShooter") ("SteeringMethod")
            ("PathValidation") ("NumericalConstraint");
        } else {
          throw Error ("Type not understood");
        }

        char** nameList = Names_t::allocbuf(ret.size());
        Names_t *names = new Names_t (ret.size(), ret.size(), nameList);
        std::size_t i = 0;
        for (Ret_t::const_iterator it = ret.begin (); it != ret.end(); ++it) {
          nameList [i] =
            (char*) malloc (sizeof(char)*(it->length ()+1));
            strcpy (nameList [i], it->c_str ());
            ++i;
        }
        return names;
      }

      // ---------------------------------------------------------------

      void Problem::setInitialConfig (const hpp::floatSeq& dofArray)
	throw (hpp::Error)
      {
	try {
	  ConfigurationPtr_t config = floatSeqToConfig (problemSolver_,
							dofArray);
	  problemSolver_->initConfig (config);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::getInitialConfig()
	throw(hpp::Error)
      {
	hpp::floatSeq *dofArray;

	// Get robot in hppPlanner object.
	ConfigurationPtr_t config = problemSolver_->initConfig ();

	if (config) {
	  std::size_t deviceDim = config->size();

	  dofArray = new hpp::floatSeq();
	  dofArray->length((CORBA::ULong)deviceDim);

	  for(unsigned int i=0; i<deviceDim; i++){
	    (*dofArray)[i] = (*config) [i];
	  }
	  return dofArray;
	}
	else {
	  throw hpp::Error ("no initial configuration defined");
	}
      }

      // ---------------------------------------------------------------

      void Problem::addGoalConfig (const hpp::floatSeq& dofArray)
	throw (hpp::Error)
      {
	try {
	  ConfigurationPtr_t config = floatSeqToConfig (problemSolver_,
							dofArray);
	  problemSolver_->addGoalConfig (config);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::floatSeqSeq* Problem::getGoalConfigs () throw(hpp::Error)
      {
	try {
	  hpp::floatSeqSeq *configSequence;
	  const core::Configurations_t goalConfigs
	    (problemSolver_->goalConfigs ());
	  std::size_t nbGoalConfig = goalConfigs.size ();
	  configSequence = new hpp::floatSeqSeq ();
	  configSequence->length ((CORBA::ULong)nbGoalConfig);
	  for (std::size_t i=0; i<nbGoalConfig ;++i) {
	    const ConfigurationPtr_t& config = goalConfigs [i];
	    std::size_t deviceDim = config->size ();

	    hpp::floatSeq dofArray;
	    dofArray.length ((CORBA::ULong)deviceDim);

	    for (std::size_t j=0; j<deviceDim; ++j)
	      dofArray[(CORBA::ULong)j] = (*config) [j];
	    (*configSequence) [(CORBA::ULong)i] = dofArray;
	  }
	  return configSequence;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::resetGoalConfigs () throw (hpp::Error)
      {
	problemSolver_->resetGoalConfigs ();
      }

      std::vector<bool> boolSeqToBoolVector (const hpp::boolSeq& mask, unsigned int length = 3)
      {
        if (mask.length () != length) {
          std::stringstream ss; ss << "Mask must be of length " << length;
	  throw hpp::Error (ss.str().c_str());
        }
        std::vector<bool> m (length);
	for (size_t i=0; i<length; i++)
	  m[i] = mask[(CORBA::ULong)i];
	return m;
      }

      // ---------------------------------------------------------------

      void Problem::createOrientationConstraint
      (const char* constraintName, const char* joint1Name,
       const char* joint2Name, const Double* p, const hpp::boolSeq& mask)
	throw (hpp::Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	JointPtr_t joint1;
	JointPtr_t joint2;
	size_type constrainedJoint = 0;
	fcl::Quaternion3f quat (p [0], p [1], p [2], p [3]);
	hpp::model::matrix3_t rotation;
	quat.toRotation (rotation);

	std::vector<bool> m = boolSeqToBoolVector (mask, 3);
	try {
	  // Test whether joint1 is world frame
	  if (std::string (joint1Name) == std::string ("")) {
	    constrainedJoint = 2;
	  } else {
	    joint1 =
	      problemSolver_->robot()->getJointByName(joint1Name);
	  }
	  // Test whether joint2 is world frame
	  if (std::string (joint2Name) == std::string ("")) {
	    if (constrainedJoint == 2) {
	      throw hpp::Error ("At least one joint should be provided.");
	    }
	    constrainedJoint = 1;
	  } else {
	    joint2 =
	      problemSolver_->robot()->getJointByName(joint2Name);
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
  std::string name (constraintName);
	if (constrainedJoint == 0) {
	  // Both joints are provided
	  problemSolver_->addNumericalConstraint
	    (name, NumericalConstraint::create
	     (RelativeOrientation::create
	      (name, problemSolver_->robot(), joint1, joint2, rotation, m)));
	} else {
	  JointPtr_t joint = constrainedJoint == 1 ? joint1 : joint2;
	  problemSolver_->addNumericalConstraint
	    (name, NumericalConstraint::create
	     (Orientation::create (name, problemSolver_->robot(), joint,
				   rotation, m)));
	}
      }

      // ---------------------------------------------------------------

      void Problem::createTransformationConstraint
      (const char* constraintName, const char* joint1Name,
       const char* joint2Name, const Transform_ p, const hpp::boolSeq& mask)
	throw (hpp::Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	JointPtr_t joint1;
	JointPtr_t joint2;
	size_type constrainedJoint = 0;
        fcl::Vec3f vec (p[0], p[1], p[2]);
	fcl::Quaternion3f quat (p [3], p [4], p [5], p [6]);
        fcl::Transform3f ref (quat, vec);

	std::vector<bool> m = boolSeqToBoolVector (mask, 6);
	try {
	  // Test whether joint1 is world frame
	  if (std::string (joint1Name) == std::string ("")) {
	    constrainedJoint = 2;
	  } else {
	    joint1 =
	      problemSolver_->robot()->getJointByName(joint1Name);
	  }
	  // Test whether joint2 is world frame
	  if (std::string (joint2Name) == std::string ("")) {
	    if (constrainedJoint == 2) {
	      throw hpp::Error ("At least one joint should be provided.");
	    }
	    constrainedJoint = 1;
	  } else {
	    joint2 =
	      problemSolver_->robot()->getJointByName(joint2Name);
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
  std::string name (constraintName);
	if (constrainedJoint == 0) {
	  // Both joints are provided
	  problemSolver_->addNumericalConstraint
	    (name, NumericalConstraint::create
	     (RelativeTransformation::create
	      (name, problemSolver_->robot(), joint1, joint2, ref, m)));
	} else {
	  JointPtr_t joint = constrainedJoint == 1 ? joint1 : joint2;
	  problemSolver_->addNumericalConstraint
	    (name, NumericalConstraint::create
	     (Transformation::create (name, problemSolver_->robot(), joint,
				   ref, m)));
	}
      }

      // ---------------------------------------------------------------

      void Problem::createRelativeComConstraint (const char* constraintName,
          const char* comName, const char* jointName, const floatSeq& p,
          const hpp::boolSeq& mask)
        throw (hpp::Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	JointPtr_t joint;
        model::CenterOfMassComputationPtr_t comc;
	vector3_t point = floatSeqToVector3 (p);

	std::vector<bool> m = boolSeqToBoolVector (mask);
	try {
          joint = problemSolver_->robot()->getJointByName(jointName);
	  // Test whether joint1 is world frame
          std::string name (constraintName), comN (comName);
          if (comN.compare ("") == 0) {
            problemSolver_->addNumericalConstraint
              (name, NumericalConstraint::create
	       (RelativeCom::create (problemSolver_->robot(),
				     joint, point, m)));
          } else {
            comc = problemSolver_->centerOfMassComputation (comN);
            if (!comc)
              throw hpp::Error ("Partial COM not found.");
            problemSolver_->addNumericalConstraint
              (name, NumericalConstraint::create
	       (RelativeCom::create (problemSolver_->robot(), comc,
				     joint, point, m)));
          }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::createComBeetweenFeet
      (const char* constraintName, const char* comName, const char* jointLName,
       const char* jointRName, const floatSeq& pL, const floatSeq& pR,
       const char* jointRefName, const hpp::boolSeq& mask)
	throw (hpp::Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	JointPtr_t jointL, jointR, jointRef;
        model::CenterOfMassComputationPtr_t comc;
	vector3_t pointL = floatSeqToVector3 (pL);
	vector3_t pointR = floatSeqToVector3 (pR);
	vector3_t pointRef (0,0,0);

	std::vector<bool> m = boolSeqToBoolVector (mask);
	try {
          jointL = problemSolver_->robot()->getJointByName(jointLName);
          jointR = problemSolver_->robot()->getJointByName(jointRName);
	  // Test whether joint1 is world frame
          if (std::string (jointRefName) == std::string (""))
            jointRef = problemSolver_->robot()->rootJoint ();
	  else
	    jointRef = problemSolver_->robot()->getJointByName(jointRefName);
          std::string name (constraintName), comN (comName);
          if (comN.compare ("") == 0) {
            problemSolver_->addNumericalConstraint
              (name, NumericalConstraint::create
	       (ComBetweenFeet::create (name, problemSolver_->robot(),
					jointL, jointR, pointL, pointR,
					jointRef, pointRef, m)));
          } else {
            comc = problemSolver_->centerOfMassComputation (comN);
            if (!comc)
              throw hpp::Error ("Partial COM not found.");
            problemSolver_->addNumericalConstraint
              (name, NumericalConstraint::create
	       (ComBetweenFeet::create (name, problemSolver_->robot(), comc,
					jointL, jointR, pointL, pointR,
					jointRef, pointRef, m)));
          }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::createStaticStabilityGravityConstraint
      (const char* constraintName, const Names_t& floorJoints,
       const Names_t& objectJoints,
       const hpp::floatSeqSeq& points, const hpp::intSeqSeq& objTriangles,
       const hpp::intSeqSeq& floorTriangles)
        throw (hpp::Error)
      {
        createConvexShapeContactConstraint (constraintName, floorJoints,
            objectJoints, points, objTriangles, floorTriangles);
      }

      // ---------------------------------------------------------------

      void Problem::createConvexShapeContactConstraint
        (const char* constraintName, const Names_t& floorJoints,
         const Names_t& objectJoints,
         const hpp::floatSeqSeq& points, const hpp::intSeqSeq& objTriangles,
         const hpp::intSeqSeq& floorTriangles)
        throw (hpp::Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
        try {
          std::string name (constraintName);
          ConvexShapeContactPtr_t f = ConvexShapeContact::create
            (name, problemSolver_->robot());
          problemSolver_->addNumericalConstraint
	    (name, NumericalConstraint::create (f));
          std::vector <fcl::Vec3f> pts (points.length ());
          for (CORBA::ULong i = 0; i < points.length (); ++i) {
            if (points[i].length () != 3)
              throw hpp::Error ("Points must be of size 3.");
            pts [i] = fcl::Vec3f (points[i][0],points[i][1],points[i][2]);
          }
	  if (objectJoints.length () != objTriangles.length ()) {
	    std::ostringstream oss;
	    oss << "Number of object joints (" << objectJoints.length ()
		<< ") should fit number of objTriangles ("
		<< objTriangles.length () << ").";
	    throw std::runtime_error (oss.str ());
	  }
          for (CORBA::ULong i = 0; i < objTriangles.length (); ++i) {
            if (objTriangles[i].length () != 3)
              throw hpp::Error ("Triangle must have size 3.");

            for (size_t j = 0; j < 3; j++)
              if (objTriangles[i][(CORBA::ULong)j] < 0 &&
		  (size_t) objTriangles[i][(CORBA::ULong)j] >= pts.size())
                throw hpp::Error ("Point index out of range.");

	    std::string jointName (objectJoints [i]);
	    JointPtr_t joint;
	    if (jointName == "None") {
	      joint = 0x0;
	    } else {
	      joint = problemSolver_->robot ()->getJointByName (jointName);
	    }
            std::vector <core::vector3_t> shapePts = boost::assign::list_of
                  (pts [objTriangles[i][0]])
                  (pts [objTriangles[i][1]])
                  (pts [objTriangles[i][2]]);
            f->addObject (constraints::ConvexShape (shapePts, joint));
          }
	  if (floorJoints.length () != floorTriangles.length ()) {
	    std::ostringstream oss;
	    oss << "Number of floor joints (" << floorJoints.length ()
		<< ") should fit number of floorTriangles ("
		<< floorTriangles.length () << ").";
	    throw std::runtime_error (oss.str ());
	  }
          for (CORBA::ULong i = 0; i < floorTriangles.length (); ++i) {
            if (floorTriangles[i].length () != 3)
              throw hpp::Error ("Triangle must have size 3.");
            for (size_t j = 0; j < 3; j++)
              if (floorTriangles[i][(CORBA::ULong)j] < 0 &&
		  (size_t) floorTriangles[i][(CORBA::ULong)j] >= pts.size())
                throw hpp::Error ("Point index out of range.");

	    std::string jointName (floorJoints [i]);
	    JointPtr_t joint;
	    if (jointName == "None") {
	      joint = 0x0;
	    } else {
	      joint = problemSolver_->robot ()->getJointByName (jointName);
	    }
            std::vector <core::vector3_t> shapePts = boost::assign::list_of
                  (pts [floorTriangles[i][0]])
                  (pts [floorTriangles[i][1]])
                  (pts [floorTriangles[i][2]]);
            f->addFloor (constraints::ConvexShape (shapePts, joint));
          }
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      void Problem::createStaticStabilityConstraint
      (const char* constraintName, const hpp::Names_t& jointNames,
       const hpp::floatSeqSeq& points, const hpp::floatSeqSeq& normals,
       const char* comRootJointName)
        throw (hpp::Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
        DevicePtr_t robot = problemSolver_->robot();
        JointPtr_t comRJ;
        try {
          std::string name (constraintName);
          /// Create the com
          comRJ = robot->getJointByName(comRootJointName);
          CenterOfMassComputationPtr_t com =
            CenterOfMassComputation::create (robot);
          com->add (comRJ);
          com->computeMass ();

          /// Create the contacts
          StaticStability::Contacts_t contacts;
          if (jointNames.length () != points.length()
              || jointNames.length () != normals.length()) {
            throw Error ("There should be as many joint names, points and normals");
          }
          for (CORBA::ULong i = 0; i < jointNames.length (); i+=2) {
            StaticStability::Contact_t c;
            // Set joints
            std::string jn1 (jointNames[i  ]);
            std::string jn2 (jointNames[i+1]);
            if (jn1.empty ()) c.joint1 = NULL;
            else c.joint1 = robot->getJointByName (jn1);
            if (jn2.empty ()) c.joint2 = NULL;
            else c.joint2 = robot->getJointByName (jn2);
            // Set points and normals
            if (points[i].length () != 3 || points[i+1].length () != 3
                || normals[i].length () != 3 || normals[i+1].length () != 3)
              throw Error ("Points and normals must be of length 3");
            for (size_t j = 0; j < 3; j++) {
              c.point1[j]  = points [i  ][j];
              c.point2[j]  = points [i+1][j];
              c.normal1[j] = normals[i  ][j];
              c.normal2[j] = normals[i+1][j];
            }
            contacts.push_back (c);
          }
          StaticStabilityPtr_t f = StaticStability::create
            (name, robot, contacts, com);
          problemSolver_->addNumericalConstraint (name,
              NumericalConstraint::create (f, core::EqualToZero::create())
              );
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      void Problem::createPositionConstraint
      (const char* constraintName, const char* joint1Name,
       const char* joint2Name, const hpp::floatSeq& point1,
       const hpp::floatSeq& point2, const hpp::boolSeq& mask)
	throw (hpp::Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	JointPtr_t joint1;
	JointPtr_t joint2;
	vector3_t targetInWorldFrame;
	vector3_t targetInLocalFrame;
	vector3_t p1 = floatSeqToVector3 (point1);
	vector3_t p2 = floatSeqToVector3 (point2);
	size_type constrainedJoint = 0;
	std::vector<bool> m = boolSeqToBoolVector (mask);
	try {
	  // Test whether joint1 is world frame
	  if (std::string (joint1Name) == std::string ("")) {
	    constrainedJoint = 2;
	    targetInWorldFrame = p1;
	    targetInLocalFrame = p2;
	  } else {
	    joint1 =
	      problemSolver_->robot()->getJointByName(joint1Name);
	  }
	  // Test whether joint2 is world frame
	  if (std::string (joint2Name) == std::string ("")) {
	    if (constrainedJoint == 2) {
	      throw hpp::Error ("At least one joint should be provided.");
	    }
	    constrainedJoint = 1;
	    targetInWorldFrame = p2;
	    targetInLocalFrame = p1;
	  } else {
	    joint2 =
	      problemSolver_->robot()->getJointByName(joint2Name);
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	std::string name (constraintName);
	if (constrainedJoint == 0) {
	  // Both joints are provided
	  problemSolver_->addNumericalConstraint
	    (name, NumericalConstraint::create
	     (RelativePosition::create
	      (name, problemSolver_->robot(), joint1, joint2, p1, p2, m)));
	} else {
	  hpp::model::matrix3_t I3; I3.setIdentity ();
	  JointPtr_t joint = constrainedJoint == 1 ? joint1 : joint2;
	  problemSolver_->addNumericalConstraint
	    (name, NumericalConstraint::create
	     (Position::create (name, problemSolver_->robot(), joint,
				targetInLocalFrame, targetInWorldFrame, I3,m)));
	}
      }

      // ---------------------------------------------------------------

      void Problem::createConfigurationConstraint (const char* constraintName,
          const hpp::floatSeq& goal) throw (hpp::Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	ConfigurationPtr_t config = floatSeqToConfig (problemSolver_, goal);
	std::string name (constraintName);
        problemSolver_->add
          (name, NumericalConstraint::create
           (constraints::ConfigurationConstraint::create
            (name, problemSolver_->robot(), *config)
            ));
      }

      // ---------------------------------------------------------------

      void Problem::createDistanceBetweenJointConstraint
      (const char* constraintName, const char* joint1Name,
       const char* joint2Name, Double) throw (Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	try {
	  JointPtr_t joint1 = problemSolver_->robot ()->getJointByName
	    (joint1Name);
	  JointPtr_t joint2 = problemSolver_->robot ()->getJointByName
	    (joint2Name);
	  std::string name (constraintName);
	  problemSolver_->addNumericalConstraint
	    (name, NumericalConstraint::create
	     (DistanceBetweenBodies::create (name, problemSolver_->robot(),
					     joint1, joint2)));
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::createDistanceBetweenJointAndObjects
      (const char* constraintName, const char* joint1Name,
       const hpp::Names_t& objects, Double) throw (Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	try {
	  JointPtr_t joint1 = problemSolver_->robot ()->getJointByName
	    (joint1Name);
	  ObjectVector_t objectList;
	  for (CORBA::ULong i=0; i<objects.length (); ++i) {
	    objectList.push_back (problemSolver_->obstacle
				  (std::string (objects [i])));
	  }
	  std::string name (constraintName);
	  problemSolver_->addNumericalConstraint
	    (name, NumericalConstraint::create
	     (DistanceBetweenBodies::create (name, problemSolver_->robot(),
					     joint1, objectList)));
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      bool Problem::applyConstraints (const hpp::floatSeq& input,
				      hpp::floatSeq_out output,
				      double& residualError)
	throw (hpp::Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	bool success = false;
	ConfigurationPtr_t config = floatSeqToConfig (problemSolver_, input);
	try {
	  success = problemSolver_->constraints ()->apply (*config);
	  if (hpp::core::ConfigProjectorPtr_t configProjector =
	      problemSolver_->constraints ()->configProjector ()) {
	    residualError = configProjector->residualError ();
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	output = vectorToFloatseq (*config);
	return success;
      }

      // ---------------------------------------------------------------

      void Problem::computeValueAndJacobian
      (const hpp::floatSeq& config, hpp::floatSeq_out value,
       hpp::floatSeqSeq_out jacobian) throw (hpp::Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	try {
	  ConfigurationPtr_t configuration = floatSeqToConfig
	    (problemSolver_, config);
	  vector_t v;
	  matrix_t J;
	  problemSolver_->computeValueAndJacobian (*configuration, v, J);
	  value = vectorToFloatseq (v);
	  jacobian = matrixToFloatSeqSeq (J);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      bool Problem::generateValidConfig (UShort maxIter,
				      hpp::floatSeq_out output,
				      double& residualError)
	throw (hpp::Error)
      {
        DevicePtr_t robot = problemSolver_->robot ();
	if (!robot) throw hpp::Error ("No robot loaded");
        core::BasicConfigurationShooterPtr_t shooter
          = core::BasicConfigurationShooter::create (robot);
	bool success = false, configIsValid = false;
        ConfigurationPtr_t config;
        while (!configIsValid && maxIter > 0)
        {
          try {
            config = shooter->shoot ();
            success = problemSolver_->constraints ()->apply (*config);
            if (hpp::core::ConfigProjectorPtr_t configProjector =
                problemSolver_->constraints ()->configProjector ()) {
              residualError = configProjector->residualError ();
            }
            if (success) {
              robot->currentConfiguration (*config);
              configIsValid = !robot->collisionTest ();
            }
          } catch (const std::exception& exc) {
            throw hpp::Error (exc.what ());
          }
          maxIter--;
        }
	ULong size = (ULong) config->size ();
	hpp::floatSeq* q_ptr = new hpp::floatSeq ();
	q_ptr->length (size);

	for (std::size_t i=0; i<size; ++i) {
	  (*q_ptr) [(CORBA::ULong)i] = (*config) [i];
	}
	output = q_ptr;
	return configIsValid;
      }

      // ---------------------------------------------------------------

      void Problem::resetConstraints ()	throw (hpp::Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	try {
	  problemSolver_->resetConstraints ();
	  problemSolver_->robot ()->controlComputation
	    (model::Device::JOINT_POSITION);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::addPassiveDofs (const char* passiveDofsName,
          const hpp::Names_t& dofNames)
        throw (hpp::Error)
      {
        DevicePtr_t robot = problemSolver_->robot ();
        if (!robot) throw hpp::Error ("No robot loaded");
        std::vector <size_type> dofs;
        /// First, translate names into velocity indexes.
        for (CORBA::ULong i=0; i<dofNames.length (); ++i) {
          std::string name (dofNames[i]);
          JointPtr_t j = robot->getJointByName (name);
          if (!j)
            throw hpp::Error ("One joint not found.");
          for (size_type i = 0; i < j->numberDof (); i++)
            dofs.push_back (j->rankInVelocity() + i);
        }
        /// Then, sort and remove non duplicated elements.
        std::sort (dofs.begin (), dofs.end ());
        std::vector <size_type>::iterator last =
          std::unique (dofs.begin (), dofs.end ());
        dofs.erase (last, dofs.end ());
        /// Then, create the intervals.
        core::SizeIntervals_t passiveDofs;
        dofs.push_back (robot->numberDof () + 1);
        size_type intStart = dofs[0], intEnd = dofs[0];
        for (size_t i = 1; i < dofs.size (); i++) {
          intEnd ++;
          if (intEnd == dofs[i]) {
            continue;
          } else {
            passiveDofs.push_back (
                core::SizeInterval_t (intStart, intEnd - intStart));
            intStart = intEnd = dofs[i];
          }
        }
        problemSolver_->addPassiveDofs (passiveDofsName, passiveDofs);
      }

      // ---------------------------------------------------------------

      void Problem::setConstantRightHandSide (const char* constraintName,
					      CORBA::Boolean constant)
	throw (hpp::Error)
      {
	try {
	  core::ComparisonTypePtr_t comparison =
	    problemSolver_->comparisonType (constraintName);
	  if (!comparison)
	    throw std::runtime_error
	      (std::string ("Numerical constraint ") + constraintName +
	       std::string ("can not be found."));
	  if (constant) {
	    problemSolver_->comparisonType (constraintName,
					   core::EqualToZero::create ());
	  } else {
	    problemSolver_->comparisonType (constraintName,
					   core::Equality::create ());
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      bool Problem::getConstantRightHandSide (const char* constraintName)
	throw (hpp::Error)
      {
	try {
	  core::ComparisonTypePtr_t comparison =
	    problemSolver_->comparisonType (constraintName);
	  if (!comparison) {
	    throw std::runtime_error
	      (std::string ("Numerical constraint ") + constraintName +
	       std::string ("can not be found."));
	  }
	  return comparison->constantRightHandSide ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::setNumericalConstraints
      (const char* constraintName, const Names_t& constraintNames,
       const hpp::intSeq& priorities)
	throw (Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	try {
	  for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
	    std::string name (constraintNames [i]);
            problemSolver_->addFunctionToConfigProjector (constraintName, name,
                (std::size_t)priorities[i]);
	    problemSolver_->robot ()->controlComputation
	      (model::Device::ALL);
	  }
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::lockJoint (const char* jointName,
			       const hpp::floatSeq& value)
	throw (hpp::Error)
      {
	if (!problemSolver_->robot ()) throw hpp::Error ("No robot loaded");
	try {
	  // Get robot in hppPlanner object.
	  DevicePtr_t robot = problemSolver_->robot ();
	  JointPtr_t joint = robot->getJointByName (jointName);
	  vector_t jointConfig = floatSeqToVector (value);

	  LockedJointPtr_t lockedJoint (LockedJoint::create (joint, jointConfig));
	  problemSolver_->addLockedJoint (lockedJoint);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::setErrorThreshold (Double threshold) throw (Error)
      {
	problemSolver_->errorThreshold (threshold);
      }

      // ---------------------------------------------------------------
      void Problem::setMaxIterations (UShort iterations) throw (Error)
      {
	problemSolver_->maxIterations (iterations);
      }

      // ---------------------------------------------------------------

      void Problem::selectPathPlanner (const char* pathPlannerType)
	throw (Error)
      {
	try {
	  problemSolver_->pathPlannerType (std::string (pathPlannerType));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::selectConFigurationShooter (const char* configurationShooterType)
    throw (Error)
      {
    try {
      problemSolver_->configurationShooterType (std::string (configurationShooterType));
    } catch (const std::exception& exc) {
      throw hpp::Error (exc.what ());
    }
      }

      // ---------------------------------------------------------------

      void Problem::selectSteeringMethod (const char* steeringMethodType)
    throw (Error)
      {
    try {
      problemSolver_->steeringMethodType (std::string (steeringMethodType));
    } catch (const std::exception& exc) {
      throw hpp::Error (exc.what ());
    }
      }

      // ---------------------------------------------------------------
      void Problem::addPathOptimizer (const char* pathOptimizerType)
	throw (Error)
      {
	try {
	  problemSolver_->addPathOptimizer (std::string (pathOptimizerType));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::clearPathOptimizers () throw (Error)
      {
	try {
	  problemSolver_->clearPathOptimizers ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::selectPathValidation (const char* pathValidationType,
					  Double tolerance) throw (Error)
      {
	try {
	  problemSolver_->pathValidationType (std::string (pathValidationType),
					      tolerance);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::selectPathProjector (const char* pathProjectorType,
                                         Double tolerance) throw (Error)
      {
        try {
          problemSolver_->pathProjectorType (std::string (pathProjectorType),
                                             tolerance);
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      bool Problem::prepareSolveStepByStep () throw (hpp::Error)
      {
	try {
	  return problemSolver_->prepareSolveStepByStep ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
        return false;
      }

      // ---------------------------------------------------------------

      bool Problem::executeOneStep () throw (hpp::Error)
      {
	try {
	  return problemSolver_->executeOneStep ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
        return false;
      }

      // ---------------------------------------------------------------

      void Problem::finishSolveStepByStep () throw (hpp::Error)
      {
	try {
	  problemSolver_->solve();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::intSeq* Problem::solve () throw (hpp::Error)
      {
	try {
          boost::posix_time::ptime start =
            boost::posix_time::microsec_clock::universal_time ();
	  problemSolver_->solve();
          boost::posix_time::time_duration time =
            boost::posix_time::microsec_clock::universal_time () - start;
          hpp::intSeq *ret = new hpp::intSeq;
          ret->length (4);
          (*ret)[0] = time.hours ();
          (*ret)[1] = time.minutes ();
          (*ret)[2] = time.seconds ();
          (*ret)[3] = (long) (time.fractional_seconds () / 1000);
          return ret;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::directPath (const hpp::floatSeq& startConfig,
				const hpp::floatSeq& endConfig)
	throw (hpp::Error)
      {
	ConfigurationPtr_t start;
	ConfigurationPtr_t end;
	try {
	  start = floatSeqToConfig (problemSolver_, startConfig);
	  end = floatSeqToConfig (problemSolver_, endConfig);
	  if (!problemSolver_->problem ()) {
	    problemSolver_->resetProblem ();
	  }
	  SteeringMethodPtr_t sm = problemSolver_->problem ()->steeringMethod ();
	  PathPtr_t dp = (*sm) (*start, *end);
	  PathPtr_t unused;
	  PathValidationReportPtr_t report;
	  if (!problemSolver_->problem()->pathValidation ()->validate
	      (dp, false, unused, report)) {
	    std::ostringstream oss; oss << *report;
	    throw hpp::Error (oss.str ().c_str ());
	  }
	  // Add Path in problem
	  PathVectorPtr_t path
	    (core::PathVector::create (dp->outputSize (),
				       dp->outputDerivativeSize ()));
	  path->appendPath (dp);
	  problemSolver_->addPath (path);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::appendDirectPath (UShort pathId,
				      const hpp::floatSeq& config)
	throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver_->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver_->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  PathVectorPtr_t path = problemSolver_->paths () [pathId];
	  Configuration_t start (path->end ());
	  ConfigurationPtr_t end (floatSeqToConfig (problemSolver_, config));
	  if (!problemSolver_->problem ()) {
	    problemSolver_->resetProblem ();
	  }
	  SteeringMethodPtr_t sm
	    (problemSolver_->problem ()->steeringMethod ());
	  PathPtr_t dp = (*sm) (start, *end);
	  PathPtr_t unused;
	  PathValidationReportPtr_t report;
	  if (!problemSolver_->problem()->pathValidation ()->validate
	      (dp, false, unused, report)) {
	    std::ostringstream oss; oss << *report;
	    throw hpp::Error (oss.str ().c_str ());
	  }
	  path->appendPath (dp);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      bool Problem::projectPath (UShort pathId)
	throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver_->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver_->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  PathVectorPtr_t initial = problemSolver_->paths () [pathId];
          core::PathProjectorPtr_t pp = problemSolver_->problem ()->pathProjector ();
          if (!pp) throw Error ("There is no path projector");

          PathPtr_t proj;
          bool success = pp->apply (initial, proj);

	  PathVectorPtr_t path
	    (core::PathVector::create (initial->outputSize (),
				       initial->outputDerivativeSize ()));
	  path->appendPath (proj);
	  problemSolver_->addPath (path);
          return success;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::interruptPathPlanning() throw (hpp::Error)
      {
	problemSolver_->interrupt ();
      }

      // ---------------------------------------------------------------

      Short Problem::numberPaths () throw (hpp::Error)
      {
	try {
	  return (Short) problemSolver_->paths ().size ();
	} catch (std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::intSeq* Problem::optimizePath(UShort pathId) throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver_->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver_->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
          // Start timer
          boost::posix_time::ptime start =
            boost::posix_time::microsec_clock::universal_time ();

	  PathVectorPtr_t initial = problemSolver_->paths () [pathId];
	  problemSolver_->optimizePath (initial);

          // Stop timer
          boost::posix_time::time_duration time =
            boost::posix_time::microsec_clock::universal_time () - start;

          hpp::intSeq *ret = new hpp::intSeq;
          ret->length (4);
          (*ret)[0] = time.hours ();
          (*ret)[1] = time.minutes ();
          (*ret)[2] = time.seconds ();
          (*ret)[3] = (long) (time.fractional_seconds () / 1000);
          return ret;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      Double Problem::pathLength (UShort pathId) throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver_->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver_->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  return problemSolver_->paths () [pathId]->length ();
	} catch (std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::configAtParam (UShort pathId,
					     Double atDistance)
	throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver_->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver_->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  PathPtr_t path = problemSolver_->paths () [pathId];
	  bool success;
	  Configuration_t config = (*path) (atDistance, success);
	  if (!success) {
	    throw std::runtime_error ("Failed to apply constraint in path "
				      "evaluation.");
	  }
	  // Allocate result now that the size is known.
	  std::size_t size =  config.size ();
	  double* dofArray = hpp::floatSeq::allocbuf((ULong)size);
	  hpp::floatSeq* floatSeq = new hpp::floatSeq
	    ((CORBA::ULong)size, (CORBA::ULong)size, dofArray, true);
	  for (std::size_t i=0; i < size; ++i) {
	    dofArray[(CORBA::ULong)i] =  config [i];
	  }
	  return floatSeq;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      //---------------------------------

      // Get end point of path (start if points == 0, end if points == 1)
      void findExtremes (PathPtr_t path, hpp::floatSeqSeq& configSequence,
			 const std::size_t points)
      {
	const std::size_t size_increment = 1;
	Configuration_t config, config_1;
	std::vector<Configuration_t> configs;
	hpp::floatSeq dofArray;
	bool success;
	config = (*path) (0, success);
	if (!success) {
	  throw std::runtime_error ("Failed to apply constraint in path "
				    "evaluation.");
	}
	configs.push_back(config);
	config_1 = (*path) (path->length(), success);
	if (!success) {
	  throw std::runtime_error ("Failed to apply constraint in path "
				    "evaluation.");
	}
	configs.push_back(config_1);
	dofArray.length ((CORBA::ULong)config.size());
	// modify configsequence
	configSequence.length (configSequence.length() +
			       (CORBA::ULong)size_increment);
	CORBA::ULong ptr = configSequence.length() -
	  (CORBA::ULong)size_increment;
	for (std::size_t i=0; i<size_increment; ++i) {
	  for (std::size_t j=0; j < ((std::size_t)config.size()); ++j) {
	    dofArray [(CORBA::ULong)j] = configs [points][j];
	  }
	  (configSequence)[ptr+(CORBA::ULong)i] = dofArray;
	}
      }

      //---------------------------------

      void findExtremities (PathVectorPtr_t path,
			    hpp::floatSeqSeq& configSequence)
      {
        std::size_t num_subpaths  = (*path).numberPaths ();
        if (num_subpaths == 1) {
	  findExtremes (path,configSequence,1);
	}
        else {
	  for (std::size_t i = 0; i < num_subpaths; ++i) {
	    PathPtr_t subpath = (*path).pathAtRank(i);
	    PathVectorPtr_t sp =
	      HPP_DYNAMIC_PTR_CAST (hpp::core::PathVector,subpath);
	    if (sp) {
	      findExtremities(sp, configSequence);
	    } else {
	      findExtremes (subpath,configSequence, 1);
	    }
	  }
	}
      }

      // --------------------------------------------------------------

      hpp::floatSeqSeq* Problem::getWaypoints (UShort pathId)
	throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver_->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver_->paths ().size () << ".";
	    throw std::runtime_error (oss.str ().c_str ());
	  }
	  hpp::floatSeqSeq *configSequence;
	  configSequence = new hpp::floatSeqSeq ();
	  PathVectorPtr_t path = problemSolver_->paths () [pathId];
	  //init path
	  findExtremes (path, *configSequence, 0);
	  findExtremities (path, *configSequence);
	  return configSequence;
	}
	catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::floatSeqSeq* Problem::nodes () throw (hpp::Error)
      {
	hpp::floatSeqSeq* res;
	try {
	  const Nodes_t & nodes
	    (problemSolver_->roadmap ()->nodes ());
	  res = new hpp::floatSeqSeq;
	  res->length ((CORBA::ULong)nodes.size ());
	  std::size_t i=0;
	  for (Nodes_t::const_iterator itNode = nodes.begin ();
	       itNode != nodes.end (); itNode++) {
	    ConfigurationPtr_t config = (*itNode)->configuration ();
	    ULong size = (ULong) config->size ();
	    double* dofArray = hpp::floatSeq::allocbuf(size);
	    hpp::floatSeq floats (size, size, dofArray, true);
	    //convert the config in dofseq
	    for (size_type j=0 ; j < config->size() ; ++j) {
	      dofArray[j] = (*config) [j];
	    }
	    (*res) [(CORBA::ULong)i] = floats;
	    ++i;
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	return res;
      }

      // ---------------------------------------------------------------

      Long Problem::numberEdges () throw (hpp::Error)
      {
	return (Long)problemSolver_->roadmap ()->edges ().size ();
      }

      // ---------------------------------------------------------------


      void Problem::edge (ULong edgeId, hpp::floatSeq_out q1,
			  hpp::floatSeq_out q2) throw (hpp::Error)
      {
	try {
	  const Edges_t & edges
	    (problemSolver_->roadmap ()->edges ());
	  Edges_t::const_iterator itEdge = edges.begin ();
	  std::size_t i=0;
	  while (i < edgeId) {
	    ++i; itEdge++;
	  }
	  ConfigurationPtr_t config1 = (*itEdge)->from ()->configuration ();
	  ConfigurationPtr_t config2 = (*itEdge)->to ()->configuration ();
	  ULong size = (ULong) config1->size ();

	  hpp::floatSeq* q1_ptr = new hpp::floatSeq ();
	  q1_ptr->length (size);
	  hpp::floatSeq* q2_ptr = new hpp::floatSeq ();
	  q2_ptr->length (size);

	  for (i=0; i<size; ++i) {
	    (*q1_ptr) [(CORBA::ULong)i] = (*config1) [i];
	    (*q2_ptr) [(CORBA::ULong)i] = (*config2) [i];
	  }
	  q1 = q1_ptr;
	  q2 = q2_ptr;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      Long Problem::numberNodes () throw (hpp::Error)
      {
	return (Long) problemSolver_->roadmap ()->nodes().size();
      }

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::node (ULong nodeId) throw (hpp::Error)
      {
      try {
        const Nodes_t & nodes (problemSolver_->roadmap()->nodes());

        if (nodes.size() > nodeId)
        {
            Nodes_t::const_iterator itNode = boost::next(nodes.begin(),nodeId);
            ConfigurationPtr_t conf = (*itNode)->configuration ();
            ULong size = (ULong) conf->size ();

            hpp::floatSeq* q_ptr = new hpp::floatSeq ();
            q_ptr->length (size);

            for (ULong i=0; i<size; ++i) {
            (*q_ptr) [i] = (*conf) [i];
            }
            return q_ptr;
        }else{
            std::ostringstream oss ("wrong nodeId :");
            oss << nodeId << ", number of nodes: "
            << nodes.size() << ".";
            throw std::runtime_error (oss.str ().c_str ());
        }
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // -----------------------------------------------------------------

      Long Problem::connectedComponentOfEdge (ULong edgeId)
        throw (hpp::Error)
        {
          try {
            const Edges_t & edges (problemSolver_->roadmap()->edges());
            if (edges.size() > edgeId) {
              Edges_t::const_iterator itEdge = boost::next(edges.begin(),edgeId);

              const core::ConnectedComponents_t& ccs = problemSolver_->roadmap()
                ->connectedComponents ();
              core::ConnectedComponents_t::const_iterator itcc = ccs.begin();
              for (std::size_t i = 0; i < ccs.size (); ++i) {
                if (*itcc == (*itEdge)->from()->connectedComponent ()) {
                  return i;
                }
                itcc++;
              }
            }
          } catch (const std::exception& exc) {
            throw hpp::Error (exc.what ());
          }
          throw hpp::Error ("Connected component not found");
        }

      // -----------------------------------------------------------------

      Long Problem::connectedComponentOfNode (ULong nodeId)
        throw (hpp::Error)
        {
          try {
            const Nodes_t & nodes (problemSolver_->roadmap()->nodes());
            if (nodes.size() > nodeId) {
              Nodes_t::const_iterator itNode = boost::next(nodes.begin(),nodeId);

              const core::ConnectedComponents_t& ccs = problemSolver_->roadmap()
                ->connectedComponents ();
              core::ConnectedComponents_t::const_iterator itcc = ccs.begin();
              for (std::size_t i = 0; i < ccs.size (); ++i) {
                if (*itcc == (*itNode)->connectedComponent ()) {
                  return i;
                }
                itcc++;
              }
            }
          } catch (const std::exception& exc) {
            throw hpp::Error (exc.what ());
          }
          throw hpp::Error ("Connected component not found");
        }

      // -----------------------------------------------------------------

      Long Problem::numberConnectedComponents () throw (hpp::Error)
      {
	return
	  (Long) problemSolver_->roadmap ()->connectedComponents ().size ();
      }

      // ---------------------------------------------------------------

      hpp::floatSeqSeq*
      Problem::nodesConnectedComponent (ULong connectedComponentId)
	throw (hpp::Error)
      {
	hpp::floatSeqSeq* res;
	try {
	  const ConnectedComponents_t& connectedComponents
	    (problemSolver_->roadmap ()->connectedComponents ());
	  ConnectedComponents_t::const_iterator itcc =
	    connectedComponents.begin ();
	  ULong i = 0;
	  while (i != connectedComponentId) {
	    ++i; itcc++;
	  }
      const Nodes_t & nodes ((*itcc)->nodes ());
	  res = new hpp::floatSeqSeq;
	  res->length ((CORBA::ULong)nodes.size ());
	  i=0;
	  for (Nodes_t::const_iterator itNode = nodes.begin ();
	       itNode != nodes.end (); itNode++) {
	    ConfigurationPtr_t config = (*itNode)->configuration ();
	    ULong size = (ULong) config->size ();
	    double* dofArray = hpp::floatSeq::allocbuf(size);
	    hpp::floatSeq floats (size, size, dofArray, true);
	    //convert the config in dofseq
	    for (size_type j=0 ; j < config->size() ; ++j) {
	      dofArray [j] = (*config) [j];
	    }
	    (*res) [i] = floats;
	    ++i;
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	return res;
      }

      // ---------------------------------------------------------------

      void Problem::clearRoadmap () throw (hpp::Error)
      {
	try {
	  problemSolver_->roadmap ()->clear ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::resetRoadmap ()
      {
	try {
	  problemSolver_->resetRoadmap ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Problem::saveRoadmap (const char* filename)
        throw (hpp::Error)
      {
        try {
          std::ofstream ofs (filename, std::ofstream::out);
          hpp::core::parser::writeRoadmap (ofs, problemSolver_->problem(),
              problemSolver_->roadmap());
          ofs.close ();
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      void Problem::readRoadmap (const char* filename)
        throw (hpp::Error)
      {
        try {
          problemSolver_->roadmap (
              hpp::core::parser::readRoadmap (std::string(filename),
                problemSolver_->problem())
              );
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }
    } // namespace impl
  } // namespace corbaServer
} // namespace hpp
