// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, Joseph Mirabel,
// JRL.
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

#include "problem.impl.hh"

#include <hpp/fcl/shape/geometric_shapes.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <hpp/constraints/affine-function.hh>
#include <hpp/constraints/com-between-feet.hh>
#include <hpp/constraints/configuration-constraint.hh>
#include <hpp/constraints/convex-shape-contact.hh>
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/distance-between-bodies.hh>
#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/manipulability.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/symbolic-calculus.hh>
#include <hpp/constraints/symbolic-function.hh>
#include <hpp/corbaserver/server-plugin.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/node.hh>
#include <hpp/core/parser/roadmap.hh>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path.hh>
#include <hpp/core/plugin.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/subchain-path.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/serialization.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>
#include <hpp/util/string.hh>
#include <iostream>
#include <iterator>
#include <pinocchio/fwd.hpp>
#include <sstream>

#include "hpp/constraints_idl/constraints-fwd.hh"
#include "hpp/corbaserver/conversions.hh"
#include "hpp/corbaserver/servant-base.hh"
#include "hpp/core_idl/_problem-fwd.hh"
#include "hpp/core_idl/distances-fwd.hh"
#include "hpp/core_idl/path_planners-fwd.hh"
#include "hpp/core_idl/path_projectors-fwd.hh"
#include "hpp/core_idl/path_validations-fwd.hh"
#include "hpp/core_idl/paths-fwd.hh"
#include "hpp/core_idl/steering_methods-fwd.hh"
#include "tools.hh"

using hpp::pinocchio::CenterOfMassComputation;
using hpp::pinocchio::CenterOfMassComputationPtr_t;
using hpp::pinocchio::ObjectVector_t;

using hpp::constraints::ComBetweenFeet;
using hpp::constraints::ComBetweenFeetPtr_t;
using hpp::constraints::ConvexShapeContact;
using hpp::constraints::ConvexShapeContactPtr_t;
using hpp::constraints::DifferentiableFunctionPtr_t;
using hpp::constraints::DistanceBetweenBodies;
using hpp::constraints::LockedJoint;
using hpp::constraints::LockedJointPtr_t;
using hpp::constraints::NumericalConstraints_t;
using hpp::constraints::Orientation;
using hpp::constraints::OrientationPtr_t;
using hpp::constraints::Position;
using hpp::constraints::PositionPtr_t;
using hpp::constraints::RelativeCom;
using hpp::constraints::RelativeComPtr_t;
using hpp::constraints::RelativeOrientation;
using hpp::constraints::RelativeOrientationPtr_t;
using hpp::constraints::RelativePosition;
using hpp::constraints::RelativePositionPtr_t;
using hpp::constraints::RelativeTransformation;
using hpp::constraints::Transformation;

using hpp::constraints::Implicit;
using hpp::constraints::ImplicitPtr_t;

namespace hpp {
namespace corbaServer {
namespace impl {
namespace {
static const matrix3_t I3(matrix3_t ::Identity());
static const vector3_t zero(vector3_t ::Zero());
static const Transform3f Id(Transform3f::Identity());

typedef hpp::core::segment_t segment_t;
typedef hpp::core::segments_t segments_t;

template <int PositionOrientationFlag>
DifferentiableFunctionPtr_t buildGenericFunc(
    const DevicePtr_t& robot, const std::string& name,
    const std::string& nameF1, const std::string& nameF2, const Transform3f& M1,
    const Transform3f& M2, const std::vector<bool> mask) {
  if (nameF1.empty() && nameF2.empty())
    throw hpp::Error("At least one joint should be provided.");
  if (nameF2.empty())
    return buildGenericFunc<PositionOrientationFlag>(robot, name, nameF2,
                                                     nameF1, M2, M1, mask);
  // From here, nameF2 is not empty.

  bool relative = (!nameF1.empty());
  JointPtr_t joint1, joint2;
  Transform3f ref1 = M1, ref2 = M2;
  if (relative) {
    Frame frame = robot->getFrameByName(nameF1);
    ref1 = frame.positionInParentJoint() * M1;
    joint1 = frame.joint();
  }

  Frame frame = robot->getFrameByName(nameF2);
  ref2 = frame.positionInParentJoint() * M2;
  joint2 = frame.joint();

  if (relative) {
    // Both joints are provided
    return constraints::GenericTransformation<
        PositionOrientationFlag | constraints::RelativeBit>::create(name, robot,
                                                                    joint1,
                                                                    joint2,
                                                                    ref1, ref2,
                                                                    mask);
  } else {
    return constraints::GenericTransformation<PositionOrientationFlag>::create(
        name, robot, joint2, ref2, ref1, mask);
  }
}

core::ProblemPtr_t problem(const core::ProblemSolverPtr_t& ps,
                           bool throwIfNull = true) {
  core::ProblemPtr_t p = ps->problem();
  if (throwIfNull && p == NULL) throw Error("No problem in the ProblemSolver");
  return p;
}
}  // namespace

// ---------------------------------------------------------------

Problem::Problem() : server_(NULL) {}

// ---------------------------------------------------------------

void Problem::setMaxNumThreads(UShort n) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    robot->numberDeviceData((size_type)n);
  } catch (const std::exception& e) {
    throw Error(e.what());
  }
}

// ---------------------------------------------------------------

UShort Problem::getMaxNumThreads() {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    return (UShort)robot->numberDeviceData();
  } catch (const std::exception& e) {
    throw Error(e.what());
  }
}

// ---------------------------------------------------------------

core::ProblemSolverPtr_t Problem::problemSolver() {
  return server_->problemSolver();
}

// ---------------------------------------------------------------

#define _NOOP ((void)0)
#define _CASE(test, op_true, op_false) \
  if (!ok) {                           \
    if (util::iequal(w, test)) {       \
      op_true;                         \
      ok = true;                       \
    } else {                           \
      op_false;                        \
    }                                  \
  }
#define _CASE_SET_RET(name, val_true) \
  _CASE(name, ret = val_true, types.push_back(name))
#define _CASE_PS_MAP(name, map) \
  _CASE_SET_RET(name, problemSolver()->map.getKeys<Ret_t>())

Names_t* Problem::getAvailable(const char* what) {
  std::string w(what);
  typedef std::list<std::string> Ret_t;
  Ret_t ret, types;
  bool ok = false;

  _CASE_PS_MAP("PathOptimizer", pathOptimizers)
  _CASE_PS_MAP("PathProjector", pathProjectors)
  _CASE_PS_MAP("PathPlanner", pathPlanners)
  _CASE_PS_MAP("ConfigurationShooter", configurationShooters)
  _CASE_PS_MAP("PathValidation", pathValidations)
  _CASE_PS_MAP("ConfigValidation", configValidations)
  _CASE_PS_MAP("SteeringMethod", steeringMethods)
  _CASE_PS_MAP("Distance", distances)
  _CASE_PS_MAP("NumericalConstraint", numericalConstraints)
  _CASE_PS_MAP("CenterOfMass", centerOfMassComputations)
  _CASE_SET_RET("Problem", server_->problemSolverMap()->keys<Ret_t>())
  _CASE_SET_RET("Parameter",
                problem(problemSolver())->parameters.getKeys<Ret_t>())
  _CASE_SET_RET(
      "DefaultParameter",
      problem(problemSolver())->parameterDescriptions().getKeys<Ret_t>())
  _CASE_SET_RET("Type", types)

  if (!ok) {
    throw Error(("Type \"" + std::string(what) + "\" not known").c_str());
  }

  return toNames_t(ret.begin(), ret.end());
}

// ---------------------------------------------------------------

#define _CASE_PUSH_TO(name, val) \
  _CASE(name, ret.push_back(val), types.push_back(name))

Names_t* Problem::getSelected(const char* what) {
  std::string w(what);
  typedef std::vector<std::string> Ret_t;
  Ret_t ret, types;
  bool ok = false;
  core::value_type tol;

  _CASE_SET_RET("PathOptimizer",
                Ret_t(problemSolver()->pathOptimizerTypes().begin(),
                      problemSolver()->pathOptimizerTypes().end()))
  _CASE_SET_RET("ConfigValidation",
                Ret_t(problemSolver()->configValidationTypes()))
  _CASE_PUSH_TO("PathProjector", problemSolver()->pathProjectorType(tol))
  _CASE_PUSH_TO("PathPlanner", problemSolver()->pathPlannerType())
  _CASE_PUSH_TO("ConfigurationShooter",
                problemSolver()->configurationShooterType())
  _CASE_PUSH_TO("PathValidation", problemSolver()->pathValidationType(tol))
  _CASE_PUSH_TO("SteeringMethod", problemSolver()->steeringMethodType())
  _CASE_PUSH_TO("Distance", problemSolver()->distanceType())
  _CASE_PUSH_TO("Problem", server_->problemSolverMap()->selectedName())
  _CASE_SET_RET("Type", types)
  if (!ok) {
    throw Error(("Type \"" + std::string(what) + "\" not known").c_str());
  }

  return toNames_t(ret.begin(), ret.end());
}

#undef _CASE
#undef _CASE_SET_RET
#undef _CASE_PUSH_TO
#undef _CASE_PS_MAP
#undef _NOOP

// ---------------------------------------------------------------

void Problem::setParameter(const char* name, const CORBA::Any& value) {
  if (problemSolver()->problem() != NULL) {
    try {
      problemSolver()->problem()->setParameter(std::string(name),
                                               toParameter(value));
    } catch (const std::exception& e) {
      throw hpp::Error(e.what());
    }
    return;
  }
  throw Error("No problem in the ProblemSolver");
}

// ---------------------------------------------------------------

CORBA::Any* Problem::getParameter(const char* name) {
  try {
    core::ProblemPtr_t p(problemSolver()->problem());
    if (p) return toCorbaAnyPtr(p->getParameter(name));
    throw hpp::Error("The problem is not initialized.");
  } catch (std::exception& e) {
    throw Error(e.what());
  }
}

// ---------------------------------------------------------------

char* Problem::getParameterDoc(const char* name) {
  if (problemSolver()->problem() != NULL) {
    try {
      const core::ParameterDescription& desc =
          core::Problem::parameterDescription(name);
      std::stringstream ss;
      ss << desc.doc() << " [Default: ";
      try {
        ss << desc.defaultValue();
      } catch (const std::logic_error&) {
        ss << "None";
      }
      ss << ']';
      return c_str(ss.str());
    } catch (const std::exception& e) {
      throw hpp::Error(e.what());
    }
  }
  throw Error("No problem in the ProblemSolver");
}

// ---------------------------------------------------------------

bool Problem::selectProblem(const char* name) {
  std::string psName(name);
  ProblemSolverMapPtr_t psMap(server_->problemSolverMap());
  bool has = psMap->has(psName);
  if (!has) psMap->add(psName, core::ProblemSolver::create());
  psMap->selected(psName);
  return !has;
}

// ---------------------------------------------------------------

void Problem::resetProblem() {
  ProblemSolverMapPtr_t psMap(server_->problemSolverMap());
  psMap->replaceSelected(core::ProblemSolver::create());
}

// ---------------------------------------------------------------

bool Problem::loadPlugin(const char* pluginName) {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    std::string libname = core::plugin::findPluginLibrary(pluginName);
    return core::plugin::loadPlugin(libname, ps);
  } catch (std::exception& exc) {
    throw Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::movePathToProblem(ULong pathId, const char* name,
                                const Names_t& jointNames) {
  ProblemSolverMapPtr_t psMap(server_->problemSolverMap());
  if (!psMap->has(std::string(name)))
    throw Error("No ProblemSolver of this name");
  core::ProblemSolverPtr_t current = problemSolver(),
                           other = psMap->get(std::string(name));

  if (pathId >= current->paths().size()) {
    std::ostringstream oss("wrong path id: ");
    oss << pathId << ", number path: " << current->paths().size() << ".";
    throw Error(oss.str().c_str());
  }

  core::PathVectorPtr_t pv;
  if (jointNames.length() == 0) {
    pv = current->paths()[pathId];
  } else {
    segments_t cInts, vInts;
    for (CORBA::ULong i = 0; i < jointNames.length(); ++i) {
      const Frame frame =
          problemSolver()->robot()->getFrameByName(std::string(jointNames[i]));
      if (!frame.isFixed()) {
        JointPtr_t joint = frame.joint();
        if (joint == NULL) {
          std::ostringstream oss;
          oss << "Joint " << jointNames[i] << "not found.";
          throw hpp::Error(oss.str().c_str());
        }
        cInts.push_back(
            segment_t(joint->rankInConfiguration(), joint->configSize()));
        vInts.push_back(segment_t(joint->rankInVelocity(), joint->numberDof()));
      }
    }

    Eigen::BlockIndex::sort(cInts);
    Eigen::BlockIndex::sort(vInts);
    Eigen::BlockIndex::shrink(cInts);
    Eigen::BlockIndex::shrink(vInts);

    core::SubchainPathPtr_t nPath =
        core::SubchainPath::create(current->paths()[pathId], cInts, vInts);
    pv = core::PathVector::create(nPath->outputSize(),
                                  nPath->outputDerivativeSize());
    pv->appendPath(nPath);
  }
  other->addPath(pv);
}

// ---------------------------------------------------------------

void Problem::setInitialConfig(const hpp::floatSeq& dofArray) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    ConfigurationPtr_t config = floatSeqToConfigPtr(robot, dofArray, true);
    problemSolver()->initConfig(config);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::floatSeq* Problem::getInitialConfig() {
  hpp::floatSeq* dofArray;

  // Get robot in hppPlanner object.
  ConfigurationPtr_t config = problemSolver()->initConfig();

  if (config) {
    std::size_t deviceDim = config->size();

    dofArray = new hpp::floatSeq();
    dofArray->length((CORBA::ULong)deviceDim);

    for (unsigned int i = 0; i < deviceDim; i++) {
      (*dofArray)[i] = (*config)[i];
    }
    return dofArray;
  } else {
    throw hpp::Error("no initial configuration defined");
  }
}

// ---------------------------------------------------------------

void Problem::addGoalConfig(const hpp::floatSeq& dofArray) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    ConfigurationPtr_t config = floatSeqToConfigPtr(robot, dofArray, true);
    problemSolver()->addGoalConfig(config);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::floatSeqSeq* Problem::getGoalConfigs() {
  try {
    hpp::floatSeqSeq* configSequence;
    const core::Configurations_t goalConfigs(problemSolver()->goalConfigs());
    std::size_t nbGoalConfig = goalConfigs.size();
    configSequence = new hpp::floatSeqSeq();
    configSequence->length((CORBA::ULong)nbGoalConfig);
    for (std::size_t i = 0; i < nbGoalConfig; ++i) {
      const ConfigurationPtr_t& config = goalConfigs[i];
      std::size_t deviceDim = config->size();

      hpp::floatSeq dofArray;
      dofArray.length((CORBA::ULong)deviceDim);

      for (std::size_t j = 0; j < deviceDim; ++j)
        dofArray[(CORBA::ULong)j] = (*config)[j];
      (*configSequence)[(CORBA::ULong)i] = dofArray;
    }
    return configSequence;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::resetGoalConfigs() { problemSolver()->resetGoalConfigs(); }

void Problem::setGoalConstraints(const Names_t& constraints) {
  try {
    NumericalConstraints_t ncs;
    for (CORBA::ULong i = 0; i < constraints.length(); ++i) {
      ImplicitPtr_t c(
          problemSolver()->numericalConstraint(std::string(constraints[i])));
      if (!c) {
        std::ostringstream os;
        os << "Constraint \"" << constraints[i] << "\" not found.";
        throw std::invalid_argument(os.str().c_str());
      }
      ncs.push_back(c);
    }
    problemSolver()->setGoalConstraints(ncs);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Problem::resetGoalConstraints() {
  try {
    problemSolver()->resetGoalConstraints();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------
static int numberOfTrue(const hpp::boolSeq& mask) {
  std::size_t res = 0;
  for (std::size_t i = 0; i < mask.length(); ++i)
    if (mask[(CORBA::ULong)i]) ++res;
  return (int)res;
}
// ---------------------------------------------------------------

void Problem::createOrientationConstraint(const char* constraintName,
                                          const char* joint1Name,
                                          const char* joint2Name,
                                          const Double* p,
                                          const hpp::boolSeq& mask) {
  try {
    if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
    Transform3f::Quaternion quat(p[3], p[0], p[1], p[2]);
    Transform3f rotation(quat.matrix(), vector3_t::Zero());
    std::string name(constraintName);

    DifferentiableFunctionPtr_t func =
        buildGenericFunc<constraints::OrientationBit>(
            problemSolver()->robot(), name, joint1Name, joint2Name, Id,
            rotation, boolSeqToVector(mask));

    problemSolver()->addNumericalConstraint(
        name,
        Implicit::create(func, numberOfTrue(mask) * constraints::EqualToZero));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createTransformationConstraint(const char* constraintName,
                                             const char* joint1Name,
                                             const char* joint2Name,
                                             const Transform_ p,
                                             const hpp::boolSeq& mask) {
  try {
    if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
    Transform3f ref(toTransform3f(p));
    std::string name(constraintName);
    DifferentiableFunctionPtr_t func =
        buildGenericFunc<constraints::PositionBit |
                         constraints::OrientationBit>(
            problemSolver()->robot(), name, joint1Name, joint2Name, ref, Id,
            boolSeqToVector(mask, 6));

    problemSolver()->addNumericalConstraint(
        name,
        Implicit::create(func, numberOfTrue(mask) * constraints::EqualToZero));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createTransformationConstraint2(const char* constraintName,
                                              const char* joint1Name,
                                              const char* joint2Name,
                                              const Transform_ frame1,
                                              const Transform_ frame2,
                                              const hpp::boolSeq& mask) {
  try {
    if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
    std::string name(constraintName);
    DifferentiableFunctionPtr_t func =
        buildGenericFunc<constraints::PositionBit |
                         constraints::OrientationBit>(
            problemSolver()->robot(), name, joint1Name, joint2Name,
            toTransform3f(frame1), toTransform3f(frame2),
            boolSeqToVector(mask, 6));

    problemSolver()->addNumericalConstraint(
        name,
        Implicit::create(func, numberOfTrue(mask) * constraints::EqualToZero));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createTransformationR3xSO3Constraint(const char* constraintName,
                                                   const char* joint1Name,
                                                   const char* joint2Name,
                                                   const Transform_ frame1,
                                                   const Transform_ frame2,
                                                   const hpp::boolSeq& mask) {
  try {
    if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
    std::string name(constraintName);
    DifferentiableFunctionPtr_t func =
        buildGenericFunc<constraints::OutputR3xSO3Bit |
                         constraints::PositionBit |
                         constraints::OrientationBit>(
            problemSolver()->robot(), name, joint1Name, joint2Name,
            toTransform3f(frame1), toTransform3f(frame2),
            boolSeqToVector(mask, 6));

    problemSolver()->addNumericalConstraint(
        name,
        Implicit::create(func, numberOfTrue(mask) * constraints::EqualToZero));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createLockedJoint(const char* lockedJointName,
                                const char* jointName,
                                const hpp::floatSeq& value) {
  try {
    // Get robot in hppPlanner object.
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    JointPtr_t joint = robot->getJointByName(jointName);
    vector_t config = floatSeqToVector(value, joint->configSize());
    hppDout(info, "joint->configurationSpace ()->name () = "
                      << joint->configurationSpace()->name());
    hppDout(info, "joint->configurationSpace ()->nq () = "
                      << joint->configurationSpace()->nq());
    LiegroupElement lge(config, joint->configurationSpace());
    LockedJointPtr_t lockedJoint(LockedJoint::create(joint, lge));
    problemSolver()->numericalConstraints.add(lockedJointName, lockedJoint);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createLockedJointWithComp(const char* lockedJointName,
                                        const char* jointName,
                                        const hpp::floatSeq& value,
                                        const hpp::ComparisonTypes_t& comp) {
  try {
    // Get robot in hppPlanner object.
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    JointPtr_t joint = robot->getJointByName(jointName);
    vector_t config = floatSeqToVector(value, joint->configSize());
    hppDout(info, "joint->configurationSpace ()->name () = "
                      << joint->configurationSpace()->name());
    hppDout(info, "joint->configurationSpace ()->nq () = "
                      << joint->configurationSpace()->nq());
    LiegroupElement lge(config, joint->configurationSpace());
    LockedJointPtr_t lockedJoint(LockedJoint::create(joint, lge));
    lockedJoint->comparisonType(convertComparison(comp));
    problemSolver()->numericalConstraints.add(lockedJointName, lockedJoint);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createLockedExtraDof(const char* lockedDofName,
                                   const CORBA::ULong index,
                                   const hpp::floatSeq& value) {
  try {
    // Get robot in hppPlanner object.
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    vector_t config = floatSeqToVector(value);

    LockedJointPtr_t lockedJoint(LockedJoint::create(robot, index, config));
    problemSolver()->numericalConstraints.add(lockedDofName, lockedJoint);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createManipulability(const char* _name, const char* _function) {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    DevicePtr_t robot = getRobotOrThrow(ps);

    std::string function(_function), name(_name);
    if (!ps->numericalConstraints.has(function))
      throw Error(("Constraint " + function + " not found").c_str());
    DifferentiableFunctionPtr_t f(
        ps->numericalConstraints.get(function)->functionPtr());

    ps->addNumericalConstraint(
        name,
        Implicit::create(constraints::Manipulability::create(f, robot, name),
                         1 * constraints::EqualToZero));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createRelativeComConstraint(const char* constraintName,
                                          const char* comName,
                                          const char* jointName,
                                          const floatSeq& p,
                                          const hpp::boolSeq& mask) {
  if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
  JointPtr_t joint;
  CenterOfMassComputationPtr_t comc;
  vector3_t point = floatSeqToVector3(p);

  std::vector<bool> m = boolSeqToVector(mask, 3);
  try {
    joint = problemSolver()->robot()->getJointByName(jointName);
    // Test whether joint1 is world frame
    std::string name(constraintName), comN(comName);
    if (comN.compare("") == 0) {
      problemSolver()->addNumericalConstraint(
          name,
          Implicit::create(RelativeCom::create(name, problemSolver()->robot(),
                                               joint, point, m),
                           numberOfTrue(mask) * constraints::EqualToZero));
    } else {
      if (!problemSolver()->centerOfMassComputations.has(comN))
        throw hpp::Error("Partial COM not found.");
      comc = problemSolver()->centerOfMassComputations.get(comN);
      problemSolver()->addNumericalConstraint(
          name,
          Implicit::create(RelativeCom::create(name, problemSolver()->robot(),
                                               comc, joint, point, m),
                           numberOfTrue(mask) * constraints::EqualToZero));
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createComBeetweenFeet(
    const char* constraintName, const char* comName, const char* jointLName,
    const char* jointRName, const floatSeq& pL, const floatSeq& pR,
    const char* jointRefName, const floatSeq& pRef, const hpp::boolSeq& mask) {
  if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
  JointPtr_t jointL, jointR, jointRef;
  CenterOfMassComputationPtr_t comc;
  vector3_t pointL = floatSeqToVector3(pL);
  vector3_t pointR = floatSeqToVector3(pR);
  vector3_t pointRef = floatSeqToVector3(pRef);

  std::vector<bool> m = boolSeqToVector(mask);
  try {
    jointL = problemSolver()->robot()->getJointByName(jointLName);
    jointR = problemSolver()->robot()->getJointByName(jointRName);
    // Test whether joint1 is world frame
    if (std::string(jointRefName) == std::string(""))
      jointRef = problemSolver()->robot()->rootJoint();
    else
      jointRef = problemSolver()->robot()->getJointByName(jointRefName);
    std::string name(constraintName), comN(comName);

    constraints::ComparisonTypes_t comps(2 * constraints::EqualToZero
                                         << constraints::Superior
                                         << constraints::Inferior);

    if (comN.compare("") == 0) {
      problemSolver()->addNumericalConstraint(
          name,
          Implicit::create(ComBetweenFeet::create(
                               name, problemSolver()->robot(), jointL, jointR,
                               pointL, pointR, jointRef, pointRef, m),
                           comps));
    } else {
      if (!problemSolver()->centerOfMassComputations.has(comN))
        throw hpp::Error("Partial COM not found.");
      comc = problemSolver()->centerOfMassComputations.get(comN);
      problemSolver()->addNumericalConstraint(
          name,
          Implicit::create(ComBetweenFeet::create(
                               name, problemSolver()->robot(), comc, jointL,
                               jointR, pointL, pointR, jointRef, pointRef, m),
                           comps));
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createConvexShapeContactConstraint(
    const char* constraintName, const Names_t& floorJoints,
    const Names_t& objectJoints, const hpp::floatSeqSeq& points,
    const hpp::intSeqSeq& objTriangles, const hpp::intSeqSeq& floorTriangles) {
  if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
  try {
    JointAndShapes_t floorSurfaces, objectSurfaces;
    std::vector<fcl::Vec3f> pts(points.length());
    for (CORBA::ULong i = 0; i < points.length(); ++i) {
      if (points[i].length() != 3)
        throw hpp::Error("Points must be of size 3.");
      pts[i] = fcl::Vec3f(points[i][0], points[i][1], points[i][2]);
    }
    if (objectJoints.length() != objTriangles.length()) {
      std::ostringstream oss;
      oss << "Number of object joints (" << objectJoints.length()
          << ") should fit number of objTriangles (" << objTriangles.length()
          << ").";
      throw std::runtime_error(oss.str());
    }
    for (CORBA::ULong i = 0; i < objTriangles.length(); ++i) {
      if (objTriangles[i].length() != 3)
        throw hpp::Error("Triangle must have size 3.");

      for (size_t j = 0; j < 3; j++)
        if (objTriangles[i][(CORBA::ULong)j] < 0 &&
            (size_t)objTriangles[i][(CORBA::ULong)j] >= pts.size())
          throw hpp::Error("Point index out of range.");

      std::string jointName(objectJoints[i]);
      JointPtr_t joint;
      if (jointName == "None") {
        // joint = 0x0;
      } else {
        joint = problemSolver()->robot()->getJointByName(jointName);
      }
      std::vector<core::vector3_t> shapePts{pts[objTriangles[i][0]],
                                            pts[objTriangles[i][1]],
                                            pts[objTriangles[i][2]]};
      objectSurfaces.push_back(JointAndShape_t(joint, shapePts));
    }
    if (floorJoints.length() != floorTriangles.length()) {
      std::ostringstream oss;
      oss << "Number of floor joints (" << floorJoints.length()
          << ") should fit number of floorTriangles ("
          << floorTriangles.length() << ").";
      throw std::runtime_error(oss.str());
    }
    for (CORBA::ULong i = 0; i < floorTriangles.length(); ++i) {
      if (floorTriangles[i].length() != 3)
        throw hpp::Error("Triangle must have size 3.");
      for (size_t j = 0; j < 3; j++)
        if (floorTriangles[i][(CORBA::ULong)j] < 0 &&
            (size_t)floorTriangles[i][(CORBA::ULong)j] >= pts.size())
          throw hpp::Error("Point index out of range.");

      std::string jointName(floorJoints[i]);
      JointPtr_t joint;
      if (jointName == "None") {
        // joint = 0x0;
      } else {
        joint = problemSolver()->robot()->getJointByName(jointName);
      }
      std::vector<core::vector3_t> shapePts{pts[floorTriangles[i][0]],
                                            pts[floorTriangles[i][1]],
                                            pts[floorTriangles[i][2]]};
      floorSurfaces.push_back(JointAndShape_t(joint, shapePts));
    }
    std::string name(constraintName);
    ConvexShapeContactPtr_t f = ConvexShapeContact::create(
        name, problemSolver()->robot(), floorSurfaces, objectSurfaces);
    problemSolver()->addNumericalConstraint(
        name, Implicit::create(f, 5 * constraints::EqualToZero));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createPositionConstraint(const char* constraintName,
                                       const char* joint1Name,
                                       const char* joint2Name,
                                       const hpp::floatSeq& point1,
                                       const hpp::floatSeq& point2,
                                       const hpp::boolSeq& mask) {
  try {
    if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
    vector3_t p1 = floatSeqToVector3(point1);
    vector3_t p2 = floatSeqToVector3(point2);
    std::string name(constraintName);

    DifferentiableFunctionPtr_t func =
        buildGenericFunc<constraints::PositionBit>(
            problemSolver()->robot(), name, joint1Name, joint2Name,
            Transform3f(I3, p1), Transform3f(I3, p2), boolSeqToVector(mask));

    problemSolver()->addNumericalConstraint(
        name,
        Implicit::create(func, numberOfTrue(mask) * constraints::EqualToZero));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createConfigurationConstraint(const char* constraintName,
                                            const hpp::floatSeq& goal,
                                            const hpp::floatSeq& weights) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    std::string name(constraintName);
    problemSolver()->numericalConstraints.add(
        name,
        Implicit::create(constraints::ConfigurationConstraint::create(
                             name, problemSolver()->robot(),
                             floatSeqToConfig(robot, goal, true),
                             floatSeqToVector(weights, robot->numberDof())),
                         1 * constraints::EqualToZero));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createDistanceBetweenJointConstraint(const char* constraintName,
                                                   const char* joint1Name,
                                                   const char* joint2Name,
                                                   Double) {
  if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
  try {
    JointPtr_t joint1 = problemSolver()->robot()->getJointByName(joint1Name);
    JointPtr_t joint2 = problemSolver()->robot()->getJointByName(joint2Name);
    std::string name(constraintName);
    problemSolver()->addNumericalConstraint(
        name,
        Implicit::create(DistanceBetweenBodies::create(
                             name, problemSolver()->robot(), joint1, joint2),
                         1 * constraints::EqualToZero));
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createDistanceBetweenJointAndObjects(const char* constraintName,
                                                   const char* joint1Name,
                                                   const hpp::Names_t& objects,
                                                   Double) {
  if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
  try {
    JointPtr_t joint1 = problemSolver()->robot()->getJointByName(joint1Name);
    std::vector<CollisionObjectPtr_t> objectList;
    for (CORBA::ULong i = 0; i < objects.length(); ++i) {
      objectList.push_back(problemSolver()->obstacle(std::string(objects[i])));
    }
    std::string name(constraintName);
    problemSolver()->addNumericalConstraint(
        name, Implicit::create(
                  DistanceBetweenBodies::create(name, problemSolver()->robot(),
                                                joint1, objectList),
                  1 * constraints::EqualToZero));
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::createIdentityConstraint(const char* constraintName,
                                       const Names_t& inJoints,
                                       const hpp::Names_t& outJoints) {
  try {
    using namespace constraints;

    DevicePtr_t robot = problemSolver()->robot();

    LiegroupSpacePtr_t ispace = LiegroupSpace::empty();
    LiegroupSpacePtr_t ospace = LiegroupSpace::empty();
    segments_t iq, oq, iv, ov;
    for (CORBA::ULong i = 0; i < inJoints.length(); ++i) {
      JointPtr_t j = robot->getJointByName(std::string(inJoints[i]));
      *ispace *= j->configurationSpace();
      iq.push_back(segment_t(j->rankInConfiguration(), j->configSize()));
      iv.push_back(segment_t(j->rankInVelocity(), j->numberDof()));
    }
    for (CORBA::ULong i = 0; i < outJoints.length(); ++i) {
      JointPtr_t j = robot->getJointByName(std::string(outJoints[i]));
      *ospace *= j->configurationSpace();
      oq.push_back(segment_t(j->rankInConfiguration(), j->configSize()));
      ov.push_back(segment_t(j->rankInVelocity(), j->numberDof()));
    }
    if (*ispace != *ospace) throw Error("Input and output space are different");
    assert(BlockIndex::cardinal(iq) == BlockIndex::cardinal(oq));
    assert(BlockIndex::cardinal(iv) == BlockIndex::cardinal(ov));

    problemSolver()->addNumericalConstraint(
        constraintName,
        Explicit::create(robot->configSpace(),
                         Identity::create(ispace, constraintName), iq, oq, iv,
                         ov));
  } catch (std::exception& exc) {
    throw Error(exc.what());
  }
}

// ---------------------------------------------------------------

bool Problem::applyConstraints(const hpp::floatSeq& input,
                               hpp::floatSeq_out output,
                               double& residualError) {
  bool success = false;
  DevicePtr_t robot = getRobotOrThrow(problemSolver());
  try {
    ConfigurationPtr_t config = floatSeqToConfigPtr(robot, input, true);
    success = problemSolver()->constraints()->apply(*config);
    if (hpp::core::ConfigProjectorPtr_t configProjector =
            problemSolver()->constraints()->configProjector()) {
      residualError = configProjector->residualError();
    }
    output = vectorToFloatSeq(*config);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
  return success;
}

// ---------------------------------------------------------------

bool Problem::optimize(const hpp::floatSeq& input, hpp::floatSeq_out output,
                       hpp::floatSeq_out residualError) {
  bool success = false;
  DevicePtr_t robot = getRobotOrThrow(problemSolver());
  Configuration_t config = floatSeqToConfig(robot, input, true);
  vector_t err;
  try {
    if (!problemSolver()->constraints())
      throw hpp::Error("The problem has no constraints");
    if (!problemSolver()->constraints()->configProjector())
      throw hpp::Error("The problem has no config projector");
    core::ConfigProjectorPtr_t cp =
        problemSolver()->constraints()->configProjector();

    success = cp->optimize(config, 0);
    cp->isSatisfied(config, err);

  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
  output = vectorToFloatSeq(config);
  residualError = vectorToFloatSeq(err);
  return success;
}

// ---------------------------------------------------------------

void Problem::computeValueAndJacobian(const hpp::floatSeq& config,
                                      hpp::floatSeq_out value,
                                      hpp::floatSeqSeq_out jacobian) {
  DevicePtr_t robot = getRobotOrThrow(problemSolver());
  try {
    ConfigurationPtr_t configuration = floatSeqToConfigPtr(robot, config, true);
    vector_t v;
    matrix_t J;
    problemSolver()->computeValueAndJacobian(*configuration, v, J);
    value = vectorToFloatSeq(v);
    jacobian = matrixToFloatSeqSeq(J);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

bool Problem::generateValidConfig(ULong maxIter, hpp::floatSeq_out output,
                                  double& residualError) {
  core::ProblemSolverPtr_t ps = problemSolver();
  DevicePtr_t robot = getRobotOrThrow(ps);
  if (!ps->problem()) ps->resetProblem();
  core::ConfigurationShooterPtr_t shooter =
      ps->problem()->configurationShooter();
  bool success = false, configIsValid = false;
  ConfigurationPtr_t config;
  while (!configIsValid && maxIter > 0) {
    try {
      config = shooter->shoot();
      success = ps->constraints()->apply(*config);
      if (hpp::core::ConfigProjectorPtr_t configProjector =
              ps->constraints()->configProjector()) {
        residualError = configProjector->residualError();
      }
      if (success) {
        core::ValidationReportPtr_t validationReport;
        configIsValid = ps->problem()->configValidations()->validate(
            *config, validationReport);
      }
    } catch (const std::exception& exc) {
      throw hpp::Error(exc.what());
    }
    maxIter--;
  }
  output = vectorToFloatSeq(*config);
  return configIsValid;
}

// ---------------------------------------------------------------

void Problem::resetConstraintMap() {
  try {
    problemSolver()->numericalConstraints.clear();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::resetConstraints() {
  if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
  try {
    problemSolver()->resetConstraints();
    problemSolver()->robot()->controlComputation(pinocchio::JOINT_POSITION);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::filterCollisionPairs() {
  try {
    problemSolver()->filterCollisionPairs();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::addPassiveDofs(const char* passiveDofsName,
                             const hpp::Names_t& dofNames) {
  DevicePtr_t robot = problemSolver()->robot();
  if (!robot) throw hpp::Error("No robot loaded");
  std::vector<size_type> dofs;
  /// First, translate names into velocity indexes.
  for (CORBA::ULong i = 0; i < dofNames.length(); ++i) {
    std::string name(dofNames[i]);
    JointPtr_t j = robot->getJointByName(name);
    if (!j) {
      std::ostringstream oss;
      oss << "Joint " << name << " not found.";
      throw hpp::Error(oss.str().c_str());
    }
    for (size_type i = 0; i < j->numberDof(); i++)
      dofs.push_back(j->rankInVelocity() + i);
  }
  /// Then, sort and remove non duplicated elements.
  std::sort(dofs.begin(), dofs.end());
  std::vector<size_type>::iterator last = std::unique(dofs.begin(), dofs.end());
  dofs.erase(last, dofs.end());
  /// Then, create the intervals.
  core::segments_t passiveDofs;
  dofs.push_back(robot->numberDof() + 1);
  size_type intStart = dofs[0], intEnd = dofs[0];
  for (size_t i = 1; i < dofs.size(); i++) {
    intEnd++;
    if (intEnd == dofs[i]) {
      continue;
    } else {
      passiveDofs.push_back(core::segment_t(intStart, intEnd - intStart));
      intStart = intEnd = dofs[i];
    }
  }
  problemSolver()->passiveDofs.add(passiveDofsName, passiveDofs);
}

// ---------------------------------------------------------------

void Problem::getConstraintDimensions(const char* constraintName,
                                      ULong& inputSize,
                                      ULong& inputDerivativeSize,
                                      ULong& outputSize,
                                      ULong& outputDerivativeSize) {
  try {
    std::string n(constraintName);
    if (!problemSolver()->numericalConstraints.has(n))
      throw Error(("Constraint " + n + " not found").c_str());
    ImplicitPtr_t c = problemSolver()->numericalConstraints.get(n);
    inputSize = (ULong)c->function().inputSize();
    inputDerivativeSize = (ULong)c->function().inputDerivativeSize();
    outputSize = (ULong)c->function().outputSize();
    outputDerivativeSize = (ULong)c->function().outputDerivativeSize();
  } catch (const std::exception& e) {
    throw hpp::Error(e.what());
  }
}

// ---------------------------------------------------------------

void Problem::setConstantRightHandSide(const char* constraintName,
                                       CORBA::Boolean constant) {
  try {
    if (constant) {
      problemSolver()->comparisonType(constraintName, constraints::EqualToZero);
    } else {
      problemSolver()->comparisonType(constraintName, constraints::Equality);
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

bool Problem::getConstantRightHandSide(const char* constraintName) {
  try {
    ImplicitPtr_t nc(problemSolver()->numericalConstraints.get(constraintName));
    return nc->parameterSize() == 0;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::setRightHandSide(const hpp::floatSeq& rhs) {
  try {
    hpp::core::ConfigProjectorPtr_t configProjector(
        problemSolver()->constraints()->configProjector());
    if (!configProjector) {
      throw std::runtime_error("No constraint has been set.");
    }
    vector_t rightHandSide(floatSeqToVector(rhs));
    configProjector->rightHandSide(rightHandSide);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::setRightHandSideFromConfig(const hpp::floatSeq& config) {
  try {
    hpp::core::ConfigProjectorPtr_t configProjector(
        problemSolver()->constraints()->configProjector());
    if (!configProjector) {
      throw std::runtime_error("No constraint has been set.");
    }
    Configuration_t q(floatSeqToConfig(problemSolver()->robot(), config, true));
    configProjector->rightHandSideFromConfig(q);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::setRightHandSideFromConfigByName(const char* constraintName,
                                               const hpp::floatSeq& config) {
  try {
    hpp::core::ConfigProjectorPtr_t configProjector(
        problemSolver()->constraints()->configProjector());
    if (!configProjector) {
      throw std::runtime_error("No constraint has been set.");
    }
    Configuration_t q(floatSeqToConfig(problemSolver()->robot(), config, true));
    // look for constraint with this key
    if (problemSolver()->numericalConstraints.has(constraintName)) {
      ImplicitPtr_t nc(
          problemSolver()->numericalConstraints.get(constraintName));
      configProjector->rightHandSideFromConfig(nc, q);
      return;
    }
    std::string msg(
        "Config projector does not contain any numerical"
        " constraint with name ");
    msg += constraintName;
    throw std::runtime_error(msg.c_str());
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::setRightHandSideByName(const char* constraintName,
                                     const hpp::floatSeq& rhs) {
  try {
    hpp::core::ConfigProjectorPtr_t configProjector(
        problemSolver()->constraints()->configProjector());
    if (!configProjector) {
      throw std::runtime_error("No constraint has been set.");
    }
    vector_t rightHandSide(floatSeqToVector(rhs));
    // look for constraint with this key
    if (problemSolver()->numericalConstraints.has(constraintName)) {
      ImplicitPtr_t nc(
          problemSolver()->numericalConstraints.get(constraintName));
      configProjector->rightHandSide(nc, rightHandSide);
      return;
    }
    std::string msg(
        "Config projector does not contain any numerical"
        " constraint with name ");
    msg += constraintName;
    throw std::runtime_error(msg.c_str());
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::floatSeq* Problem::getRightHandSide() {
  try {
    hpp::core::ConfigProjectorPtr_t configProjector(
        problemSolver()->constraints()->configProjector());
    if (!configProjector) {
      throw std::runtime_error("No constraint has been set.");
    }
    return vectorToFloatSeq(configProjector->rightHandSide());
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::addNumericalConstraints(const char* configProjName,
                                      const Names_t& constraintNames,
                                      const hpp::intSeq& priorities) {
  if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
  try {
    for (CORBA::ULong i = 0; i < constraintNames.length(); ++i) {
      std::string name(constraintNames[i]);
      problemSolver()->addNumericalConstraintToConfigProjector(
          configProjName, name, (std::size_t)priorities[i]);
      problemSolver()->robot()->controlComputation(pinocchio::COMPUTE_ALL);
    }
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::setNumericalConstraintsLastPriorityOptional(const bool optional) {
  if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
  if (!problemSolver()->constraints())
    throw hpp::Error("The problem has no constraints");
  if (!problemSolver()->constraints()->configProjector())
    throw hpp::Error("The problem has no config projector");
  try {
    problemSolver()->constraints()->configProjector()->lastIsOptional(optional);
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::addLockedJointConstraints(const char* configProjName,
                                        const hpp::Names_t& lockedJointNames) {
  if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
  try {
    for (CORBA::ULong i = 0; i < lockedJointNames.length(); ++i) {
      std::string name(lockedJointNames[i]);
      problemSolver()->addNumericalConstraintToConfigProjector(configProjName,
                                                               name);
      problemSolver()->robot()->controlComputation(pinocchio::COMPUTE_ALL);
    }
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

// ---------------------------------------------------------------

char* Problem::displayConstraints() {
  try {
    std::ostringstream oss;
    if (problemSolver()->constraints()) oss << *problemSolver()->constraints();
    return c_str(oss.str());
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

Double Problem::getErrorThreshold() {
  return problemSolver()->errorThreshold();
}

// ---------------------------------------------------------------

typedef std::map<std::string, core::ConfigProjector::LineSearchType>
    LineSearchTypeMap_t;

LineSearchTypeMap_t makeLineSearchTypeMap() {
  LineSearchTypeMap_t map;
  map["Backtracking"] = core::ConfigProjector::Backtracking;
  map["FixedSequence"] = core::ConfigProjector::FixedSequence;
  map["Constant"] = core::ConfigProjector::Constant;
  map["ErrorNormBased"] = core::ConfigProjector::ErrorNormBased;
  return map;
}
void Problem::setDefaultLineSearchType(const char* type) {
  static const LineSearchTypeMap_t map = makeLineSearchTypeMap();
  std::string t(type);

  LineSearchTypeMap_t::const_iterator _t = map.find(t);
  if (_t != map.end())
    core::ConfigProjector::defaultLineSearch(_t->second);
  else {
    std::ostringstream oss;
    oss << "Available types are:";
    for (_t = map.begin(); _t != map.end(); ++_t)
      oss << " \"" << _t->first << '"';
    throw Error(oss.str().c_str());
  }
}

// ---------------------------------------------------------------

void Problem::setErrorThreshold(Double threshold) {
  problemSolver()->errorThreshold(threshold);
}

// ---------------------------------------------------------------
void Problem::setMaxIterProjection(ULong iterations) {
  problemSolver()->maxIterProjection((size_type)iterations);
}

// ---------------------------------------------------------------
ULong Problem::getMaxIterProjection() {
  return (ULong)problemSolver()->maxIterProjection();
}

// ---------------------------------------------------------------
void Problem::setMaxIterPathPlanning(ULong iterations) {
  problemSolver()->maxIterPathPlanning((size_type)iterations);
}

// ---------------------------------------------------------------
ULong Problem::getMaxIterPathPlanning() {
  return (ULong)problemSolver()->maxIterPathPlanning();
}

// ---------------------------------------------------------------
void Problem::setTimeOutPathPlanning(double timeOut) {
  problemSolver()->setTimeOutPathPlanning(timeOut);
}

// ---------------------------------------------------------------
double Problem::getTimeOutPathPlanning() {
  return problemSolver()->getTimeOutPathPlanning();
}

// ---------------------------------------------------------------

void Problem::selectPathPlanner(const char* pathPlannerType) {
  try {
    problemSolver()->pathPlannerType(std::string(pathPlannerType));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::selectConfigurationShooter(const char* configurationShooterType) {
  try {
    problemSolver()->configurationShooterType(
        std::string(configurationShooterType));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::selectDistance(const char* distanceType) {
  try {
    problemSolver()->distanceType(std::string(distanceType));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::selectSteeringMethod(const char* steeringMethodType) {
  try {
    problemSolver()->steeringMethodType(std::string(steeringMethodType));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------
void Problem::addPathOptimizer(const char* pathOptimizerType) {
  try {
    problemSolver()->addPathOptimizer(std::string(pathOptimizerType));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::clearPathOptimizers() {
  try {
    problemSolver()->clearPathOptimizers();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------
void Problem::addConfigValidation(const char* configValidationType) {
  try {
    problemSolver()->addConfigValidation(std::string(configValidationType));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::clearConfigValidations() {
  try {
    problemSolver()->clearConfigValidations();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::selectPathValidation(const char* pathValidationType,
                                   Double tolerance) {
  try {
    problemSolver()->pathValidationType(std::string(pathValidationType),
                                        tolerance);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::selectPathProjector(const char* pathProjectorType,
                                  Double tolerance) {
  try {
    problemSolver()->pathProjectorType(std::string(pathProjectorType),
                                       tolerance);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

bool Problem::prepareSolveStepByStep() {
  try {
    return problemSolver()->prepareSolveStepByStep();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
  return false;
}

// ---------------------------------------------------------------

bool Problem::executeOneStep() {
  try {
    return problemSolver()->executeOneStep();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
  return false;
}

// ---------------------------------------------------------------

void Problem::finishSolveStepByStep() {
  try {
    problemSolver()->finishSolveStepByStep();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::intSeq* Problem::solve() {
  try {
    boost::posix_time::ptime start =
        boost::posix_time::microsec_clock::universal_time();
    problemSolver()->solve();
    boost::posix_time::time_duration time =
        boost::posix_time::microsec_clock::universal_time() - start;
    hpp::intSeq* ret = new hpp::intSeq;
    ret->length(4);
    (*ret)[0] = static_cast<CORBA::Long>(time.hours());
    (*ret)[1] = static_cast<CORBA::Long>(time.minutes());
    (*ret)[2] = static_cast<CORBA::Long>(time.seconds());
    (*ret)[3] = static_cast<CORBA::Long>(time.fractional_seconds() / 1000);
    return ret;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

bool Problem::directPath(const hpp::floatSeq& startConfig,
                         const hpp::floatSeq& endConfig,
                         CORBA::Boolean validate, ULong& pathId,
                         CORBA::String_out report) {
  ConfigurationPtr_t start;
  ConfigurationPtr_t end;
  bool pathValid = false;
  DevicePtr_t robot = getRobotOrThrow(problemSolver());
  try {
    start = floatSeqToConfigPtr(robot, startConfig, true);
    end = floatSeqToConfigPtr(robot, endConfig, true);
    if (!problemSolver()->problem()) {
      problemSolver()->resetProblem();
    }
    std::size_t pid;
    std::string r;
    pathValid = problemSolver()->directPath(*start, *end, validate, pid, r);
    report = CORBA::string_dup(r.c_str());
    pathId = (ULong)pid;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
  return pathValid;
}

// ---------------------------------------------------------------

bool Problem::reversePath(ULong pathId, ULong& reversedPathId) {
  try {
    if (pathId >= problemSolver()->paths().size()) {
      std::ostringstream oss("wrong path id. ");
      oss << "Number path: " << problemSolver()->paths().size() << ".";
      std::string err = oss.str();
      throw hpp::Error(err.c_str());
    }
    PathVectorPtr_t path = problemSolver()->paths()[pathId];
    PathVectorPtr_t reversed =
        HPP_DYNAMIC_PTR_CAST(core::PathVector, path->reverse());
    if (!reversed) return false;
    problemSolver()->addPath(reversed);
    reversedPathId = (ULong)problemSolver()->paths().size() - 1;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
  return true;
}

// ---------------------------------------------------------------

void Problem::addConfigToRoadmap(const hpp::floatSeq& config) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    ConfigurationPtr_t configuration(floatSeqToConfigPtr(robot, config, true));
    problemSolver()->addConfigToRoadmap(configuration);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::addEdgeToRoadmap(const hpp::floatSeq& config1,
                               const hpp::floatSeq& config2, ULong pathId,
                               bool bothEdges) {
  try {
    if (pathId >= problemSolver()->paths().size()) {
      std::ostringstream oss("wrong path id: ");
      oss << pathId << ", number path: " << problemSolver()->paths().size()
          << ".";
      throw std::runtime_error(oss.str());
    }
    PathVectorPtr_t path = problemSolver()->paths()[pathId];
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    ConfigurationPtr_t start(floatSeqToConfigPtr(robot, config1, true));
    ConfigurationPtr_t finish(floatSeqToConfigPtr(robot, config2, true));
    if (bothEdges) {
      problemSolver()->addEdgeToRoadmap(start, finish, path);
      problemSolver()->addEdgeToRoadmap(finish, start, path->reverse());
      return;
    }
    problemSolver()->addEdgeToRoadmap(start, finish, path);
    return;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}
// ---------------------------------------------------------------

void Problem::appendDirectPath(ULong pathId, const hpp::floatSeq& config,
                               Boolean validate) {
  try {
    if (pathId >= problemSolver()->paths().size()) {
      std::ostringstream oss("wrong path id: ");
      oss << pathId << ", number path: " << problemSolver()->paths().size()
          << ".";
      throw std::runtime_error(oss.str());
    }
    PathVectorPtr_t path = problemSolver()->paths()[pathId];
    Configuration_t start(path->end());
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    ConfigurationPtr_t end(floatSeqToConfigPtr(robot, config, true));
    if (!problemSolver()->problem()) {
      problemSolver()->resetProblem();
    }
    SteeringMethodPtr_t sm(problemSolver()->problem()->steeringMethod());
    PathPtr_t dp = (*sm)(start, *end);
    if (!dp) {
      std::ostringstream oss;
      oss << "steering method failed to build a path between q1="
          << pinocchio::displayConfig(start)
          << "; q2=" << pinocchio::displayConfig(*end) << ".";
      throw std::runtime_error(oss.str().c_str());
    }
    if (validate) {
      PathPtr_t unused;
      PathValidationReportPtr_t report;
      if (!problemSolver()->problem()->pathValidation()->validate(
              dp, false, unused, report)) {
        std::ostringstream oss;
        oss << *report;
        throw hpp::Error(oss.str().c_str());
      }
    }
    path->appendPath(dp);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::concatenatePath(ULong startId, ULong endId) {
  try {
    if (startId >= problemSolver()->paths().size() ||
        endId >= problemSolver()->paths().size()) {
      std::ostringstream oss("wrong path id. ");
      oss << "Number path: " << problemSolver()->paths().size() << ".";
      throw std::runtime_error(oss.str());
    }
    PathVectorPtr_t start = problemSolver()->paths()[startId];
    PathVectorPtr_t end = problemSolver()->paths()[endId];
    start->concatenate(end);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::extractPath(ULong pathId, Double start, Double end) {
  try {
    if (pathId >= problemSolver()->paths().size()) {
      std::ostringstream oss("wrong path id. ");
      oss << "Number path: " << problemSolver()->paths().size() << ".";
      throw std::runtime_error(oss.str());
    }
    PathVectorPtr_t initPath = problemSolver()->paths()[pathId];
    if ((start < initPath->timeRange().first) ||
        end > initPath->timeRange().second) {
      std::ostringstream oss("Parameters out of bounds. ");
      oss << "For pathId =  " << pathId << ", lenght  = ["
          << initPath->timeRange().first << " ; "
          << initPath->timeRange().second << "].";
      throw std::runtime_error(oss.str());
    }
    core::PathPtr_t path = initPath->extract(core::interval_t(start, end));
    PathVectorPtr_t pathVector = dynamic_pointer_cast<core::PathVector>(path);
    if (pathVector)
      problemSolver()->addPath(pathVector);
    else {
      std::ostringstream oss("Error during dynamic-cast in extractPath ");
      throw std::runtime_error(oss.str());
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::erasePath(ULong pathId) {
  try {
    if (pathId >= problemSolver()->paths().size()) {
      std::ostringstream oss("wrong path id. ");
      oss << "Number path: " << problemSolver()->paths().size() << ".";
      throw std::runtime_error(oss.str());
    }
    problemSolver()->erasePath(pathId);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

bool Problem::projectPath(ULong pathId) {
  try {
    if (pathId >= problemSolver()->paths().size()) {
      std::ostringstream oss("wrong path id: ");
      oss << pathId << ", number path: " << problemSolver()->paths().size()
          << ".";
      throw std::runtime_error(oss.str());
    }
    PathVectorPtr_t initial = problemSolver()->paths()[pathId];
    core::PathProjectorPtr_t pp = problemSolver()->problem()->pathProjector();
    if (!pp) throw Error("There is no path projector");

    PathPtr_t proj;
    bool success = pp->apply(initial, proj);

    PathVectorPtr_t path(core::PathVector::create(
        initial->outputSize(), initial->outputDerivativeSize()));
    path->appendPath(proj);
    problemSolver()->addPath(path);
    return success;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::interruptPathPlanning() { problemSolver()->interrupt(); }

// ---------------------------------------------------------------

Long Problem::numberPaths() {
  try {
    return (Long)problemSolver()->paths().size();
  } catch (std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::intSeq* Problem::optimizePath(ULong pathId) {
  try {
    if (pathId >= problemSolver()->paths().size()) {
      std::ostringstream oss("wrong path id: ");
      oss << pathId << ", number path: " << problemSolver()->paths().size()
          << ".";
      throw std::runtime_error(oss.str());
    }
    // Start timer
    boost::posix_time::ptime start =
        boost::posix_time::microsec_clock::universal_time();

    PathVectorPtr_t initial = problemSolver()->paths()[pathId];
    problemSolver()->optimizePath(initial);

    // Stop timer
    boost::posix_time::time_duration time =
        boost::posix_time::microsec_clock::universal_time() - start;

    hpp::intSeq* ret = new hpp::intSeq;
    ret->length(4);
    (*ret)[0] = static_cast<CORBA::Long>(time.hours());
    (*ret)[1] = static_cast<CORBA::Long>(time.minutes());
    (*ret)[2] = static_cast<CORBA::Long>(time.seconds());
    (*ret)[3] = static_cast<CORBA::Long>(time.fractional_seconds() / 1000);
    return ret;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

Double Problem::pathLength(ULong pathId) {
  try {
    if (pathId >= problemSolver()->paths().size()) {
      std::ostringstream oss("wrong path id: ");
      oss << pathId << ", number path: " << problemSolver()->paths().size()
          << ".";
      throw std::runtime_error(oss.str());
    }
    return problemSolver()->paths()[pathId]->length();
  } catch (std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::floatSeq* Problem::configAtParam(ULong pathId, Double atDistance) {
  try {
    if (pathId >= problemSolver()->paths().size()) {
      std::ostringstream oss("wrong path id: ");
      oss << pathId << ", number path: " << problemSolver()->paths().size()
          << ".";
      throw std::runtime_error(oss.str());
    }
    PathPtr_t path = problemSolver()->paths()[pathId];
    bool success;
    Configuration_t config = path->eval(atDistance, success);
    if (!success) {
      throw std::runtime_error(
          "Failed to apply constraint in path "
          "evaluation.");
    }
    // Allocate result now that the size is known.
    std::size_t size = config.size();
    double* dofArray = hpp::floatSeq::allocbuf((ULong)size);
    hpp::floatSeq* floatSeq = new hpp::floatSeq(
        (CORBA::ULong)size, (CORBA::ULong)size, dofArray, true);
    for (std::size_t i = 0; i < size; ++i) {
      dofArray[(CORBA::ULong)i] = config[i];
    }
    return floatSeq;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::floatSeq* Problem::derivativeAtParam(ULong pathId, ULong order,
                                          Double atDistance) {
  try {
    if (pathId >= problemSolver()->paths().size()) {
      std::ostringstream oss("wrong path id: ");
      oss << pathId << ", number path: " << problemSolver()->paths().size()
          << ".";
      throw std::runtime_error(oss.str());
    }
    PathPtr_t path = problemSolver()->paths()[pathId];
    vector_t velocity(problemSolver()->robot()->numberDof());
    path->derivative(velocity, atDistance, order);
    return vectorToFloatSeq(velocity);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// --------------------------------------------------------------

hpp::floatSeqSeq* Problem::getWaypoints(ULong pathId, hpp::floatSeq_out times) {
  try {
    if (pathId >= problemSolver()->paths().size()) {
      std::ostringstream oss("wrong path id: ");
      oss << pathId << ", number path: " << problemSolver()->paths().size()
          << ".";
      throw std::runtime_error(oss.str().c_str());
    }
    PathVectorPtr_t path = problemSolver()->paths()[pathId];
    PathVectorPtr_t flat = core::PathVector::create(
        path->outputSize(), path->outputDerivativeSize());
    path->flatten(flat);
    core::matrix_t points(flat->numberPaths() + 1, path->outputSize());
    core::vector_t ts(flat->numberPaths() + 1);
    ts(0) = flat->timeRange().first;
    for (std::size_t i = 0; i < flat->numberPaths(); ++i) {
      points.row(i) = flat->pathAtRank(i)->initial();
      ts(i + 1) = ts(i) + flat->pathAtRank(i)->length();
    }
    points.row(flat->numberPaths()) = flat->end();
    times = vectorToFloatSeq(ts);
    return matrixToFloatSeqSeq(points);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::floatSeqSeq* Problem::nodes() {
  hpp::floatSeqSeq* res;
  try {
    const Nodes_t& nodes(problemSolver()->roadmap()->nodes());
    res = new hpp::floatSeqSeq;
    res->length((CORBA::ULong)nodes.size());
    std::size_t i = 0;
    for (Nodes_t::const_iterator itNode = nodes.begin(); itNode != nodes.end();
         itNode++) {
      ConfigurationPtr_t config = (*itNode)->configuration();
      ULong size = (ULong)config->size();
      double* dofArray = hpp::floatSeq::allocbuf(size);
      hpp::floatSeq floats(size, size, dofArray, true);
      // convert the config in dofseq
      for (size_type j = 0; j < config->size(); ++j) {
        dofArray[j] = (*config)[j];
      }
      (*res)[(CORBA::ULong)i] = floats;
      ++i;
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
  return res;
}

// ---------------------------------------------------------------

Long Problem::numberEdges() {
  return (Long)problemSolver()->roadmap()->edges().size();
}

// ---------------------------------------------------------------

void Problem::edge(ULong edgeId, hpp::floatSeq_out q1, hpp::floatSeq_out q2) {
  try {
    const Edges_t& edges(problemSolver()->roadmap()->edges());
    Edges_t::const_iterator itEdge = edges.begin();
    std::size_t i = 0;
    while (i < edgeId) {
      ++i;
      itEdge++;
    }
    ConfigurationPtr_t config1 = (*itEdge)->from()->configuration();
    ConfigurationPtr_t config2 = (*itEdge)->to()->configuration();
    ULong size = (ULong)config1->size();

    hpp::floatSeq* q1_ptr = new hpp::floatSeq();
    q1_ptr->length(size);
    hpp::floatSeq* q2_ptr = new hpp::floatSeq();
    q2_ptr->length(size);

    for (i = 0; i < size; ++i) {
      (*q1_ptr)[(CORBA::ULong)i] = (*config1)[i];
      (*q2_ptr)[(CORBA::ULong)i] = (*config2)[i];
    }
    q1 = q1_ptr;
    q2 = q2_ptr;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

Long Problem::numberNodes() {
  try {
    if (problemSolver()->roadmap()) {
      return (Long)problemSolver()->roadmap()->nodes().size();
    } else {
      throw std::runtime_error("No roadmap in problem solver.");
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::floatSeq* Problem::node(ULong nodeId) {
  try {
    const Nodes_t& nodes(problemSolver()->roadmap()->nodes());

    if (nodes.size() > nodeId) {
      Nodes_t::const_iterator itNode = std::next(nodes.begin(), nodeId);
      ConfigurationPtr_t conf = (*itNode)->configuration();
      ULong size = (ULong)conf->size();

      hpp::floatSeq* q_ptr = new hpp::floatSeq();
      q_ptr->length(size);

      for (ULong i = 0; i < size; ++i) {
        (*q_ptr)[i] = (*conf)[i];
      }
      return q_ptr;
    } else {
      std::ostringstream oss("wrong nodeId :");
      oss << nodeId << ", number of nodes: " << nodes.size() << ".";
      throw std::runtime_error(oss.str().c_str());
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// -----------------------------------------------------------------

Long Problem::connectedComponentOfEdge(ULong edgeId) {
  try {
    const Edges_t& edges(problemSolver()->roadmap()->edges());
    if (edges.size() > edgeId) {
      Edges_t::const_iterator itEdge = std::next(edges.begin(), edgeId);

      const core::ConnectedComponents_t& ccs =
          problemSolver()->roadmap()->connectedComponents();
      core::ConnectedComponents_t::const_iterator itcc = ccs.begin();
      for (std::size_t i = 0; i < ccs.size(); ++i) {
        if (*itcc == (*itEdge)->from()->connectedComponent()) {
          return (CORBA::Long)i;
        }
        itcc++;
      }
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
  throw hpp::Error("Connected component not found");
}

// -----------------------------------------------------------------

Long Problem::connectedComponentOfNode(ULong nodeId) {
  try {
    const Nodes_t& nodes(problemSolver()->roadmap()->nodes());
    if (nodes.size() > nodeId) {
      Nodes_t::const_iterator itNode = std::next(nodes.begin(), nodeId);

      const core::ConnectedComponents_t& ccs =
          problemSolver()->roadmap()->connectedComponents();
      core::ConnectedComponents_t::const_iterator itcc = ccs.begin();
      for (std::size_t i = 0; i < ccs.size(); ++i) {
        if (*itcc == (*itNode)->connectedComponent()) {
          return (CORBA::Long)i;
        }
        itcc++;
      }
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
  throw hpp::Error("Connected component not found");
}

// -----------------------------------------------------------------

Long Problem::numberConnectedComponents() {
  return (Long)problemSolver()->roadmap()->connectedComponents().size();
}

// ---------------------------------------------------------------

hpp::floatSeqSeq* Problem::nodesConnectedComponent(ULong connectedComponentId) {
  const ConnectedComponents_t& connectedComponents(
      problemSolver()->roadmap()->connectedComponents());
  if ((std::size_t)connectedComponentId >= connectedComponents.size()) {
    std::ostringstream oss;
    oss << "connectedComponentId=" << connectedComponentId
        << " out of range [0," << connectedComponents.size() - 1 << "].";
    throw hpp::Error(oss.str().c_str());
  }
  hpp::floatSeqSeq* res;
  try {
    ConnectedComponents_t::const_iterator itcc = connectedComponents.begin();
    ULong i = 0;
    while (i != connectedComponentId) {
      ++i;
      itcc++;
    }
    const NodeVector_t& nodes((*itcc)->nodes());
    res = new hpp::floatSeqSeq;
    res->length((CORBA::ULong)nodes.size());
    i = 0;
    for (NodeVector_t::const_iterator itNode = nodes.begin();
         itNode != nodes.end(); itNode++) {
      ConfigurationPtr_t config = (*itNode)->configuration();
      ULong size = (ULong)config->size();
      double* dofArray = hpp::floatSeq::allocbuf(size);
      hpp::floatSeq floats(size, size, dofArray, true);
      // convert the config in dofseq
      for (size_type j = 0; j < config->size(); ++j) {
        dofArray[j] = (*config)[j];
      }
      (*res)[i] = floats;
      ++i;
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
  return res;
}

// ---------------------------------------------------------------

hpp::floatSeq* Problem::getNearestConfig(const hpp::floatSeq& config,
                                         const Long connectedComponentId,
                                         hpp::core::value_type& distance) {
  hpp::floatSeq* res;
  try {
    const hpp::core::ConnectedComponents_t& connectedComponents(
        problemSolver()->roadmap()->connectedComponents());
    hpp::core::NodePtr_t nearest;
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    ConfigurationPtr_t configuration = floatSeqToConfigPtr(robot, config, true);
    if (connectedComponentId < 0) {
      nearest =
          problemSolver()->roadmap()->nearestNode(configuration, distance);
    } else {
      if ((std::size_t)connectedComponentId >= connectedComponents.size()) {
        std::ostringstream oss;
        oss << "connectedComponentId=" << connectedComponentId
            << " out of range [0," << connectedComponents.size() - 1 << "].";
        throw std::runtime_error(oss.str().c_str());
      }
      hpp::core::ConnectedComponents_t::const_iterator itcc =
          connectedComponents.begin();
      std::advance(itcc, connectedComponentId);
      nearest = problemSolver()->roadmap()->nearestNode(configuration, *itcc,
                                                        distance);
    }
    if (!nearest) throw hpp::Error("Nearest node not found");
    res = vectorToFloatSeq(*(nearest->configuration()));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
  return res;
}

// ---------------------------------------------------------------

void Problem::clearRoadmap() {
  try {
    problemSolver()->roadmap()->clear();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::resetRoadmap() {
  try {
    problemSolver()->resetRoadmap();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Problem::saveRoadmap(const char* filename) {
  try {
    core::ProblemSolverPtr_t ps(problemSolver());
    DevicePtr_t robot = getRobotOrThrow(ps);

    using namespace core::parser;

    typedef hpp::serialization::archive_tpl<
        boost::archive::binary_oarchive,
        hpp::serialization::remove_duplicate::vector_archive>
        archive_type;
    core::RoadmapPtr_t roadmap(ps->roadmap());
    serializeRoadmap<archive_type>(roadmap, std::string(filename),
                                   make_nvp(robot->name(), robot.get()));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Problem::loadRoadmap(const char* filename) {
  try {
    core::ProblemSolverPtr_t ps(problemSolver());
    DevicePtr_t robot = getRobotOrThrow(ps);

    using namespace core::parser;

    typedef hpp::serialization::archive_tpl<
        boost::archive::binary_iarchive,
        hpp::serialization::remove_duplicate::vector_archive>
        archive_type;
    hpp::core::RoadmapPtr_t roadmap;
    serializeRoadmap<archive_type>(roadmap, std::string(filename),
                                   make_nvp(robot->name(), robot.get()));
    problemSolver()->roadmap(roadmap);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::scCreateScalarMultiply(const char* outName, Double scalar,
                                     const char* inName) {
  if (!problemSolver()->robot()) throw hpp::Error("No robot loaded");
  try {
    std::string inN(inName);
    std::string outN(outName);
    if (!problemSolver()->numericalConstraints.has(inN))
      throw Error(("Constraint " + inN + " not found").c_str());
    ImplicitPtr_t in = problemSolver()->numericalConstraints.get(inN);
    typedef constraints::FunctionExp<constraints::DifferentiableFunction>
        FunctionExp_t;
    typedef constraints::ScalarMultiply<FunctionExp_t> Expr_t;
    constraints::DifferentiableFunctionPtr_t out =
        constraints::SymbolicFunction<Expr_t>::create(
            outN, problemSolver()->robot(),
            constraints::Traits<Expr_t>::Ptr_t(
                new Expr_t(scalar, FunctionExp_t::create(in->functionPtr()))));

    problemSolver()->addNumericalConstraint(
        outN, Implicit::create(out, in->comparisonType()));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::core_idl::Distance_ptr Problem::getDistance() {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    DevicePtr_t robot = getRobotOrThrow(ps);
    core::DistancePtr_t distance = problem(ps, true)->distance();

    hpp::core_idl::Distance_var d = makeServantDownCast<core_impl::Distance>(
        server_->parent(), core_impl::Distance::Storage(distance));
    //(server_->parent(), core_impl::Distance::Storage (robot, distance));
    return d._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::setDistance(hpp::core_idl::Distance_ptr distance) {
  core::DistancePtr_t d;
  try {
    d = reference_to_object<core::Distance>(server_->parent(), distance);
  } catch (const Error& e) {
    // TODO in this case, we should define a distance from the CORBA type.
    // This would allow to implement a distance class in Python.
    throw;
  }

  core::ProblemSolverPtr_t ps = problemSolver();
  core::ProblemPtr_t p = problem(ps, true);
  p->distance(d);
}

// ---------------------------------------------------------------

hpp::core_idl::Path_ptr Problem::getPath(ULong pathId) {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    if (pathId >= ps->paths().size()) {
      HPP_THROW(Error, "wrong path id: " << pathId << ", number path: "
                                         << ps->paths().size() << ".");
    }

    core::PathVectorPtr_t pv = ps->paths()[pathId];
    hpp::core_idl::Path_var d =
        makeServantDownCast<core_impl::Path>(server_->parent(), pv);
    return d._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

ULong Problem::addPath(hpp::core_idl::PathVector_ptr _path) {
  core::PathVectorPtr_t path;
  try {
    path = reference_to_object<core::PathVector> (server_->parent(), _path);
  } catch (const Error& e) {
    // TODO in this case, we should define a distance from the CORBA type.
    // This would allow to implement a distance class in Python.
    throw;
  }
  if (!path) throw Error("Could not convert the path into a PathVector");
  core::ProblemSolverPtr_t ps = problemSolver();
  ps->addPath(path);
  return (ULong)(ps->paths().size() - 1);
}

// ---------------------------------------------------------------

hpp::core_idl::SteeringMethod_ptr Problem::getSteeringMethod() {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    DevicePtr_t robot = getRobotOrThrow(ps);
    core::SteeringMethodPtr_t sm = problem(ps, true)->steeringMethod();

    hpp::core_idl::SteeringMethod_var d =
        makeServantDownCast<core_impl::SteeringMethod>(
            server_->parent(), core_impl::SteeringMethod::Storage(sm));
    // core_idl::SteeringMethod::Storage (robot, sm));
    return d._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::core_idl::PathValidation_ptr Problem::getPathValidation() {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    core::PathValidationPtr_t pv = problem(ps, true)->pathValidation();

    hpp::core_idl::PathValidation_var d =
        makeServantDownCast<core_impl::PathValidation>(
            server_->parent(), core_impl::PathValidation::Storage(pv));
    return d._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::core_idl::PathPlanner_ptr Problem::getPathPlanner() {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    core::PathPlannerPtr_t pv = ps->pathPlanner();

    hpp::core_idl::PathPlanner_var d =
        makeServantDownCast<core_impl::PathPlanner>(
            server_->parent(), core_impl::PathPlanner::Storage(pv));
    return d._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::core_idl::Problem_ptr Problem::getProblem() {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    core::ProblemPtr_t pb = problem(ps, true);

    hpp::core_idl::Problem_var pb_idl = makeServantDownCast<core_impl::Problem>(
        server_->parent(), core_impl::Problem::Storage(pb));
    return pb_idl._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

hpp::constraints_idl::Implicit_ptr Problem::getConstraint(const char* name) {
  try {
    const std::string fn(name);
    core::ProblemSolverPtr_t ps = problemSolver();
    if (!ps->numericalConstraints.has(fn))
      throw Error(("Constraint " + fn + " not found").c_str());

    hpp::constraints_idl::Implicit_var d =
        makeServantDownCast<constraints_impl::Implicit>(
            server_->parent(), ps->numericalConstraints.get(fn));
    return d._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

// ---------------------------------------------------------------

void Problem::setRobot(hpp::pinocchio_idl::Device_ptr _robot) {
  pinocchio::DevicePtr_t robot;
  try {
    robot = reference_to_object<pinocchio::Device>(server_->parent(), _robot);
  } catch (const Error& e) {
    throw;
  }
  if (!robot) throw Error("Could not get the robot");
  problemSolver()->robot(robot);
}

pinocchio_idl::CollisionObject_ptr Problem::getObstacle(const char* name) {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    pinocchio_idl::CollisionObject_var o =
        makeServantDownCast<pinocchio_impl::CollisionObject>(
            server_->parent(), ps->obstacle(name));
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

core_idl::Problem_ptr Problem::createProblem(pinocchio_idl::Device_ptr robot) {
  try {
    core_idl::Problem_var o = makeServantDownCast<core_impl::Problem>(
        server_->parent(),
        core::Problem::create(
            reference_to_object<pinocchio::Device>(server_->parent(), robot)));
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

core_idl::Roadmap_ptr Problem::createRoadmap(core_idl::Distance_ptr distance,
                                             pinocchio_idl::Device_ptr robot) {
  try {
    core_idl::Roadmap_var o = makeServantDownCast<core_impl::Roadmap>(
        server_->parent(),
        core::Roadmap::create(
            reference_to_object<core::Distance>(server_->parent(), distance),
            reference_to_object<pinocchio::Device>(server_->parent(), robot)));
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

core_idl::Roadmap_ptr Problem::readRoadmap(const char* filename,
                                           pinocchio_idl::Device_ptr robot) {
  try {
    DevicePtr_t device =
        reference_to_object<pinocchio::Device>(server_->parent(), robot);

    hpp::core::RoadmapPtr_t roadmap;
    std::string fn(filename);
    bool xml = (fn.size() >= 4 && fn.compare(fn.size() - 4, 4, ".xml") == 0);
    using namespace core::parser;
    if (xml) {
      typedef hpp::serialization::archive_tpl<
          boost::archive::xml_iarchive,
          hpp::serialization::remove_duplicate::vector_archive>
          archive_type;
      serializeRoadmap<archive_type>(roadmap, fn,
                                     make_nvp(device->name(), device.get()));
    } else {
      typedef hpp::serialization::archive_tpl<
          boost::archive::binary_iarchive,
          hpp::serialization::remove_duplicate::vector_archive>
          archive_type;
      serializeRoadmap<archive_type>(roadmap, fn,
                                     make_nvp(device->name(), device.get()));
    }

    core_idl::Roadmap_var o =
        makeServantDownCast<core_impl::Roadmap>(server_->parent(), roadmap);
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Problem::writeRoadmap(const char* filename, core_idl::Roadmap_ptr _roadmap,
                           pinocchio_idl::Device_ptr robot) {
  try {
    DevicePtr_t device =
        reference_to_object<pinocchio::Device>(server_->parent(), robot);
    core::RoadmapPtr_t roadmap =
        reference_to_object<core::Roadmap>(server_->parent(), _roadmap);

    std::string fn(filename);
    bool xml = (fn.size() >= 4 && fn.compare(fn.size() - 4, 4, ".xml") == 0);
    using namespace core::parser;
    if (xml) {
      typedef hpp::serialization::archive_tpl<
          boost::archive::xml_oarchive,
          hpp::serialization::remove_duplicate::vector_archive>
          archive_type;
      serializeRoadmap<archive_type>(roadmap, fn,
                                     make_nvp(device->name(), device.get()));
    } else {
      typedef hpp::serialization::archive_tpl<
          boost::archive::binary_oarchive,
          hpp::serialization::remove_duplicate::vector_archive>
          archive_type;
      serializeRoadmap<archive_type>(roadmap, fn,
                                     make_nvp(device->name(), device.get()));
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

core_idl::PathPlanner_ptr Problem::createPathPlanner(
    const char* type, core_idl::Problem_ptr problem,
    core_idl::Roadmap_ptr roadmap) {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    core_idl::PathPlanner_var o = makeServantDownCast<core_impl::PathPlanner>(
        server_->parent(),
        (ps->pathPlanners.get(type))(
            reference_to_object<core::Problem>(server_->parent(), problem),
            reference_to_object<core::Roadmap>(server_->parent(), roadmap)));
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}
core_idl::PathOptimizer_ptr Problem::createPathOptimizer(
    const char* type, core_idl::Problem_ptr problem) {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    core_idl::PathOptimizer_var o =
        makeServantDownCast<core_impl::PathOptimizer>(
            server_->parent(),
            (ps->pathOptimizers.get(type))(reference_to_object<core::Problem>(
                server_->parent(), problem)));
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

core_idl::PathProjector_ptr Problem::createPathProjector(
    const char* type, core_idl::Problem_ptr problem, value_type parameter) {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    core_idl::PathProjector_var o =
        makeServantDownCast<core_impl::PathProjector>(
            server_->parent(),
            (ps->pathProjectors.get(type))(
                reference_to_object<core::Problem>(server_->parent(), problem),
                parameter));
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

core_idl::PathValidation_ptr Problem::createPathValidation(
    const char* type, pinocchio_idl::Device_ptr robot, value_type parameter) {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    core_idl::PathValidation_var o =
        makeServantDownCast<core_impl::PathValidation>(
            server_->parent(), (ps->pathValidations.get(type))(
                                   reference_to_object<pinocchio::Device>(
                                       server_->parent(), robot),
                                   parameter));
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

core_idl::ConfigurationShooter_ptr Problem::createConfigurationShooter(
    const char* type, core_idl::Problem_ptr problem) {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    core_idl::ConfigurationShooter_var o =
        makeServantDownCast<core_impl::ConfigurationShooter>(
            server_->parent(), (ps->configurationShooters.get(type))(
                                   reference_to_object<core::Problem>(
                                       server_->parent(), problem)));
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}
core_idl::ConfigValidation_ptr Problem::createConfigValidation(
    const char* type, pinocchio_idl::Device_ptr robot) {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    core_idl::ConfigValidation_var o =
        makeServantDownCast<core_impl::ConfigValidation>(
            server_->parent(), (ps->configValidations.get(type))(
                                   reference_to_object<pinocchio::Device>(
                                       server_->parent(), robot)));
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}
core_idl::Distance_ptr Problem::createDistance(const char* type,
                                               core_idl::Problem_ptr problem) {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    core_idl::Distance_var o = makeServantDownCast<core_impl::Distance>(
        server_->parent(),
        (ps->distances.get(type))(
            reference_to_object<core::Problem>(server_->parent(), problem)));
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}
core_idl::SteeringMethod_ptr Problem::createSteeringMethod(
    const char* type, core_idl::Problem_ptr problem) {
  try {
    core::ProblemSolverPtr_t ps = problemSolver();
    core_idl::SteeringMethod_var o =
        makeServantDownCast<core_impl::SteeringMethod>(
            server_->parent(),
            (ps->steeringMethods.get(type))(reference_to_object<core::Problem>(
                server_->parent(), problem)));
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

}  // namespace impl
}  // namespace corbaServer
}  // namespace hpp
