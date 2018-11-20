// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, Joseph Mirabel, JRL.
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
#include <boost/icl/interval_set.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/portability.hh>

#include <hpp/fcl/shape/geometric_shapes.h>

#include <hpp/pinocchio/configuration.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/subchain-path.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/parser/roadmap-factory.hh>

#include <hpp/constraints/affine-function.hh>
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/distance-between-bodies.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/com-between-feet.hh>
#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/convex-shape-contact.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/symbolic-calculus.hh>
#include <hpp/constraints/symbolic-function.hh>
#ifdef HPP_CONSTRAINTS_USE_QPOASES
# include <hpp/constraints/static-stability.hh>
#endif
#include <hpp/constraints/configuration-constraint.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>

#include "problem.impl.hh"
#include "tools.hh"

using hpp::pinocchio::ObjectVector_t;
using hpp::pinocchio::CenterOfMassComputation;
using hpp::pinocchio::CenterOfMassComputationPtr_t;

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
#ifdef HPP_CONSTRAINTS_USE_QPOASES
using hpp::constraints::StaticStability;
using hpp::constraints::StaticStabilityPtr_t;
#endif
using hpp::constraints::DifferentiableFunctionPtr_t;
using hpp::constraints::LockedJoint;
using hpp::constraints::LockedJointPtr_t;

using hpp::constraints::Implicit;
using hpp::constraints::ImplicitPtr_t;

namespace boost {
  namespace icl {
    template<>
      struct interval_traits< hpp::core::segment_t >
      {
        typedef hpp::core::segment_t interval_type;
        typedef hpp::core::size_type      domain_type;
        typedef std::less<domain_type>    domain_compare;

        static interval_type construct(const domain_type& lo, const domain_type& up)
        { return interval_type(lo, up - lo); }

        static domain_type lower(const interval_type& inter){ return inter.first; };
        static domain_type upper(const interval_type& inter){ return inter.first + inter.second; };
      };

    template<>
      struct interval_bound_type<hpp::core::segment_t>
      {
        typedef interval_bound_type type;
        BOOST_STATIC_CONSTANT(bound_type, value = interval_bounds::static_right_open);//[lo..up)
      };

  } // namespace icl
} // namespace boost


namespace hpp
{
  namespace corbaServer
  {
    namespace impl
    {
      namespace {
        using hpp::core::Parameter;

        static const matrix3_t   I3   (matrix3_t  ::Identity());
        static const vector3_t   zero (vector3_t  ::Zero());
        static const Transform3f Id   (Transform3f::Identity());

        struct Parameter_CorbaAny {
          private:
            template <typename first, typename second>
              static inline second as (const first& f) { return second (f); }

            struct convertToParam {
              const CORBA::Any& corbaAny;
              Parameter& param;
              bool& done;
              template <typename T> void operator()(T) {
                if (done) return;
                typename T::second_type d;
                if (corbaAny >>= d) {
                  param = Parameter (as<typename T::second_type, typename T::first_type>(d));
                  done = true;
                }
              }
              convertToParam (const CORBA::Any& a, Parameter& p, bool& d) : corbaAny(a), param(p), done(d) {}
            };
            /// TODO: This list of CORBA types is not complete.
            typedef boost::mpl::vector<
              // There is no operator<<= (CORBA::Any, CORBA::Boolean)
              std::pair<bool       , CORBA::Boolean>,

              std::pair<size_type  , CORBA::LongLong>,
              std::pair<size_type  , CORBA::Long>,
              std::pair<size_type  , CORBA::Short>,
              std::pair<size_type  , CORBA::ULongLong>,
              std::pair<size_type  , CORBA::ULong>,
              std::pair<size_type  , CORBA::UShort>,

              std::pair<value_type , CORBA::Float>,
              std::pair<value_type , CORBA::Double>,

              std::pair<std::string, const char*>

              // This two do not work because omniidl must be given the option -Wba in order to generate 
              // a DynSK.cc file containing the conversion to/from Any type.
              // std::pair<vector_t   , floatSeq>,
              // std::pair<matrix_t   , floatSeqSeq>
                > Parameter_AnyPairs_t;

          public:
            static Parameter toParameter (const CORBA::Any& an) {
              Parameter out;
              bool done = false;
              convertToParam ret(an, out, done);
              boost::mpl::for_each<Parameter_AnyPairs_t> (ret);
              if (!done) throw hpp::Error ("Could not convert to Parameter");
              return out;
            }
            static CORBA::Any toCorbaAny (const Parameter& p) {
              CORBA::Any out;
              switch (p.type()) {
                case Parameter::BOOL:
                  out <<= p.boolValue();
                  break;
                case Parameter::INT:
                  out <<= p.intValue();
                  break;
                case Parameter::FLOAT:
                  out <<= p.floatValue();
                  break;
                case Parameter::STRING:
                  out <<= p.stringValue().c_str();
                  break;
                case Parameter::VECTOR:
                  out <<= vectorToFloatSeq(p.vectorValue());
                  break;
                case Parameter::MATRIX:
                  out <<= matrixToFloatSeqSeq(p.matrixValue());
                  break;
                default:
                  throw hpp::Error ("Could not convert to CORBA::Any");
                  break;
              }
              return out;
            }
        };

        template <> inline
          vector_t Parameter_CorbaAny::as<floatSeq, vector_t> (const floatSeq& in)
          { return floatSeqToVector(in); }
        template <> inline
          matrix_t Parameter_CorbaAny::as<floatSeqSeq, matrix_t> (const floatSeqSeq& in)
          { return floatSeqSeqToMatrix(in); }

        typedef hpp::core::segment_t segment_t;
        typedef hpp::core::segments_t segments_t;
        typedef boost::icl::interval_set<size_type, std::less, segment_t>
          BoostIntervalSet_t;

        segments_t convertInterval (const BoostIntervalSet_t& intSet)
        {
          segments_t ret;
          for (BoostIntervalSet_t::const_iterator _int = intSet.begin (); _int != intSet.end (); ++_int)
            ret.push_back(*_int);
          return ret;
        }

        template <int PositionOrientationFlag> DifferentiableFunctionPtr_t
          buildGenericFunc(const DevicePtr_t& robot,
              const std::string& name,
              const std::string& nameF1, const std::string& nameF2,
              const Transform3f& M1,     const Transform3f& M2,
              const std::vector<bool> mask)
          {
            if (nameF1.empty() && nameF2.empty())
	      throw hpp::Error ("At least one joint should be provided.");
            if (nameF2.empty())
              return buildGenericFunc<PositionOrientationFlag>
                (robot, name, nameF2, nameF1, M2, M1, mask);
            // From here, nameF2 is not empty.

            const se3::Model& model (robot->model());
            bool relative = (!nameF1.empty());
            JointPtr_t joint1, joint2;
            Transform3f ref1 = M1, ref2 = M2;
            if (relative) {
              if (!model.existFrame(nameF1)) {
		std::ostringstream oss;
		oss << "Joint " << nameF1 << " not found.";
                throw hpp::Error (oss.str ().c_str ());
	      }
              const se3::Frame& f1 = model.frames[model.getFrameId(nameF1)];
              // If f1 is a JOINT, then f1.placement should be identity
              assert (f1.type != se3::JOINT || f1.placement.isIdentity());
              ref1 = f1.placement * M1;
              joint1 = JointPtr_t (new Joint (robot, f1.parent));
            }

            if (!model.existFrame(nameF2)) {
		std::ostringstream oss;
		oss << "Joint " << nameF2 << " not found.";
                throw hpp::Error (oss.str ().c_str ());
	    }
            const se3::Frame& f2 = model.frames[model.getFrameId(nameF2)];
            // If f2 is a JOINT, then f2.placement should be identity
            assert (f2.type != se3::JOINT || f2.placement.isIdentity());
            ref2 = f2.placement * M2;
            joint2 = JointPtr_t (new Joint (robot, f2.parent));

            if (relative) {
              // Both joints are provided
              return constraints::GenericTransformation
                <PositionOrientationFlag | constraints::RelativeBit>
                ::create (name, robot, joint1, joint2, ref1, ref2, mask);
            } else {
              return constraints::GenericTransformation<PositionOrientationFlag>
                ::create (name, robot, joint2, ref2, ref1, mask);
            }
          }

        core::ProblemPtr_t problem (const core::ProblemSolverPtr_t& ps,
            bool throwIfNull = true)
        {
          core::ProblemPtr_t p = ps->problem();
          if (throwIfNull && p==NULL)
            throw Error ("No problem in the ProblemSolver");
          return p;
        }
      }

      // ---------------------------------------------------------------

      Problem::Problem (corbaServer::Server* server)
	: server_ (server)
      {}

      // ---------------------------------------------------------------

      void Problem::shutdown ()
      {
        server_->requestShutdown(false);
      }

      // ---------------------------------------------------------------

      core::ProblemSolverPtr_t Problem::problemSolver ()
      {
        return server_->problemSolver();
      }

      // ---------------------------------------------------------------

#define _NOOP ((void)0)
#define _CASE(test, op_true, op_false)                                         \
      if (!ok) { if (boost::iequals(w,test)) {                                 \
        op_true;                                                               \
        ok = true;                                                             \
      } else { op_false; } }
#define _CASE_SET_RET(name, val_true)                                          \
      _CASE (name, ret = val_true, types.push_back(name))
#define _CASE_PS_MAP(name, map)                                                \
      _CASE_SET_RET (name, problemSolver()->map.getKeys <Ret_t> ())

      Names_t* Problem::getAvailable (const char* what) throw (hpp::Error)
      {
        std::string w (what);
        typedef std::list <std::string> Ret_t;
        Ret_t ret, types;
        bool ok = false;

        _CASE_PS_MAP ("PathOptimizer"       , pathOptimizers)
        _CASE_PS_MAP ("PathProjector"       , pathProjectors)
        _CASE_PS_MAP ("PathPlanner"         , pathPlanners)
        _CASE_PS_MAP ("ConfigurationShooter", configurationShooters)
        _CASE_PS_MAP ("PathValidation"      , pathValidations)
        _CASE_PS_MAP ("ConfigValidation"    , configValidations)
        _CASE_PS_MAP ("SteeringMethod"      , steeringMethods)
        _CASE_PS_MAP ("Distance"            , distances)
        _CASE_PS_MAP ("NumericalConstraint" , numericalConstraints)
        _CASE_PS_MAP ("LockedJoint"         , lockedJoints)
        _CASE_PS_MAP ("CenterOfMass"        , centerOfMassComputations)
        _CASE_SET_RET ("Problem", server_->problemSolverMap()->keys <Ret_t> ())
        _CASE_SET_RET ("Parameter", problem(problemSolver())->parameters.getKeys <Ret_t> ())
        _CASE_SET_RET ("DefaultParameter", problem(problemSolver())->parameterDescriptions().getKeys <Ret_t> ())
        _CASE_SET_RET ("Type", types)

        if (!ok) {
          throw Error (("Type \"" + std::string(what) + "\" not known").c_str());
        }

        return toNames_t (ret.begin(), ret.end());
      }

      // ---------------------------------------------------------------

#define _CASE_PUSH_TO(name, val)                                               \
      _CASE(name, ret.push_back(val), types.push_back(name))

      Names_t* Problem::getSelected (const char* what) throw (hpp::Error)
      {
        std::string w (what);
        typedef std::list <std::string> Ret_t;
        Ret_t ret, types;
        bool ok = false;
        core::value_type tol;

        _CASE_SET_RET ("PathOptimizer",
            Ret_t(problemSolver()->pathOptimizerTypes().begin(),
              problemSolver()->pathOptimizerTypes().end()))
        _CASE_SET_RET ("ConfigValidation",
            Ret_t(problemSolver()->configValidationTypes().begin(),
              problemSolver()->configValidationTypes().end()))
        _CASE_PUSH_TO ("PathProjector"       , problemSolver()->pathProjectorType (tol))
        _CASE_PUSH_TO ("PathPlanner"         , problemSolver()->pathPlannerType ())
        _CASE_PUSH_TO ("ConfigurationShooter", problemSolver()->configurationShooterType ())
        _CASE_PUSH_TO ("PathValidation"      , problemSolver()->pathValidationType (tol))
        _CASE_PUSH_TO ("SteeringMethod"      , problemSolver()->steeringMethodType ())
        _CASE_PUSH_TO ("Distance"            , problemSolver()->distanceType ())
        _CASE_PUSH_TO ("Problem"             , server_->problemSolverMap()->selected_)
        _CASE_SET_RET ("Type", types)
        if (!ok) {
          throw Error (("Type \"" + std::string(what) + "\" not known").c_str());
        }

        return toNames_t (ret.begin(), ret.end());
      }

#undef _CASE
#undef _CASE_SET_RET
#undef _CASE_PUSH_TO
#undef _CASE_PS_MAP
#undef _NOOP

      // ---------------------------------------------------------------

      void Problem::setParameter (const char* name, const CORBA::Any& value)
        throw (Error)
      {
        if (problemSolver()->problem() != NULL) {
          try {
            problemSolver()->problem()->setParameter (std::string(name),
                Parameter_CorbaAny::toParameter (value));
          } catch (const std::exception& e) {
            throw hpp::Error (e.what ());
          }
          return;
        }
        throw Error ("No problem in the ProblemSolver");
      }

      // ---------------------------------------------------------------

      CORBA::Any* Problem::getParameter (const char* name) throw (Error)
      {
        if (problemSolver()->problem() != NULL) {
          try {
            const Parameter& param = problemSolver()->problem()->getParameter (name);
            CORBA::Any* ap = new CORBA::Any;
            *ap = Parameter_CorbaAny::toCorbaAny(param);
            return ap;
          } catch (const std::exception& e) {
            throw hpp::Error (e.what ());
          }
        }
        throw Error ("No problem in the ProblemSolver");
      }

      // ---------------------------------------------------------------

      char* Problem::getParameterDoc (const char* name) throw (Error)
      {
        if (problemSolver()->problem() != NULL) {
          try {
            const core::ParameterDescription& desc =
              core::Problem::parameterDescription (name);
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
            throw hpp::Error (e.what ());
          }
        }
        throw Error ("No problem in the ProblemSolver");
      }

      // ---------------------------------------------------------------

      bool Problem::selectProblem (const char* name) throw (hpp::Error)
      {
        std::string psName (name);
        ProblemSolverMapPtr_t psMap (server_->problemSolverMap());
        bool has = psMap->has (psName);
        if (!has) psMap->map_[psName] = core::ProblemSolver::create ();
        psMap->selected_ = psName;
        return !has;
      }

      // ---------------------------------------------------------------

      void Problem::resetProblem () throw (hpp::Error)
      {
        ProblemSolverMapPtr_t psMap (server_->problemSolverMap());
        delete psMap->map_ [ psMap->selected_ ];
        psMap->map_ [ psMap->selected_ ] = core::ProblemSolver::create ();
      }

      // ---------------------------------------------------------------

      void Problem::movePathToProblem (ULong pathId, const char* name,
          const Names_t& jointNames) throw (hpp::Error)
      {
        ProblemSolverMapPtr_t psMap (server_->problemSolverMap());
        if (!psMap->has (std::string(name))) throw Error ("No ProblemSolver of this name");
        core::ProblemSolverPtr_t current = problemSolver(),
                                 other = psMap->map_[std::string(name)];

        BoostIntervalSet_t ints;
        for (CORBA::ULong i = 0; i < jointNames.length (); ++i) {
          JointPtr_t joint = problemSolver()->robot ()->getJointByName
	    (std::string(jointNames[i]));
          if (joint == NULL) {
	    std::ostringstream oss;
	    oss << "Joint " << jointNames[i] << "not found.";
	    throw hpp::Error (oss.str ().c_str ());
	  }
          ints.insert(segment_t(joint->rankInConfiguration(), joint->configSize()));
        }
        segments_t intervals = convertInterval(ints);

        if (pathId >= current->paths().size()) {
          std::ostringstream oss ("wrong path id: ");
          oss << pathId << ", number path: "
            << current->paths ().size () << ".";
          throw Error (oss.str().c_str());
        }
        core::SubchainPathPtr_t nPath =
          core::SubchainPath::create (current->paths()[pathId], intervals);
        core::PathVectorPtr_t pv =
            core::PathVector::create(nPath->outputSize(), nPath->outputDerivativeSize());
        pv->appendPath(nPath);
        other->addPath(pv);
      }

      // ---------------------------------------------------------------

      void Problem::setInitialConfig (const hpp::floatSeq& dofArray)
	throw (hpp::Error)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  ConfigurationPtr_t config = floatSeqToConfigPtr (robot, dofArray, true);
	  problemSolver()->initConfig (config);
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
	ConfigurationPtr_t config = problemSolver()->initConfig ();

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
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  ConfigurationPtr_t config = floatSeqToConfigPtr (robot, dofArray, true);
	  problemSolver()->addGoalConfig (config);
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
	    (problemSolver()->goalConfigs ());
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
	problemSolver()->resetGoalConfigs ();
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
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
        Transform3f::Quaternion_t quat (p [3], p [0], p [1], p [2]);
	Transform3f rotation (quat.matrix(), vector3_t::Zero());
        std::string name (constraintName);

        DifferentiableFunctionPtr_t func = buildGenericFunc<constraints::OrientationBit> (
              problemSolver()->robot(), name,
              joint1Name          , joint2Name,
              Id                  , rotation,
              boolSeqToBoolVector(mask));

        problemSolver()->addNumericalConstraint
          (name, Implicit::create (func));
      }

      // ---------------------------------------------------------------

      void Problem::createTransformationConstraint
      (const char* constraintName, const char* joint1Name,
       const char* joint2Name, const Transform_ p, const hpp::boolSeq& mask)
	throw (hpp::Error)
      {
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
        Transform3f ref (toTransform3f(p));
        std::string name (constraintName);
        DifferentiableFunctionPtr_t func = buildGenericFunc
          <constraints::PositionBit | constraints::OrientationBit> (
              problemSolver()->robot(), name,
              joint1Name          , joint2Name,
              ref                  , Id,
              boolSeqToBoolVector(mask, 6));

        problemSolver()->addNumericalConstraint
          (name, Implicit::create (func));
      }

      // ---------------------------------------------------------------

      void Problem::createTransformationConstraint2
      (const char* constraintName, const char* joint1Name,
       const char* joint2Name, const Transform_ frame1, const Transform_ frame2,
       const hpp::boolSeq& mask)
	throw (hpp::Error)
      {
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
        std::string name (constraintName);
        DifferentiableFunctionPtr_t func = buildGenericFunc
          <constraints::PositionBit | constraints::OrientationBit> (
              problemSolver()->robot(), name,
              joint1Name           , joint2Name,
              toTransform3f(frame1), toTransform3f(frame2),
              boolSeqToBoolVector(mask, 6));

        problemSolver()->addNumericalConstraint
          (name, Implicit::create (func));
      }

      // ---------------------------------------------------------------

      void Problem::createLockedJoint
      (const char* lockedJointName, const char* jointName,
       const hpp::floatSeq& value)
	throw (hpp::Error)
      {
	try {
	  // Get robot in hppPlanner object.
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
	  JointPtr_t joint = robot->getJointByName (jointName);
	  vector_t config = floatSeqToVector (value, joint->configSize());
          hppDout (info, "joint->configurationSpace ()->name () = "
                   << joint->configurationSpace ()->name ());
          hppDout (info, "joint->configurationSpace ()->nq () = "
                   << joint->configurationSpace ()->nq ());
          LiegroupElement lge (config, joint->configurationSpace ());
          LockedJointPtr_t lockedJoint (LockedJoint::create (joint, lge));
          problemSolver()->lockedJoints.add (lockedJointName, lockedJoint);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::createLockedExtraDof
      (const char* lockedDofName, const CORBA::ULong index,
       const hpp::floatSeq& value)
	throw (hpp::Error)
      {
	try {
	  // Get robot in hppPlanner object.
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
	  vector_t config = floatSeqToVector (value);

          LockedJointPtr_t lockedJoint
            (LockedJoint::create (robot, index, config));
          problemSolver()->lockedJoints.add (lockedDofName, lockedJoint);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::createRelativeComConstraint (const char* constraintName,
          const char* comName, const char* jointName, const floatSeq& p,
          const hpp::boolSeq& mask)
        throw (hpp::Error)
      {
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
	JointPtr_t joint;
        CenterOfMassComputationPtr_t comc;
	vector3_t point = floatSeqToVector3 (p);

	std::vector<bool> m = boolSeqToBoolVector (mask, 3);
	try {
          joint = problemSolver()->robot()->getJointByName(jointName);
	  // Test whether joint1 is world frame
          std::string name (constraintName), comN (comName);
          if (comN.compare ("") == 0) {
            problemSolver()->addNumericalConstraint
              (name, Implicit::create
	       (RelativeCom::create (name, problemSolver()->robot(),
				     joint, point, m)));
          } else {
            if (!problemSolver()->centerOfMassComputations.has(comN))
              throw hpp::Error ("Partial COM not found.");
            comc = problemSolver()->centerOfMassComputations.get (comN);
            problemSolver()->addNumericalConstraint
              (name, Implicit::create
	       (RelativeCom::create (name, problemSolver()->robot(), comc,
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
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
	JointPtr_t jointL, jointR, jointRef;
        CenterOfMassComputationPtr_t comc;
	vector3_t pointL = floatSeqToVector3 (pL);
	vector3_t pointR = floatSeqToVector3 (pR);
	vector3_t pointRef (0,0,0);

	std::vector<bool> m = boolSeqToBoolVector (mask);
	try {
          jointL = problemSolver()->robot()->getJointByName(jointLName);
          jointR = problemSolver()->robot()->getJointByName(jointRName);
	  // Test whether joint1 is world frame
          if (std::string (jointRefName) == std::string (""))
            jointRef = problemSolver()->robot()->rootJoint ();
	  else
	    jointRef = problemSolver()->robot()->getJointByName(jointRefName);
          std::string name (constraintName), comN (comName);
          if (comN.compare ("") == 0) {
            problemSolver()->addNumericalConstraint
              (name, Implicit::create
	       (ComBetweenFeet::create (name, problemSolver()->robot(),
					jointL, jointR, pointL, pointR,
					jointRef, pointRef, m)));
          } else {
            if (!problemSolver()->centerOfMassComputations.has(comN))
              throw hpp::Error ("Partial COM not found.");
            comc = problemSolver()->centerOfMassComputations.get (comN);
            problemSolver()->addNumericalConstraint
              (name, Implicit::create
	       (ComBetweenFeet::create (name, problemSolver()->robot(), comc,
					jointL, jointR, pointL, pointR,
					jointRef, pointRef, m)));
          }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::createConvexShapeContactConstraint
        (const char* constraintName, const Names_t& floorJoints,
         const Names_t& objectJoints,
         const hpp::floatSeqSeq& points, const hpp::intSeqSeq& objTriangles,
         const hpp::intSeqSeq& floorTriangles)
        throw (hpp::Error)
      {
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
        try {
          std::string name (constraintName);
          ConvexShapeContactPtr_t f = ConvexShapeContact::create
            (name, problemSolver()->robot());
          problemSolver()->addNumericalConstraint
	    (name, Implicit::create (f));
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
	      // joint = 0x0;
	    } else {
	      joint = problemSolver()->robot ()->getJointByName (jointName);
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
	      // joint = 0x0;
	    } else {
	      joint = problemSolver()->robot ()->getJointByName (jointName);
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

#ifdef HPP_CONSTRAINTS_USE_QPOASES
      void Problem::createStaticStabilityConstraint
      (const char* constraintName, const hpp::Names_t& jointNames,
       const hpp::floatSeqSeq& points, const hpp::floatSeqSeq& normals,
       const char* comRootJointName)
        throw (hpp::Error)
      {
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
        DevicePtr_t robot = problemSolver()->robot();
        JointPtr_t comRJ;
        try {
          std::string name (constraintName);
          /// Create the com
          comRJ = robot->getJointByName(comRootJointName);
          CenterOfMassComputationPtr_t com =
            CenterOfMassComputation::create (robot);
          com->add (comRJ);

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
            if (jn1.empty ()) c.joint1.reset();
            else c.joint1 = robot->getJointByName (jn1);
            if (jn2.empty ()) c.joint2.reset();
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
          problemSolver()->addNumericalConstraint (name,
              Implicit::create (f, core::EqualToZero::create())
              );
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }
#else
      void Problem::createStaticStabilityConstraint
      (const char*, const hpp::Names_t&, const hpp::floatSeqSeq&,
       const hpp::floatSeqSeq&, const char*)
        throw (hpp::Error)
      {
	throw hpp::Error ("Not implemented");
      }
#endif

      // ---------------------------------------------------------------

      void Problem::createPositionConstraint
      (const char* constraintName, const char* joint1Name,
       const char* joint2Name, const hpp::floatSeq& point1,
       const hpp::floatSeq& point2, const hpp::boolSeq& mask)
	throw (hpp::Error)
      {
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
	vector3_t p1 = floatSeqToVector3 (point1);
	vector3_t p2 = floatSeqToVector3 (point2);
	std::string name (constraintName);

        DifferentiableFunctionPtr_t func = buildGenericFunc<constraints::PositionBit> (
              problemSolver()->robot(), name,
              joint1Name          , joint2Name,
              Transform3f (I3, p1), Transform3f (I3, p2),
              boolSeqToBoolVector(mask));

        problemSolver()->addNumericalConstraint
          (name, Implicit::create (func));
      }

      // ---------------------------------------------------------------

      void Problem::createConfigurationConstraint (const char* constraintName,
          const hpp::floatSeq& goal,
          const hpp::floatSeq& weights) throw (hpp::Error)
      {
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
	std::string name (constraintName);
        problemSolver()->numericalConstraints.add
          (name, Implicit::create
           (constraints::ConfigurationConstraint::create
            (name, problemSolver()->robot(),
             floatSeqToConfig (robot, goal, true),
             floatSeqToVector (weights, robot->numberDof()))
            ));
      }

      // ---------------------------------------------------------------

      void Problem::createDistanceBetweenJointConstraint
      (const char* constraintName, const char* joint1Name,
       const char* joint2Name, Double) throw (Error)
      {
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
	try {
	  JointPtr_t joint1 = problemSolver()->robot ()->getJointByName
	    (joint1Name);
	  JointPtr_t joint2 = problemSolver()->robot ()->getJointByName
	    (joint2Name);
	  std::string name (constraintName);
	  problemSolver()->addNumericalConstraint
	    (name, Implicit::create
	     (DistanceBetweenBodies::create (name, problemSolver()->robot(),
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
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
	try {
	  JointPtr_t joint1 = problemSolver()->robot ()->getJointByName
	    (joint1Name);
          std::vector<CollisionObjectPtr_t> objectList;
	  for (CORBA::ULong i=0; i<objects.length (); ++i) {
	    objectList.push_back (problemSolver()->obstacle
				  (std::string (objects [i])));
	  }
	  std::string name (constraintName);
	  problemSolver()->addNumericalConstraint
	    (name, Implicit::create
	     (DistanceBetweenBodies::create (name, problemSolver()->robot(),
					     joint1, objectList)));
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::createIdentityConstraint (const char* constraintName,
          const Names_t& inJoints, const hpp::Names_t& outJoints) throw (Error)
      {
        try {
          using namespace constraints;

          DevicePtr_t robot = problemSolver()->robot ();

          LiegroupSpacePtr_t ispace = LiegroupSpace::empty();
          LiegroupSpacePtr_t ospace = LiegroupSpace::empty();
          segments_t iq, oq, iv, ov;
          for (CORBA::ULong i=0; i<inJoints.length (); ++i) {
            JointPtr_t j = robot->getJointByName (std::string(inJoints[i]));
            *ispace *= j->configurationSpace();
            iq.push_back (segment_t(j->rankInConfiguration(), j->configSize()));
            iv.push_back (segment_t(j->rankInVelocity     (), j->numberDof ()));
          }
          for (CORBA::ULong i=0; i<outJoints.length (); ++i) {
            JointPtr_t j = robot->getJointByName (std::string(outJoints[i]));
            *ospace *= j->configurationSpace();
            oq.push_back (segment_t(j->rankInConfiguration(), j->configSize()));
            ov.push_back (segment_t(j->rankInVelocity     (), j->numberDof ()));
          }
          if (*ispace != *ospace)
            throw Error ("Input and output space are different");
          assert (BlockIndex::cardinal(iq) == BlockIndex::cardinal(oq));
          assert (BlockIndex::cardinal(iv) == BlockIndex::cardinal(ov));

          problemSolver()->addNumericalConstraint (constraintName,
              Explicit::create (robot->configSpace(),
                Identity::create (ispace, constraintName),
                iq, oq, iv, ov)
              );
        } catch (std::exception& exc) {
          throw Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      bool Problem::applyConstraints (const hpp::floatSeq& input,
				      hpp::floatSeq_out output,
				      double& residualError)
	throw (hpp::Error)
      {
	bool success = false;
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
	ConfigurationPtr_t config = floatSeqToConfigPtr (robot, input, true);
	try {
	  success = problemSolver()->constraints ()->apply (*config);
	  if (hpp::core::ConfigProjectorPtr_t configProjector =
	      problemSolver()->constraints ()->configProjector ()) {
	    residualError = configProjector->residualError ();
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	output = vectorToFloatSeq (*config);
	return success;
      }

      // ---------------------------------------------------------------

      bool Problem::optimize (const hpp::floatSeq& input,
          hpp::floatSeq_out output,
          hpp::floatSeq_out residualError)
        throw (hpp::Error)
      {
        bool success = false;
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
        Configuration_t config = floatSeqToConfig (robot, input, true);
        vector_t err;
        try {
          if (!problemSolver()->constraints())
            throw hpp::Error ("The problem has no constraints");
          if (!problemSolver()->constraints()->configProjector())
            throw hpp::Error ("The problem has no config projector");
          core::ConfigProjectorPtr_t cp = problemSolver()->constraints()->configProjector();

          success = cp->optimize (config, 0);
          cp->isSatisfied (config, err);
        
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
        output = vectorToFloatSeq (config);
        residualError = vectorToFloatSeq (err);
        return success;
      }


      // ---------------------------------------------------------------

      void Problem::computeValueAndJacobian
      (const hpp::floatSeq& config, hpp::floatSeq_out value,
       hpp::floatSeqSeq_out jacobian) throw (hpp::Error)
      {
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
	try {
	  ConfigurationPtr_t configuration = floatSeqToConfigPtr (robot, config, true);
	  vector_t v;
	  matrix_t J;
	  problemSolver()->computeValueAndJacobian (*configuration, v, J);
	  value = vectorToFloatSeq (v);
	  jacobian = matrixToFloatSeqSeq (J);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      bool Problem::generateValidConfig (ULong maxIter,
				      hpp::floatSeq_out output,
				      double& residualError)
	throw (hpp::Error)
      {
        DevicePtr_t robot = problemSolver()->robot ();
	if (!robot) throw hpp::Error ("No robot loaded");
        if (!problemSolver()->problem ()) problemSolver()->resetProblem ();
        core::ConfigurationShooterPtr_t shooter = problemSolver()->problem()->configurationShooter();
	bool success = false, configIsValid = false;
        ConfigurationPtr_t config;
        while (!configIsValid && maxIter > 0)
        {
          try {
            config = shooter->shoot ();
            success = problemSolver()->constraints ()->apply (*config);
            if (hpp::core::ConfigProjectorPtr_t configProjector =
                problemSolver()->constraints ()->configProjector ()) {
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
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
	try {
	  problemSolver()->resetConstraints ();
	  problemSolver()->robot ()->controlComputation
	    (pinocchio::JOINT_POSITION);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::filterCollisionPairs () throw (hpp::Error)
      {
        try {
          problemSolver()->filterCollisionPairs ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::addPassiveDofs (const char* passiveDofsName,
          const hpp::Names_t& dofNames)
        throw (hpp::Error)
      {
        DevicePtr_t robot = problemSolver()->robot ();
        if (!robot) throw hpp::Error ("No robot loaded");
        std::vector <size_type> dofs;
        /// First, translate names into velocity indexes.
        for (CORBA::ULong i=0; i<dofNames.length (); ++i) {
          std::string name (dofNames[i]);
          JointPtr_t j = robot->getJointByName (name);
          if (!j) {
	    std::ostringstream oss;
	    oss << "Joint " << name << " not found.";
            throw hpp::Error (oss.str ().c_str ());
	  }
          for (size_type i = 0; i < j->numberDof (); i++)
            dofs.push_back (j->rankInVelocity() + i);
        }
        /// Then, sort and remove non duplicated elements.
        std::sort (dofs.begin (), dofs.end ());
        std::vector <size_type>::iterator last =
          std::unique (dofs.begin (), dofs.end ());
        dofs.erase (last, dofs.end ());
        /// Then, create the intervals.
        core::segments_t passiveDofs;
        dofs.push_back (robot->numberDof () + 1);
        size_type intStart = dofs[0], intEnd = dofs[0];
        for (size_t i = 1; i < dofs.size (); i++) {
          intEnd ++;
          if (intEnd == dofs[i]) {
            continue;
          } else {
            passiveDofs.push_back (
                core::segment_t (intStart, intEnd - intStart));
            intStart = intEnd = dofs[i];
          }
        }
        problemSolver()->passiveDofs.add (passiveDofsName, passiveDofs);
      }

      // ---------------------------------------------------------------

      void Problem::getConstraintDimensions (const char* constraintName,
            ULong& inputSize , ULong& inputDerivativeSize,
            ULong& outputSize, ULong& outputDerivativeSize)
        throw (hpp::Error)
      {
        try {
          std::string n (constraintName);
          if (!problemSolver()->numericalConstraints.has(n))
            throw Error (("Constraint " + n + " not found").c_str());
          ImplicitPtr_t c =
            problemSolver()->numericalConstraints.get(n);
          inputSize            = (ULong)c->function().inputSize();
          inputDerivativeSize  = (ULong)c->function().inputDerivativeSize();
          outputSize           = (ULong)c->function().outputSize();
          outputDerivativeSize = (ULong)c->function().outputDerivativeSize();
        } catch (const std::exception& e) {
          throw hpp::Error (e.what ());
        }
      }

      // ---------------------------------------------------------------

      void Problem::setConstantRightHandSide (const char* constraintName,
					      CORBA::Boolean constant)
	throw (hpp::Error)
      {
	try {
	  if (constant) {
	    problemSolver()->comparisonType (constraintName,
					     constraints::EqualToZero);
	  } else {
	    problemSolver()->comparisonType (constraintName,
					     constraints::Equality);
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
          ImplicitPtr_t nc
            (problemSolver ()->numericalConstraints.get (constraintName));
	  return nc->constantRightHandSide ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::setRightHandSide (const hpp::floatSeq& rhs)
          throw (hpp::Error)
      {
        try {
          hpp::core::ConfigProjectorPtr_t configProjector
            (problemSolver ()->constraints ()->configProjector ());
          if (!configProjector) {
            throw std::runtime_error ("No constraint has been set.");
          }
          vector_t rightHandSide (floatSeqToVector (rhs));
          configProjector->rightHandSide (rightHandSide);
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      void Problem::setRightHandSideFromConfig (const hpp::floatSeq& config)
          throw (hpp::Error)
      {
        try {
          hpp::core::ConfigProjectorPtr_t configProjector
            (problemSolver ()->constraints ()->configProjector ());
          if (!configProjector) {
            throw std::runtime_error ("No constraint has been set.");
          }
          Configuration_t q (floatSeqToConfig (problemSolver()->robot(), config, true));
          configProjector->rightHandSideFromConfig (q);
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      void Problem::setRightHandSideFromConfigByName (const char* constraintName,
                                                      const hpp::floatSeq& config)
        throw (hpp::Error)
      {
        try {
          hpp::core::ConfigProjectorPtr_t configProjector
            (problemSolver ()->constraints ()->configProjector ());
          if (!configProjector) {
            throw std::runtime_error ("No constraint has been set.");
          }
          Configuration_t q (floatSeqToConfig (problemSolver()->robot(), config, true));
          // look for constraint with this key
          if (problemSolver ()->numericalConstraints.has (constraintName)) {
            ImplicitPtr_t nc
              (problemSolver ()->numericalConstraints.get (constraintName));
            configProjector->rightHandSideFromConfig (nc, q);
            return;
          }
          if (problemSolver ()->lockedJoints.has (constraintName)) {
            LockedJointPtr_t lj
              (problemSolver ()->lockedJoints.get (constraintName));
            configProjector->rightHandSideFromConfig (lj, q);
            return;
          }
          std::string msg ("Config projector does not contain any constraint or locked joint with name ");
          msg += constraintName;
          throw std::runtime_error (msg.c_str ());
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      void Problem::setRightHandSideByName (const char* constraintName,
                                            const hpp::floatSeq& rhs)
        throw (hpp::Error)
      {
        try {
          hpp::core::ConfigProjectorPtr_t configProjector
            (problemSolver ()->constraints ()->configProjector ());
          if (!configProjector) {
            throw std::runtime_error ("No constraint has been set.");
          }
          vector_t rightHandSide (floatSeqToVector (rhs));
          // look for constraint with this key
          if (problemSolver ()->numericalConstraints.has (constraintName)) {
            ImplicitPtr_t nc
              (problemSolver ()->numericalConstraints.get (constraintName));
            configProjector->rightHandSide (nc, rightHandSide);
            return;
          }
          if (problemSolver ()->lockedJoints.has (constraintName)) {
            LockedJointPtr_t lj
              (problemSolver ()->lockedJoints.get (constraintName));
            configProjector->rightHandSide (lj, rightHandSide);
            return;
          }
          std::string msg ("Config projector does not contain any constraint or locked joint with name ");
          msg += constraintName;
          throw std::runtime_error (msg.c_str ());
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::getRightHandSide ()
        throw (hpp::Error)
      {
        try {
          hpp::core::ConfigProjectorPtr_t configProjector
            (problemSolver ()->constraints ()->configProjector ());
          if (!configProjector) {
            throw std::runtime_error ("No constraint has been set.");
          }
          return vectorToFloatSeq (configProjector->rightHandSide());
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      void Problem::addNumericalConstraints
      (const char* configProjName, const Names_t& constraintNames,
       const hpp::intSeq& priorities)
	throw (Error)
      {
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
	try {
	  for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
	    std::string name (constraintNames [i]);
            problemSolver()->addNumericalConstraintToConfigProjector
              (configProjName, name,
                (std::size_t)priorities[i]);
	    problemSolver()->robot ()->controlComputation
	      (pinocchio::COMPUTE_ALL);
	  }
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::setNumericalConstraintsLastPriorityOptional (const bool optional)
	throw (Error)
      {
	if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
        if (!problemSolver()->constraints())
          throw hpp::Error ("The problem has no constraints");
        if (!problemSolver()->constraints()->configProjector())
          throw hpp::Error ("The problem has no config projector");
	try {
          problemSolver()->constraints()->configProjector()->lastIsOptional(optional);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::addLockedJointConstraints
      (const char* configProjName,const hpp::Names_t& lockedJointNames)
        throw (Error)
      {
        if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
	try {
	  for (CORBA::ULong i=0; i<lockedJointNames.length (); ++i) {
	    std::string name (lockedJointNames [i]);
            problemSolver()->addLockedJointToConfigProjector
              (configProjName, name);
	    problemSolver()->robot ()->controlComputation
	      (pinocchio::COMPUTE_ALL);
	  }
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      char* Problem::displayConstraints () throw (Error)
      {
        try {
          std::ostringstream oss;
          if (problemSolver()->constraints())
            oss << *problemSolver()->constraints();
          return c_str(oss.str());
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      Double Problem::getErrorThreshold () throw (Error)
      {
	return problemSolver()->errorThreshold ();
      }

      // ---------------------------------------------------------------

      typedef std::map<std::string, core::ConfigProjector::LineSearchType> LineSearchTypeMap_t;

      LineSearchTypeMap_t makeLineSearchTypeMap ()
      {
        LineSearchTypeMap_t map;
        map["Backtracking"] = core::ConfigProjector::Backtracking;
        map["FixedSequence"] = core::ConfigProjector::FixedSequence;
        map["Constant"] = core::ConfigProjector::Constant;
        map["ErrorNormBased"] = core::ConfigProjector::ErrorNormBased;
        return map;
      }
      void Problem::setDefaultLineSearchType (const char* type) throw (Error)
      {
        static const LineSearchTypeMap_t map = makeLineSearchTypeMap();
        std::string t (type);

        LineSearchTypeMap_t::const_iterator _t = map.find(t);
        if (_t != map.end())
          core::ConfigProjector::defaultLineSearch (_t->second);
        else {
          std::ostringstream oss;
          oss << "Available types are:";
          for (_t = map.begin(); _t != map.end(); ++_t)
            oss << " \"" << _t->first << '"';
          throw Error (oss.str().c_str());
        }
      }

      // ---------------------------------------------------------------

      void Problem::setErrorThreshold (Double threshold) throw (Error)
      {
	problemSolver()->errorThreshold (threshold);
      }

      // ---------------------------------------------------------------
      void Problem::setMaxIterProjection (ULong iterations) throw (Error)
      {
	problemSolver()->maxIterProjection ((size_type)iterations);
      }

      // ---------------------------------------------------------------
      ULong Problem::getMaxIterProjection () throw (Error)
      {
	return (ULong) problemSolver()->maxIterProjection ();
      }

      // ---------------------------------------------------------------
      void Problem::setMaxIterPathPlanning (ULong iterations) throw (Error)
      {
	problemSolver()->maxIterPathPlanning ((size_type)iterations);
      }

      // ---------------------------------------------------------------
      ULong Problem::getMaxIterPathPlanning () throw (Error)
      {
	return (ULong) problemSolver()->maxIterPathPlanning ();
      }

      // ---------------------------------------------------------------

      void Problem::selectPathPlanner (const char* pathPlannerType)
	throw (Error)
      {
	try {
	  problemSolver()->pathPlannerType (std::string (pathPlannerType));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::selectConfigurationShooter (const char* configurationShooterType)
    throw (Error)
      {
    try {
      problemSolver()->configurationShooterType (std::string (configurationShooterType));
    } catch (const std::exception& exc) {
      throw hpp::Error (exc.what ());
    }
      }

      // ---------------------------------------------------------------

      void Problem::selectDistance (const char* distanceType)
    throw (Error)
      {
    try {
      problemSolver()->distanceType(std::string (distanceType));
    } catch (const std::exception& exc) {
      throw hpp::Error (exc.what ());
    }
      }

      // ---------------------------------------------------------------

      void Problem::selectSteeringMethod (const char* steeringMethodType)
    throw (Error)
      {
    try {
      problemSolver()->steeringMethodType (std::string (steeringMethodType));
    } catch (const std::exception& exc) {
      throw hpp::Error (exc.what ());
    }
      }

      // ---------------------------------------------------------------
      void Problem::addPathOptimizer (const char* pathOptimizerType)
	throw (Error)
      {
	try {
	  problemSolver()->addPathOptimizer (std::string (pathOptimizerType));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::clearPathOptimizers () throw (Error)
      {
	try {
	  problemSolver()->clearPathOptimizers ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------
      void Problem::addConfigValidation (const char* configValidationType)
	throw (Error)
      {
	try {
	  problemSolver()->addConfigValidation (std::string (configValidationType));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::clearConfigValidations () throw (Error)
      {
	try {
	  problemSolver()->clearConfigValidations ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::selectPathValidation (const char* pathValidationType,
					  Double tolerance) throw (Error)
      {
	try {
	  problemSolver()->pathValidationType (std::string (pathValidationType),
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
          problemSolver()->pathProjectorType (std::string (pathProjectorType),
                                             tolerance);
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      bool Problem::prepareSolveStepByStep () throw (hpp::Error)
      {
	try {
	  return problemSolver()->prepareSolveStepByStep ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
        return false;
      }

      // ---------------------------------------------------------------

      bool Problem::executeOneStep () throw (hpp::Error)
      {
	try {
	  return problemSolver()->executeOneStep ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
        return false;
      }

      // ---------------------------------------------------------------

      void Problem::finishSolveStepByStep () throw (hpp::Error)
      {
	try {
         problemSolver()->finishSolveStepByStep();
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
	  problemSolver()->solve();
          boost::posix_time::time_duration time =
            boost::posix_time::microsec_clock::universal_time () - start;
          hpp::intSeq *ret = new hpp::intSeq;
          ret->length (4);
          (*ret)[0] = time.hours ();
          (*ret)[1] = time.minutes ();
          (*ret)[2] = time.seconds ();
          (*ret)[3] = (long) ((int) time.fractional_seconds () / 1000);
          return ret;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      bool Problem::directPath (const hpp::floatSeq& startConfig,
				const hpp::floatSeq& endConfig,
				CORBA::Boolean validate, ULong& pathId,
				CORBA::String_out report)
	throw (hpp::Error)
      {
	ConfigurationPtr_t start;
	ConfigurationPtr_t end;
	bool pathValid = false;
        DevicePtr_t robot = getRobotOrThrow(problemSolver());
	try {
	  start = floatSeqToConfigPtr (robot, startConfig, true);
	  end = floatSeqToConfigPtr (robot, endConfig, true);
	  if (!problemSolver()->problem ()) {
	    problemSolver()->resetProblem ();
	  }
          std::size_t pid;
	  std::string r;
	  pathValid = problemSolver()->directPath (*start, *end, validate,
						   pid, r);
	  report = CORBA::string_dup(r.c_str ());
          pathId = (ULong) pid;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	return pathValid;
      }

      // ---------------------------------------------------------------

      void  Problem::addConfigToRoadmap (const hpp::floatSeq& config)
	throw (hpp::Error)
      {
        try {
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
          ConfigurationPtr_t configuration
            (floatSeqToConfigPtr (robot, config, true));
          problemSolver()->addConfigToRoadmap (configuration);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::addEdgeToRoadmap (const hpp::floatSeq& config1,
				      const hpp::floatSeq& config2,
				      ULong pathId, bool bothEdges)
	throw (hpp::Error)
      {
        try {
	  if (pathId >= problemSolver()->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver()->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
          PathVectorPtr_t path = problemSolver()->paths () [pathId];
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  ConfigurationPtr_t start (floatSeqToConfigPtr (robot, config1, true));
	  ConfigurationPtr_t finish (floatSeqToConfigPtr (robot, config2, true));
	  if (bothEdges) {
	    problemSolver()->addEdgeToRoadmap (start, finish, path);
	    problemSolver()->addEdgeToRoadmap (finish, start, path->reverse());
	    return;
	  }
	  problemSolver()->addEdgeToRoadmap (start, finish, path);
	  return;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
       }
      // ---------------------------------------------------------------

      void Problem::appendDirectPath (ULong pathId,
				      const hpp::floatSeq& config)
	throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver()->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver()->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  PathVectorPtr_t path = problemSolver()->paths () [pathId];
	  Configuration_t start (path->end ());
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  ConfigurationPtr_t end (floatSeqToConfigPtr (robot, config, true));
	  if (!problemSolver()->problem ()) {
	    problemSolver()->resetProblem ();
	  }
	  SteeringMethodPtr_t sm
	    (problemSolver()->problem ()->steeringMethod ());
	  PathPtr_t dp = (*sm) (start, *end);
          if (!dp) {
            std::ostringstream oss;
            oss << "steering method failed to build a path between q1="
                << pinocchio::displayConfig (start) << "; q2="
                << pinocchio::displayConfig (*end) << ".";
            throw std::runtime_error (oss.str ().c_str ());
          }
	  PathPtr_t unused;
	  PathValidationReportPtr_t report;
	  if (!problemSolver()->problem()->pathValidation ()->validate
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

      void Problem::concatenatePath (ULong startId, ULong endId)
	throw (hpp::Error)
      {
	try {
          if ( startId >= problemSolver()->paths ().size ()
              || endId >= problemSolver()->paths ().size ()) {
	    std::ostringstream oss ("wrong path id. ");
	    oss << "Number path: " << problemSolver()->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  PathVectorPtr_t start = problemSolver()->paths () [startId];
	  PathVectorPtr_t end   = problemSolver()->paths () [  endId];
          start->concatenate(end);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::erasePath (ULong pathId)
	throw (hpp::Error)
      {
	try {
          if (pathId >= problemSolver()->paths ().size ()) {
	    std::ostringstream oss ("wrong path id. ");
	    oss << "Number path: " << problemSolver()->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  problemSolver()->erasePath(pathId);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      bool Problem::projectPath (ULong pathId)
	throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver()->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver()->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  PathVectorPtr_t initial = problemSolver()->paths () [pathId];
          core::PathProjectorPtr_t pp = problemSolver()->problem ()->pathProjector ();
          if (!pp) throw Error ("There is no path projector");

          PathPtr_t proj;
          bool success = pp->apply (initial, proj);

	  PathVectorPtr_t path
	    (core::PathVector::create (initial->outputSize (),
				       initial->outputDerivativeSize ()));
	  path->appendPath (proj);
	  problemSolver()->addPath (path);
          return success;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::interruptPathPlanning() throw (hpp::Error)
      {
	problemSolver()->interrupt ();
      }

      // ---------------------------------------------------------------

      Long Problem::numberPaths () throw (hpp::Error)
      {
	try {
	  return (Long) problemSolver()->paths ().size ();
	} catch (std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::intSeq* Problem::optimizePath(ULong pathId) throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver()->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver()->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
          // Start timer
          boost::posix_time::ptime start =
            boost::posix_time::microsec_clock::universal_time ();

	  PathVectorPtr_t initial = problemSolver()->paths () [pathId];
	  problemSolver()->optimizePath (initial);

          // Stop timer
          boost::posix_time::time_duration time =
            boost::posix_time::microsec_clock::universal_time () - start;

          hpp::intSeq *ret = new hpp::intSeq;
          ret->length (4);
          (*ret)[0] = time.hours ();
          (*ret)[1] = time.minutes ();
          (*ret)[2] = time.seconds ();
          (*ret)[3] = (long) ((int) time.fractional_seconds () / 1000);
          return ret;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      Double Problem::pathLength (ULong pathId) throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver()->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver()->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  return problemSolver()->paths () [pathId]->length ();
	} catch (std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::configAtParam (ULong pathId,
					     Double atDistance)
	throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver()->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver()->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  PathPtr_t path = problemSolver()->paths () [pathId];
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

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::derivativeAtParam (ULong pathId,
						 ULong order,
					         Double atDistance)
	throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver()->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver()->paths ().size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  PathPtr_t path = problemSolver()->paths () [pathId];
	  vector_t velocity (problemSolver ()->robot ()->numberDof ());
	  path->derivative (velocity, atDistance, order);
          return vectorToFloatSeq (velocity);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------

      hpp::floatSeqSeq* Problem::getWaypoints (ULong pathId,
                                               hpp::floatSeq_out times)
	throw (hpp::Error)
      {
	try {
	  if (pathId >= problemSolver()->paths ().size ()) {
	    std::ostringstream oss ("wrong path id: ");
	    oss << pathId << ", number path: "
		<< problemSolver()->paths ().size () << ".";
	    throw std::runtime_error (oss.str ().c_str ());
	  }
	  PathVectorPtr_t path = problemSolver()->paths () [pathId];
          PathVectorPtr_t flat = core::PathVector::create(path->outputSize(), path->outputDerivativeSize());
          path->flatten(flat);
          core::matrix_t points (flat->numberPaths() + 1, path->outputSize());
          core::vector_t ts (flat->numberPaths() + 1);
          ts(0) = flat->timeRange().first;
          for (std::size_t i = 0; i < flat->numberPaths(); ++i) {
            points.row(i) = flat->pathAtRank(i)->initial();
            ts(i+1) = ts(i) + flat->pathAtRank(i)->length();
          }
          points.row(flat->numberPaths()) = flat->end();
          times = vectorToFloatSeq (ts);
	  return matrixToFloatSeqSeq(points);
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
	    (problemSolver()->roadmap ()->nodes ());
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
	return (Long)problemSolver()->roadmap ()->edges ().size ();
      }

      // ---------------------------------------------------------------


      void Problem::edge (ULong edgeId, hpp::floatSeq_out q1,
			  hpp::floatSeq_out q2) throw (hpp::Error)
      {
	try {
	  const Edges_t & edges
	    (problemSolver()->roadmap ()->edges ());
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
	return (Long) problemSolver()->roadmap ()->nodes().size();
      }

      // ---------------------------------------------------------------

      hpp::floatSeq* Problem::node (ULong nodeId) throw (hpp::Error)
      {
      try {
        const Nodes_t & nodes (problemSolver()->roadmap()->nodes());

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
            const Edges_t & edges (problemSolver()->roadmap()->edges());
            if (edges.size() > edgeId) {
              Edges_t::const_iterator itEdge = boost::next(edges.begin(),edgeId);

              const core::ConnectedComponents_t& ccs = problemSolver()->roadmap()
                ->connectedComponents ();
              core::ConnectedComponents_t::const_iterator itcc = ccs.begin();
              for (std::size_t i = 0; i < ccs.size (); ++i) {
                if (*itcc == (*itEdge)->from()->connectedComponent ()) {
                  return (CORBA::Long) i;
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
            const Nodes_t & nodes (problemSolver()->roadmap()->nodes());
            if (nodes.size() > nodeId) {
              Nodes_t::const_iterator itNode = boost::next(nodes.begin(),nodeId);

              const core::ConnectedComponents_t& ccs = problemSolver()->roadmap()
                ->connectedComponents ();
              core::ConnectedComponents_t::const_iterator itcc = ccs.begin();
              for (std::size_t i = 0; i < ccs.size (); ++i) {
                if (*itcc == (*itNode)->connectedComponent ()) {
                  return (CORBA::Long) i;
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
	  (Long) problemSolver()->roadmap ()->connectedComponents ().size ();
      }

      // ---------------------------------------------------------------

      hpp::floatSeqSeq*
      Problem::nodesConnectedComponent (ULong connectedComponentId)
	throw (hpp::Error)
      {
	hpp::floatSeqSeq* res;
	try {
	  const ConnectedComponents_t& connectedComponents
	    (problemSolver()->roadmap ()->connectedComponents ());
	  ConnectedComponents_t::const_iterator itcc =
	    connectedComponents.begin ();
	  ULong i = 0;
	  while (i != connectedComponentId) {
	    ++i; itcc++;
	  }
      const NodeVector_t & nodes ((*itcc)->nodes ());
	  res = new hpp::floatSeqSeq;
	  res->length ((CORBA::ULong)nodes.size ());
	  i=0;
	  for (NodeVector_t::const_iterator itNode = nodes.begin ();
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

      hpp::floatSeq*
      Problem::getNearestConfig (const hpp::floatSeq& config, const Long connectedComponentId,
      				 hpp::core::value_type& distance) throw (hpp::Error)
      {
	hpp::floatSeq* res;
	try {
	  const hpp::core::ConnectedComponents_t& connectedComponents
	        (problemSolver()->roadmap ()->connectedComponents ());
	  hpp::core::NodePtr_t nearest;
          DevicePtr_t robot = getRobotOrThrow(problemSolver());
	  ConfigurationPtr_t configuration = floatSeqToConfigPtr (robot, config, true);
	  if (connectedComponentId < 0) {
	    nearest = problemSolver()->roadmap ()->nearestNode (configuration, distance);
	    	  } else {
	    if ((std::size_t)connectedComponentId >= connectedComponents.size ()) {
	      std::ostringstream oss;
	      oss << "connectedComponentId=" << connectedComponentId
		  << " out of range [0," << connectedComponents.size () - 1
		  << "].";
	      throw std::runtime_error (oss.str ().c_str ());
	    }
	    hpp::core::ConnectedComponents_t::const_iterator itcc =
	      connectedComponents.begin ();
            std::advance (itcc, connectedComponentId);
	    nearest = problemSolver()->roadmap ()->nearestNode (configuration, *itcc, distance);
	  }
          if (!nearest) throw hpp::Error ("Nearest node not found");
          res = vectorToFloatSeq (*(nearest->configuration ()));
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	return res;
      }

      // ---------------------------------------------------------------

      void Problem::clearRoadmap () throw (hpp::Error)
      {
	try {
	  problemSolver()->roadmap ()->clear ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::resetRoadmap ()
      {
	try {
	  problemSolver()->resetRoadmap ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Problem::saveRoadmap (const char* filename)
        throw (hpp::Error)
      {
        try {
          std::ofstream ofs (filename, std::ofstream::out);
          hpp::core::parser::writeRoadmap (ofs, problemSolver()->problem(),
              problemSolver()->roadmap());
          ofs.close ();
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      void Problem::readRoadmap (const char* filename)
        throw (hpp::Error)
      {
        try {
          hpp::core::ProblemSolverPtr_t p = problemSolver();
          hpp::core::parser::readRoadmap (std::string(filename),
              p->roadmap(), p->problem());
          // ProblemSolver should be update to use the init and goal nodes of
          // the roadmap
          if (p->roadmap()->initNode())
            p->initConfig(p->roadmap()->initNode()->configuration());
          const hpp::core::NodeVector_t& goals = p->roadmap()->goalNodes();
          p->resetGoalConfigs();
          for (hpp::core::NodeVector_t::const_iterator _goals = goals.begin();
              _goals != goals.end(); ++_goals)
            p->addGoalConfig ((*_goals)->configuration());
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      // ---------------------------------------------------------------

      void Problem::scCreateScalarMultiply
      (const char* outName, Double scalar, const char* inName)
        throw (hpp::Error)
      {
        if (!problemSolver()->robot ()) throw hpp::Error ("No robot loaded");
        try {
          std::string inN (inName);
          std::string outN (outName);
          if (!problemSolver()->numericalConstraints.has (inN))
            throw Error (("Constraint " + inN + " not found").c_str());
          ImplicitPtr_t in = problemSolver()->numericalConstraints.get (inN);
          typedef constraints::FunctionExp<constraints::DifferentiableFunction> FunctionExp_t;
          typedef constraints::ScalarMultiply<FunctionExp_t> Expr_t;
          constraints::DifferentiableFunctionPtr_t out =
            constraints::SymbolicFunction<Expr_t>::create (outN, problemSolver()->robot(),
                constraints::Traits<Expr_t>::Ptr_t (new Expr_t (scalar,
                    FunctionExp_t::create (in->functionPtr())))
                );

          problemSolver()->addNumericalConstraint
            (outN, Implicit::create (out, in->comparisonType()));
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

    } // namespace impl
  } // namespace corbaServer
} // namespace hpp
