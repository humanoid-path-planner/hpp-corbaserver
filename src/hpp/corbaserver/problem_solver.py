#!/usr/bin/env python
# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-corbaserver.
# hpp-corbaserver is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-corbaserver is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-corbaserver.  If not, see
# <http://www.gnu.org/licenses/>.

def newProblem (client = None, name = None):
    if client is None:
        from hpp.corbaserver import Client
        client = Client()
    if name is not None:
        if not client.problem.selectProblem(name):
            # if not created, reset it.
            client.problem.resetProblem()
    else:
        client.problem.resetProblem()

def _convertToCorbaAny (value):
    from sys import version_info
    if version_info.major > 2:
        integers = (int,)
    else:
        integers = (long,int,)
    import CORBA
    t = type(value)
    if t is float:
        return CORBA.Any(CORBA.TC_double, value)
    elif isinstance(value, bool):
        return CORBA.Any(CORBA.TC_boolean, value)
    elif isinstance(value, integers):
        return CORBA.Any(CORBA.TC_longlong, value)
    elif isinstance(value, str):
        return CORBA.Any(CORBA.TC_string, value)
    elif isinstance (value, (list, tuple)):
        if isinstance (value[0], (list, tuple)):
            return CORBA.Any(CORBA.TypeCode("IDL:hpp/floatSeqSeq:1.0"), value)
        else:
            return CORBA.Any(CORBA.TypeCode("IDL:hpp/floatSeq:1.0"), value)
    else: # Assume value is already a CORBA.Any
        return value

## Definition of a path planning problem
#
#  This class wraps the Corba client to the server implemented by
#  libhpp-corbaserver.so
#
#  Some method implemented by the server can be considered as private. The
#  goal of this class is to hide them and to expose those that can be
#  considered as public.
class ProblemSolver (object):
    def __init__ (self, robot, hppcorbaClient = None):
        self.client = robot.client
        self.hppcorba = robot.client if hppcorbaClient is None else hppcorbaClient
        self.robot = robot

    ## Load a plugin into the current ProblemSolver.
    #  \param pluginName either an absolute filename or a filename relative
    #                    to `<a_path_in_LD_LIBRARY_PATH>/hppPlugins`.
    #  \note This is reset each time resetProblem is called.
    def loadPlugin (self, pluginName):
        return self.hppcorba.problem.loadPlugin (pluginName)

    ## Set random seed of random number generator
    def setRandomSeed (self, seed):
        return self.hppcorba.problem.setRandomSeed (seed)

    ## Set the maximum number of threads.
    #  This parameter defines the number of possible concurrent
    #  access to the device.
    def setMaxNumThreads (self, n):
        return self.hppcorba.problem.setMaxNumThreads (n)

    ## Get the maximum number of concurrent
    #  access to the device (in thread safe areas).
    def getMaxNumThreads (self):
        return self.hppcorba.problem.getMaxNumThreads ()

    ## Return a list of available elements of type type
    #  \param type enter "type" to know what types I know of.
    #              This is case insensitive.
    def getAvailable (self, type):
        return self.hppcorba.problem.getAvailable (type)

    ## Return a list of selected elements of type type
    #  \param type enter "type" to know what types I know of.
    #              This is case insensitive.
    #  \note For most of the types, the list will contain only one element.
    def getSelected (self, type):
        return self.hppcorba.problem.getSelected (type)

    ## Set a parameter
    #  \param value the input type must be long, double, const char*
    #
    #  \code
    #  import CORBA
    #  ProblemSolver ps(robot)
    #  ps.setParameter ("name", CORBA.Any(CORBA.TC_double, 3.2233))
    #  \endcode
    def setParameter (self, name, value):
        value = _convertToCorbaAny (value)
        return self.hppcorba.problem.setParameter (name, value)

    ## Get parameter with given name
    #  raise an exception when the parameter is not found.
    def getParameter (self, name, keep_any=False):
        any = self.hppcorba.problem.getParameter (name)
        return any if keep_any else any.value()

    ## Get parameter documentation
    #  raise an exception when the parameter is not found.
    def getParameterDoc (self, name):
        return self.hppcorba.problem.getParameterDoc (name)

    #  Select a problem by its name.
    #  If no problem with this name exists, a new problem is created and
    #  selected.
    #  \param name the problem name.
    #  \return true if a new problem was created.
    def selectProblem (self, name):
        return self.hppcorba.problem.selectProblem (name)

    #  Move a path from the current problem to another problem.
    #  \param problemName the destination problem
    #  \param jointNames a list of joint names representing the subchain to
    #         extract from the original path.
    #  \todo the configuration parameter can be selected but not reorganized.
    def movePathToProblem (self, pathId, problemName, jointNames):
        return self.hppcorba.problem.movePathToProblem \
            (pathId, problemName, jointNames)

    ## \name Initial and goal configurations
    # \{

    ## Set initial configuration of specified problem.
    #  \param dofArray Array of degrees of freedom
    #  \throw Error.
    def setInitialConfig (self, dofArray):
        return self.hppcorba.problem.setInitialConfig (dofArray)

    ## Get initial configuration of specified problem.
    #  \return Array of degrees of freedom
    def getInitialConfig (self):
        return self.hppcorba.problem.getInitialConfig ()

    ## Add goal configuration to specified problem.
    #  \param dofArray Array of degrees of freedom
    #  \throw Error.
    def addGoalConfig (self, dofArray):
        return self.hppcorba.problem.addGoalConfig (dofArray)

    ## Get goal configurations of specified problem.
    #  \return Array of degrees of freedom
    def getGoalConfigs (self):
        return self.hppcorba.problem.getGoalConfigs ()

    ## Reset goal configurations
    def resetGoalConfigs (self):
        return self.hppcorba.problem.resetGoalConfigs ()
    ## \}

    ## \name Obstacles
    # \{

    ## Load obstacle from urdf file
    #  \param package Name of the package containing the model,
    #  \param filename name of the urdf file in the package
    #         (without suffix .urdf)
    #  \param prefix prefix added to object names in case the same file is
    #         loaded several times
    #
    #  The ros url is built as follows:
    #  "package://${package}/urdf/${filename}.urdf"
    #
    #  The kinematic structure of the urdf file is ignored. Only the geometric
    #  objects are loaded as obstacles.
    def loadObstacleFromUrdf (self, package, filename, prefix):
        return self.hppcorba.obstacle.loadObstacleModel (package, filename,
                                                       prefix)

    ## Remove an obstacle from outer objects of a joint body
    #
    #  \param objectName name of the object to remove,
    #  \param jointName name of the joint owning the body,
    #  \param collision whether collision with object should be computed,
    #  \param distance whether distance to object should be computed.
    #  \throw Error.
    def removeObstacleFromJoint (self, objectName, jointName, collision,
                                 distance):
        return self.hppcorba.obstacle.removeObstacleFromJoint \
            (objectName, jointName, collision, distance)

    ## Cut the obstacle with the given AABB
    # \param aabb a vector of 6 floats. The 3 first represent one corner
    #             and the 3 last represent the opposite corner.
    def cutObstacle (self, objectName, aabb):
        return self.hppcorba.obstacle.cutObstacle(objectName, aabb)

    ## Move an obstacle to a given configuration.
    #  \param objectName name of the polyhedron.
    #  \param cfg the configuration of the obstacle.
    #  \throw Error.
    #
    #  \note The obstacle is not added to local map
    #  impl::Obstacle::collisionListMap.
    #
    #  \note Build the collision entity of polyhedron for KCD.
    def moveObstacle (self, objectName, cfg):
        return self.hppcorba.obstacle.moveObstacle (objectName, cfg)
    ## Get the position of an obstacle
    #
    #  \param objectName name of the polyhedron.
    #  \retval cfg Position of the obstacle.
    #  \throw Error.
    def getObstaclePosition (self, objectName):
        return self.hppcorba.obstacle.getObstaclePosition (objectName)

    ## Get list of obstacles
    #
    #  \param collision whether to return obstacle for collision,
    #  \param distance whether to return obstacles for distance computation
    # \return list of obstacles
    def getObstacleNames (self, collision, distance):
        return self.hppcorba.obstacle.getObstacleNames (collision, distance)

    def getObstacleLinkPosition (self, objectName):
        return self.hppcorba.obstacle.getObstacleLinkPosition (objectName)

    ## Get list of obstacles
    #
    #  \param collision whether to return obstacle for collision,
    #  \param distance whether to return obstacles for distance computation
    # \return list of obstacles
    def getObstacleLinkNames (self):
        return self.hppcorba.obstacle.getObstacleLinkNames ()

    ##\}

    ## \name Constraints
    #  \{

    ## Create orientation constraint between two joints
    #
    #  \param constraintName name of the constraint created,
    #  \param joint1Name name of first joint
    #  \param joint2Name name of second joint
    #  \param p quaternion representing the desired orientation
    #         of joint2 in the frame of joint1.
    #  \param mask Select which axis to be constrained.
    #  If joint1 or joint2 is "", the corresponding joint is replaced by
    #  the global frame.
    #  constraints are stored in ProblemSolver object
    def createOrientationConstraint (self, constraintName, joint1Name,
                                     joint2Name, p, mask):
        return self.hppcorba.problem.createOrientationConstraint \
            (constraintName, joint1Name, joint2Name, p, mask)

    ## Create transformation constraint between two joints
    #
    #  \param constraintName name of the constraint created,
    #  \param joint1Name name of first joint
    #  \param joint2Name name of second joint
    #  \param ref desired transformation of joint2 in the frame of joint1.
    #  \param mask Select which axis to be constrained.
    #  If joint1 of joint2 is "", the corresponding joint is replaced by
    #  the global frame.
    #  constraints are stored in ProblemSolver object
    def createTransformationConstraint (self, constraintName, joint1Name,
                                        joint2Name, ref, mask) :
        return self.hppcorba.problem.createTransformationConstraint \
            (constraintName, joint1Name, joint2Name, ref, mask)

    ## Create a LockedJoint constraint with given value
    #  \param lockedJointName key of the constraint in the ProblemSolver map,
    #  \param jointName name of the joint,
    #  \param value value of the joint configuration,
    def createLockedJoint (self, lockedDofName, jointName, value):
        return self.hppcorba.problem.createLockedJoint \
            (lockedDofName, jointName, value)

    ## Create a locked extradof
    #         hpp::manipulation::ProblemSolver map
    #  \param lockedDofName key of the constraint in the Problem Solver map
    #  \param index index of the extra dof (0 means the first extra dof)
    #  \param value value of the extra dof configuration. The size
    #               of this vector defines the size of the constraints.
    def createLockedExtraDof (self, lockedDofName, index, value):
        return self.hppcorba.problem.createLockedExtraDof \
            (lockedDofName, index, value)

    ## Create RelativeCom constraint between two joints
    #
    #  \param constraintName name of the constraint created,
    #  \param comName name of CenterOfMassComputation
    #  \param jointName name of joint
    #  \param point point in local frame of joint.
    #  \param mask Select axis to be constrained.
    #  If jointName is "", the robot root joint is used.
    #  Constraints are stored in ProblemSolver object
    def createRelativeComConstraint (self, constraintName, comName, jointLName, point, mask):
        return self.hppcorba.problem.createRelativeComConstraint \
            (constraintName, comName, jointLName, point, mask)

    ## Create ComBeetweenFeet constraint between two joints
    #
    #  \param constraintName name of the constraint created,
    #  \param comName name of CenterOfMassComputation
    #  \param jointLName name of first joint
    #  \param jointRName name of second joint
    #  \param pointL point in local frame of jointL.
    #  \param pointR point in local frame of jointR.
    #  \param jointRefName name of second joint
    #  \param mask Select axis to be constrained.
    #  If jointRef is "", the robot root joint is used.
    #  Constraints are stored in ProblemSolver object
    def createComBeetweenFeet (self, constraintName, comName, jointLName, jointRName,
        pointL, pointR, jointRefName, mask):
        return self.hppcorba.problem.createComBeetweenFeet \
            (constraintName, comName, jointLName, jointRName, pointL, pointR, jointRefName, mask)

    ## Add an object to compute a partial COM of the robot.
    # \param name of the partial com
    # \param jointNames list of joint name of each tree ROOT to consider.
    # \note Joints are added recursively, it is not possible so far to add a
    # joint without addind all its children.
    def addPartialCom (self, comName, jointNames):
        return self.hppcorba.robot.addPartialCom (comName, jointNames);

    def getPartialCom (self, comName):
        return self.hppcorba.robot.getPartialCom (comName);

    ## Create position constraint between two joints
    #
    #  \param constraintName name of the constraint created,
    #  \param joint1Name name of first joint
    #  \param joint2Name name of second joint
    #  \param point1 point in local frame of joint1,
    #  \param point2 point in local frame of joint2.
    #  \param mask Select which axis to be constrained.
    #  If joint1 of joint2 is "", the corresponding joint is replaced by
    #  the global frame.
    #  constraints are stored in ProblemSolver object
    def createPositionConstraint (self, constraintName, joint1Name,
                                  joint2Name, point1, point2, mask):
        return self.hppcorba.problem.createPositionConstraint \
            (constraintName, joint1Name, joint2Name, point1, point2, mask)

    ## Create distance constraint between robot objects
    #
    #  \param constraintName name of the constraint created,
    #  \param joint1Name name of first joint,
    #  \param joint2Name name of second joint,
    #  \param distance desired distance between joint bodies.
    #  Constraints are stored in ProblemSolver object
    def createDistanceBetweenJointConstraint (self, constraintName, joint1Name,\
                                              joint2Name, distance) :
        return self.hppcorba.problem.createDistanceBetweenJointConstraint \
            (constraintName, joint1Name, joint2Name, distance)

    ## Create distance constraint between robot and environment objects
    #
    #  \param constraintName name of the constraint created,
    #  \param joint1Name name of first joint,
    #  \param objects names of environment objects,
    #  \param distance desired distance between joint bodies.
    #  Constraints are stored in ProblemSolver object
    def createDistanceBetweenJointAndObjects (self, constraintName, joint1Name,\
                                              objects, distance) :
        return self.hppcorba.problem.createDistanceBetweenJointAndObjects \
            (constraintName, joint1Name, objects, distance)

    ## Reset Constraints
    #
    #  Reset all constraints, including numerical constraints and locked
    #  joints
    def resetConstraints (self):
        return self.hppcorba.problem.resetConstraints ()

    ## Delete all the constraint in the ProblemSolver map.
    #  It erases both the numerical and locked joint constraints
    #  This does not resetConstraints of the problem to be solved.
    def resetConstraintMap (self):
        return self.hppcorba.problem.resetConstraintMap ()

    ## Add numerical constraints in ConfigProjector
    #
    #  \param name name of the config projector created if any,
    #  \param names list of names of the numerical constraints previously
    #         created by methods createTransformationConstraint,
    #         createRelativeComConstraint, ...
    def addNumericalConstraints (self, name, names, priorities = None):
        if priorities is None:
            priorities = [ 0 for i in names ]
        return self.hppcorba.problem.addNumericalConstraints \
            (name, names, priorities)

    ## Add locked joint in ConfigProjector
    #
    #  \param name name of the config projector created if any,
    #  \param names list of names of the locked joints previously created by
    #         method createLockedJoint.
    def addLockedJointConstraints (self, name, names):
        return self.hppcorba.problem.addLockedJointConstraints (name, names)

    ## Set right hand side of constraints in config projector
    #  \param rhs right hand side of constraints. Contains only right hand side
    #         of non-constant constraints
    #  \note Locked joints are also considered.
    def setRightHandSide (self, rhs):
        return self.hppcorba.problem.setRightHandSide (rhs)

    ## Set right hand side of given constraint in config projector
    #  \param constraintName name of the numerical constraint or locked joint
    #  \param rhs right hand side of constraint. raises an exception if
    #         constraint has constant right hand side.
    def setRightHandSideByName (self, constraintName, rhs):
        return self.hppcorba.problem.setRightHandSideByName (constraintName, rhs)

    ## Set right hand side of constraints in config projector
    #  \param config a robot configuration use to compute the right hand side
    #         of constraints. Contains only right hand side of non-constant
    #         constraints
    #  \note Locked joints are also considered.
    def setRightHandSideFromConfig (self, config):
        return self.hppcorba.problem.setRightHandSideFromConfig (config)

    ## Set right hand side of given constraint in config projector
    #  \param constraintName name of the numerical constraint or locked joint
    #  \param config a robot configuration use to compute the right hand side.
    #         raises an exception if constraint has constant right hand side.
    def setRightHandSideFromConfigByName (self, constraintName, config):
        return self.hppcorba.problem.setRightHandSideFromConfigByName (constraintName, config)


    ## Apply constraints
    #
    #  \param q initial configuration
    #  \return configuration projected in success,
    #  \throw Error if projection failed.
    def applyConstraints (self, q):
        return self.hppcorba.problem.applyConstraints (q)

    ## Find a local minimum of the least priority constraints
    #
    # while respecting the other priorities.
    # \param input input configuration,
    # \return output output configuration,
    # \return residualError.
    def optimize (self, q):
        return self.hppcorba.problem.optimize (q)

    ## Compute value and Jacobian of numerical constraints
    #
    #  \param q input configuration
    #  \return value values of the numerical constraints stacked in a unique
    #          vector,
    #  \return Jacobian of the numerical constraints stacked in a unique
    #          matrix.
    #
    #  Columns of the Jacobian corresponding to locked joints are omitted,
    #  columns corresponding to passive dofs are set to 0.
    def computeValueAndJacobian (self, q):
        return self.hppcorba.problem.computeValueAndJacobian (q)

    ## Create a vector of passive dofs.
    #
    #  \param name name of the vector in the ProblemSolver map.
    #  \param dofNames list of names of DOF that may
    def addPassiveDofs (self, name, dofNames):
        return self.hppcorba.problem.addPassiveDofs (name, dofNames)

    ## (Dis-)Allow to modify right hand side of a numerical constraint
    #  \param constraintName Name of the numerical constraint,
    #  \param constant whether right hand side is constant
    #
    #  Constraints should have been added in the ProblemSolver local map,
    #  but not inserted in the config projector.
    def setConstantRightHandSide (self, constraintName, constant) :
        return self.hppcorba.problem.setConstantRightHandSide (constraintName,
                                                             constant)

    ## Get whether right hand side of a numerical constraint is constant
    #  \param constraintName Name of the numerical constraint,
    #  \return whether right hand side is constant
    def getConstantRightHandSide (self, constraintName) :
        return self.hppcorba.problem.getConstantRightHandSide (constraintName)

    ## Generate a configuration satisfying the constraints
    #
    #  \param maxIter maximum number of tries,
    #  \return configuration projected in success,
    #  \throw Error if projection failed.
    def generateValidConfig (self, maxIter):
        return self.hppcorba.problem.generateValidConfig (maxIter)

    ## error threshold in numerical constraint resolution
    def getErrorThreshold (self):
        return self.hppcorba.problem.getErrorThreshold ()

    ## error threshold in numerical constraint resolution
    def setErrorThreshold (self, threshold):
        return self.hppcorba.problem.setErrorThreshold (threshold)

    ## set the default line search type used in the projection.
    # \param See hpp::core::ConfigProjector::LineSearchType for possible values.
    def setDefaultLineSearchType (self, type):
        return self.hppcorba.problem.setDefaultLineSearchType (type)

    ## Get the maximal number of iterations in projection
    def getMaxIterPathPlanning (self):
        return self.hppcorba.problem.getMaxIterPathPlanning ()

    ## Set the maximal number of iterations in projection
    def setMaxIterPathPlanning (self, iterations):
        return self.hppcorba.problem.setMaxIterPathPlanning (iterations)

    ## Get time out in path planning (in seconds)
    def getTimeOutPathPlanning (self):
        return self.hppcorba.problem.getTimeOutPathPlanning ()

    ## Set time out in path planning (in seconds)
    def setTimeOutPathPlanning (self, timeOut):
        return self.hppcorba.problem.setTimeOutPathPlanning (timeOut)

    ## Get the maximal number of iterations in projection
    def getMaxIterProjection (self):
        return self.hppcorba.problem.getMaxIterProjection ()

    ## Set the maximal number of iterations in projection
    def setMaxIterProjection (self, iterations):
        return self.hppcorba.problem.setMaxIterProjection (iterations)
    ## \}

    ## \name Collision Checking
    # \{

    ## Build matrix of relative motions between joints
    #
    #  See hpp::core::Problem::filterCollisionPairs.
    def filterCollisionPairs (self):
        return self.hppcorba.problem.filterCollisionPairs ()
    ## \}

    ## \name Solve problem and get paths
    # \{

    ## Select path planner type
    #  \param Name of the path planner type, either "DiffusingPlanner",
    #         "VisibilityPrmPlanner", or any type added by method
    #         core::ProblemSolver::addPathPlannerType
    def selectPathPlanner (self, pathPlannerType):
        return self.hppcorba.problem.selectPathPlanner (pathPlannerType)

    ## Select configuration shooter type
    #  \param Name of the configuration shooter type
    #  \note the configuration shooter is created and initialized
    #        when calling this method. This might be important if the
    #        initialization depends on the current state of the robot.
    def selectConfigurationShooter (self, configurationShooterType):
        return self.hppcorba.problem.selectConfigurationShooter \
            (configurationShooterType)

    ## Add a path optimizer
    #  \param Name of the path optimizer type, either "RandomShortcut" or
    #   any type added by core::ProblemSolver::addPathOptimizerType
    def addPathOptimizer (self, pathOptimizerType):
        return self.hppcorba.problem.addPathOptimizer (pathOptimizerType)

    ## Clear sequence of path optimizers
    #
    def clearPathOptimizers (self):
        return self.hppcorba.problem.clearPathOptimizers ()

    ## Select path validation method
    #  \param Name of the path validation method, either "Discretized"
    #  "Progressive", "Dichotomy", or any type added by
    #  core::ProblemSolver::addPathValidationType,
    #  \param tolerance maximal acceptable penetration.
    def selectPathValidation (self, pathValidationType, tolerance):
        return self.hppcorba.problem.selectPathValidation (pathValidationType,
                                                         tolerance)

    ## Add path validation object
    #  \param configValidationType name of the config validation method
    def addConfigValidation (self, configValidationType):
        return self.hppcorba.problem.addConfigValidation (configValidationType)

    ## Select configuration shooter type
    #  \param Name of the configuration shooter type
    def selectConfigurationShooter (self, type):
        return self.hppcorba.problem.selectConfigurationShooter (type)

    ## Select path projector method
    #  \param Name of the path projector method, either "Discretized"
    #  "Progressive", "Dichotomy", or any type added by
    #  core::ProblemSolver::addPathProjectorType,
    #  \param tolerance maximal acceptable penetration.
    def selectPathProjector (self, pathProjectorType, tolerance):
        return self.hppcorba.problem.selectPathProjector (pathProjectorType,
                                                        tolerance)


    ##  Select distance type
    #   \param Name of the distance type, either
    #      "WeighedDistance" or any type added by method
    #      core::ProblemSolver::addDistanceType
    def selectDistance (self, distanceType):
        return self.hppcorba.problem.selectDistance (distanceType)


    ##  Select steering method type
    #   \param Name of the steering method type, either
    #      "SteeringMethodStraight" or any type added by method
    #      core::ProblemSolver::addSteeringMethodType
    def selectSteeringMethod (self, steeringMethodType):
        return self.hppcorba.problem.selectSteeringMethod (steeringMethodType)


    def prepareSolveStepByStep (self):
        return self.hppcorba.problem.prepareSolveStepByStep ()

    def executeOneStep (self):
        return self.hppcorba.problem.executeOneStep ()

    def finishSolveStepByStep (self):
        return self.hppcorba.problem.finishSolveStepByStep ()

    ## Solve the problem of corresponding ChppPlanner object
    def solve (self):
        return self.hppcorba.problem.solve ()

    ## Make direct connection between two configurations
    #  \param startConfig, endConfig: the configurations to link.
    #  \param validate whether path should be validated. If true, path
    #         validation is called and only valid part of path is inserted
    #         in the path vector.
    #  \return True if the path is fully valid, False otherwise.
    #  \return the path index of the collision-free part from startConfig,
    #  \return a message explaining the reason why the path is not valid if
    #          relevant
    def directPath (self, startConfig, endConfig, validate):
        return self.hppcorba.problem.directPath (startConfig, endConfig, validate)

    ## Append a path to an existing path
    #  \param pathId Id of the path in this problem,
    #  \param config end configuration of the new path.
    #  \throw Error if steering method fails to create a direct path of if
    #  direct path is not valid
    #  Call steering method between end of path and input config and append
    #  direct path in case of success.
    def appendDirectPath (self, pathId, config, validate):
        return self.hppcorba.problem.appendDirectPath (pathId, config, validate)

    ## Concatenate path endId at the end of startId.
    #  \note No path are created. The resulting path is at rank startId.
    def concatenatePath (self, startId, endId):
        return self.hppcorba.problem.concatenatePath (startId, endId)

    ## extract path pathId from param start to end
    #  \note a New path is added to problem-solver
    def extractPath (self, pathId, start, end):
        return self.hppcorba.problem.extractPath (pathId, start, end)

    ## Erase path pathId from stored.
    def erasePath (self, pathId):
        return self.hppcorba.problem.erasePath (pathId)

    ## Project path using the path projector.
    # \return True in case of success, False otherwise.
    def projectPath (self, pathId):
        return self.hppcorba.problem.projectPath (pathId)

    ## Get Number of paths
    def numberPaths (self):
        return self.hppcorba.problem.numberPaths ()

    ## Optimize a given path
    # \param inPathId Id of the path in this problem.
    # \throw Error.
    def optimizePath(self, inPathId):
        return self.hppcorba.problem.optimizePath (inPathId)

    ## Get length of path
    # \param inPathId rank of the path in the problem
    # \return length of path if path exists.
    def pathLength(self, inPathId):
        return self.hppcorba.problem.pathLength(inPathId)

    ## Get the robot config at param on a path
    # \param inPathId rank of the path in the problem
    # \param param : the user parameter choice
    # \return dofseq : the config at param
    def configAtParam (self, inPathId, param):
        return self.hppcorba.problem.configAtParam (inPathId, param)

    ## Get the robot velocity at param on a path
    # \param inPathId rank of the path in the problem
    # \param orderId order of the derivative
    # \param param : the user parameter choice
    # \return dofseq : the velocity at param
    def derivativeAtParam (self, inPathId, order, param):
        return self.hppcorba.problem.derivativeAtParam (inPathId, order, param)

    ## Get way points of a path
    #  \param pathId rank of the path in the problem
    def getWaypoints (self, pathId):
        return self.hppcorba.problem.getWaypoints (pathId)

    ## \name Interruption of a path planning request
    #  \{

    ## \brief Interrupt path planning activity
    #   \note this method is effective only when multi-thread policy is used
    #         by CORBA server.
    #         See constructor of class Server for details.
    def interruptPathPlanning (self):
        return self.hppcorba.problem.interruptPathPlanning ()
    # \}

    ## \name exploring the roadmap
    #  \{

    ## Get nodes of the roadmap.
    def nodes(self):
      return self.hppcorba.problem.nodes ()

    # the configuration of the node nodeId
    def node(self,nodeId):
      return self.hppcorba.problem.node(nodeId)

    # the number of nodes in the roadmap
    def numberNodes(self):
      return self.hppcorba.problem.numberNodes ()

    ## Number of edges
    def numberEdges (self):
        return self.hppcorba.problem.numberEdges ()

    ## Edge at given rank
    def edge (self, edgeId):
        return self.hppcorba.problem.edge (edgeId)

    ## Number of connected components
    def numberConnectedComponents (self):
        return self.hppcorba.problem.numberConnectedComponents ()

    ## Nodes of a connected component
    #  \param connectedComponentId index of connected component in roadmap
    #  \return list of nodes of the connected component.
    def nodesConnectedComponent (self, ccId):
        return self.hppcorba.problem.nodesConnectedComponent (ccId)

    ## Return nearest neighbour of given input configuration.
    # \param connectedComponentId is the index of a connected component in the roadmap.
    #        If connectedComponentId is negative, function goes through all
    #        connected components looking for the nearest node (configuration).
    # \param distance returns the one-dimensional distance between \param config and
    #        computed nearest node (configuration). 
    # \sa numberConnectedComponents
    def getNearestConfig (self, randomConfig, connectedComponentId = -1):
        return self.hppcorba.problem.getNearestConfig (randomConfig, connectedComponentId)

    ## Add a configuration to the roadmap.
    # \param config to be added to the roadmap.
    def addConfigToRoadmap (self, config):
        return self.hppcorba.problem.addConfigToRoadmap(config)

    ## Add an edge to roadmap. If
    # \param config1, config2 the ends of the path,
    # \param pathId the index if the path in the vector of path,
    # \param bothEdges if FALSE, only add config1 to config2, otherwise, If TRUE. add edges config1->config2 AND config2->config1.
    def addEdgeToRoadmap (self, config1, config2, pathId, bothEdges):
        return self.hppcorba.problem.addEdgeToRoadmap (config1, config2, pathId, bothEdges)

    ## Clear the roadmap
    def clearRoadmap (self):
        return self.hppcorba.problem.clearRoadmap ()

    ## Save the current roadmap in a file
    #  \param filename name of the file where the roadmap is saved
    def saveRoadmap (self, filename):
        return self.hppcorba.problem.saveRoadmap (filename)

    ## Read a roadmap from a file
    #  \param filename name of the file from which the roadmap is read.
    def readRoadmap (self, filename):
        return self.hppcorba.problem.readRoadmap (filename)

    ## \}
