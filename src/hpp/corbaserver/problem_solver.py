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

## Definition of a path planning problem
#
#  This class wraps the Corba client to the server implemented by
#  libhpp-corbaserver.so
#
#  Some method implemented by the server can be considered as private. The
#  goal of this class is to hide them and to expose those that can be
#  considered as public.
class ProblemSolver (object):
    def __init__ (self, robot):
        self.client = robot.client
        self.robot = robot

    ## \name Initial and goal configurations
    # \{

    ## Set initial configuration of specified problem.
    #	\param dofArray Array of degrees of freedom
    #	\throw Error.
    def setInitialConfig (self, dofArray):
        return self.client.problem.setInitialConfig (dofArray)

    ## Get initial configuration of specified problem.
    #	\return Array of degrees of freedom
    def getInitialConfig (self):
        return self.client.problem.getInitialConfig ()

    ## Add goal configuration to specified problem.
    #	\param dofArray Array of degrees of freedom
    #	\throw Error.
    def addGoalConfig (self, dofArray):
        return self.client.problem.addGoalConfig (dofArray)

    ## Get goal configurations of specified problem.
    #	\return Array of degrees of freedom
    def getGoalConfigs (self):
        return self.client.problem.getGoalConfigs ()

    ## Reset goal configurations
    def resetGoalConfigs (self):
        return self.client.problem.resetGoalConfigs ()
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
        return self.client.obstacle.loadObstacleModel (package, filename,
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
        return self.client.obstacle.removeObstacleFromJoint \
            (objectName, jointName, collision, distance)

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
        return self.client.obstacle.moveObstacle (objectName, cfg)
    ## Get the position of an obstacle
    #
    #  \param objectName name of the polyhedron.
    #  \retval cfg Position of the obstacle.
    #  \throw Error.
    def getObstaclePosition (self, objectName):
        return self.client.obstacle.getObstaclePosition (objectName)

    ## Get list of obstacles
    #
    #  \param collision whether to return obstacle for collision,
    #  \param distance whether to return obstacles for distance computation
    # \return list of obstacles
    def getObstacleNames (self, collision, distance):
        return self.client.obstacle.getObstacleNames (collision, distance)

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
    #  If joint1 of joint2 is "", the corresponding joint is replaced by
    #  the global frame.
    #  constraints are stored in ProblemSolver object
    def createOrientationConstraint (self, constraintName, joint1Name,
                                     joint2Name, p, mask):
        return self.client.problem.createOrientationConstraint \
            (constraintName, joint1Name, joint2Name, p, mask)

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
        return self.client.problem.createPositionConstraint \
            (constraintName, joint1Name, joint2Name, point1, point2, mask)

    ## Reset Constraints
    #
    #  Reset all constraints, including numerical constraints and locked
    #  joints
    def resetConstraints (self):
        return self.client.problem.resetConstraints ()

    ## Set numerical constraints in ConfigProjector
    #
    #  \param name name of the resulting numerical constraint obtained
    #         by stacking elementary numerical constraints,
    #  \param names list of names of the numerical constraints as
    #         inserted by method hpp::core::ProblemSolver::addNumericalConstraint.
    def setNumericalConstraints (self, name, names):
        return self.client.problem.setNumericalConstraints (name, names)

    ## Apply constraints
    #
    #  \param q initial configuration
    #  \return configuration projected in success,
    #  \throw Error if projection failed.
    def applyConstraints (self, q):
        return self.client.problem.applyConstraints (q)

    ## Create a vector of passive dofs.
    #
    #  \param name name of the vector in the ProblemSolver map.
    #  \param dofNames list of names of DOF that may
    def addPassiveDofs (self, name, dofNames):
        return self.client.problem.addPassiveDofs (name, dofNames)

    ## Generate a configuration satisfying the constraints
    #
    #  \param maxIter maximum number of tries,
    #  \return configuration projected in success,
    #  \throw Error if projection failed.
    def generateValidConfig (self, maxIter):
        return self.client.problem.generateValidConfig (maxIter)

    ## Lock joint with given joint configuration
    # \param jointName name of the joint
    # \param value value of the joint configuration
    def lockJoint (self, jointName, value):
        return self.client.problem.lockJoint (jointName, value)

    ## error threshold in numerical constraint resolution
    def setErrorThreshold (self, threshold):
        return self.client.problem.setErrorThreshold (threshold)

    ## Set the maximal number of iterations
    def setMaxIterations (self, iterations):
	return self.client.problem.setMaxIterations (iterations)


    ## \}

    ## \name Solve problem and get paths
    # \{

    ## Select path planner type
    #  \param Name of the path planner type, either "DiffusingPlanner",
    #         "VisibilityPrmPlanner", or any type added by method
    #         core::ProblemSolver::addPathPlannerType
    def selectPathPlanner (self, pathPlannerType):
        return self.client.problem.selectPathPlanner (pathPlannerType)

    ## Select path optimizer type
    #  \param Name of the path optimizer type, either "RandomShortcut" or
    #   any type added by core::ProblemSolver::addPathOptimizerType
    def selectPathOptimizer (self, pathOptimizerType):
        return self.client.problem.selectPathOptimizer (pathOptimizerType)

    ## Select path validation method
    #  \param Name of the path validation method, either "Discretized"
    #  "Progressive", "Dichotomy", or any type added by
    #  core::ProblemSolver::addPathValidationType,
    #  \param tolerance maximal acceptable penetration.
    def selectPathValidation (self, pathValidationType, tolerance):
        return self.client.problem.selectPathValidation (pathValidationType,
                                                         tolerance)

    ## Select path projector method
    #  \param Name of the path projector method, either "Discretized"
    #  "Progressive", "Dichotomy", or any type added by
    #  core::ProblemSolver::addPathProjectorType,
    #  \param tolerance maximal acceptable penetration.
    def selectPathProjector (self, pathProjectorType, tolerance):
        return self.client.problem.selectPathProjector (pathProjectorType,
                                                        tolerance)

    ## Solve the problem of corresponding ChppPlanner object
    def solve (self):
        return self.client.problem.solve ()

    ## Make direct connection between two configurations
    #  \param startConfig, endConfig: the configurations to link.
    #  \throw Error if steering method fails to create a direct path of if
    #  direct path is not valid
    def directPath (self, startConfig, endConfig):
        return self.client.problem.directPath (startConfig, endConfig)

    ## Get Number of paths
    def numberPaths (self):
        return self.client.problem.numberPaths ()

    ## Optimize a given path
    # \param inPathId Id of the path in this problem.
    # \throw Error.
    def optimizePath(self, inPathId):
        return self.client.problem.optimizePath (inPathId)

    ## Get length of path
    # \param inPathId rank of the path in the problem
    # \return length of path if path exists.
    def pathLength(self, inPathId):
        return self.client.problem.pathLength(inPathId)

    ## Get the robot's config at param on the a path
    # \param inPathId rank of the path in the problem
    # \param atDistance : the user parameter choice
    # \return dofseq : the config at param
    def configAtParam (self, inPathId, atDistance):
        return self.client.problem.configAtParam (inPathId, atDistance)

    ## Get way points of a path
    #  \param pathId rank of the path in the problem
    def getWaypoints (self, pathId):
        return self.client.problem.getWaypoints (pathId)

    ## \name Interruption of a path planning request
    #  \{

    ## \brief Interrupt path planning activity
    #   \note this method is effective only when multi-thread policy is used
    #         by CORBA server.
    #         See constructor of class Server for details.
    def interruptPathPlanning (self):
        return self.client.problem.interruptPathPlanning ()
    # \}

    ## \name exploring the roadmap
    #  \{

    ## Get nodes of the roadmap.
    def nodes(self):
	return self.client.problem.nodes ()

    ## Number of edges
    def numberEdges (self):
        return self.client.problem.numberEdges ()

    ## Edge at given rank
    def edge (self, edgeId):
        return self.client.problem.edge (edgeId)

    ## Number of connected components
    def numberConnectedComponents (self):
        return self.client.problem.numberConnectedComponents ()

    ## Nodes of a connected component
    #  \param connectedComponentId index of connected component in roadmap
    #  \return list of nodes of the connected component.
    def nodesConnectedComponent (self, ccId):
        return self.client.problem.nodesConnectedComponent (ccId)

    ## Clear the roadmap
    def clearRoadmap (self):
        return self.client.problem.clearRoadmap ()
    ## \}
