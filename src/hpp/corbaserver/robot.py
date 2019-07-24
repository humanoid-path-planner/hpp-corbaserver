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

from hpp import Transform
from hpp.corbaserver.client import Client

##
#  Helper class to load a robot model in hpp::core::ProblemSolver.
#
#  This class is also a wrapper of idl methods defined by
#  hpp::corbaserver::Robot. Most methods call idl methods.
class Robot (object):
    def __init__ (self, robotName = None, rootJointType = None, load = True, client = None, hppcorbaClient = None):
        if client is None: client = Client ()
        self.client = client
        self.hppcorba = client if hppcorbaClient is None else hppcorbaClient
        if robotName is None:
            # assert (rootJointType is None), "rootJointType is ignore when robotName is None"
            self.name = self.hppcorba.robot.getRobotName()
            self.rootJointType = rootJointType
            self.displayName = self.name
            self.rebuildRanks()
        else:
            self.name = robotName
            self.displayName = robotName
            self.rootJointType = rootJointType
            if load:
                self.loadModel (robotName, rootJointType)
            else:
                self.rebuildRanks()

    ## Rebuild inner variables rankInConfiguration and rankInVelocity
    def rebuildRanks (self):
        try:
            self.jointNames = self.hppcorba.robot.getJointNames ()
        except:
            # No robot yet
            return
        self.allJointNames = self.hppcorba.robot.getAllJointNames ()
        self.rankInConfiguration = dict ()
        self.rankInVelocity = dict ()
        rankInConfiguration = rankInVelocity = 0
        for j in self.jointNames:
            self.rankInConfiguration [j] = rankInConfiguration
            rankInConfiguration += self.hppcorba.robot.getJointConfigSize (j)
            self.rankInVelocity [j] = rankInVelocity
            rankInVelocity += self.hppcorba.robot.getJointNumberDof (j)

    def loadModel (self, robotName, rootJointType):
        self.hppcorba.robot.loadRobotModel (robotName, rootJointType,
                                          self.packageName, self.urdfName,
                                          self.urdfSuffix, self.srdfSuffix)
        self.rebuildRanks()

    def urdfPath (self):
        return "package://" + self.packageName + '/urdf/' + self.urdfName + self.urdfSuffix + '.urdf'

    ## \name Degrees of freedom
    #  \{

    ## Get size of configuration
    # \return size of configuration
    def getConfigSize (self):
        return self.hppcorba.robot.getConfigSize ()

    # Get size of velocity
    # \return size of velocity
    def getNumberDof (self):
        return self.hppcorba.robot.getNumberDof ()
    ## \}

    ## \name Joints
    #\{

    ## Get joint names in the same order as in the configuration.
    def getJointNames (self):
        return self.hppcorba.robot.getJointNames ()

    ## Get joint types in the same order as in the configuration.
    def getJointTypes (self):
        return self.hppcorba.robot.getJointTypes ()

    ## Get joint names in the same order as in the configuration.
    def getAllJointNames (self):
        return self.hppcorba.robot.getAllJointNames ()

    ## Get joint position.
    def getJointPosition (self, jointName):
        return self.hppcorba.robot.getJointPosition (jointName)

    ## Get constant position of root joint in world frame in initial position
    def getRootJointPosition (self):
        return self.hppcorba.robot.getRootJointPosition ()

    ## Set position of root joint in world frame in initial configuration
    def setRootJointPosition (self, position):
        return self.hppcorba.robot.setRootJointPosition (position)

    ## Set the static position of joint WRT its parent
    def setJointPosition (self, jointName, position):
        return self.hppcorba.robot.setJointPositionInParentFrame (jointName, position)

    ## Get joint transformation in world frame for current configuration.
    def getCurrentTransformation(self, jointName):
        return self.hppcorba.robot.getCurrentTransformation (jointName)

    ## Get joint number degrees of freedom.
    def getJointNumberDof (self, jointName):
        return self.hppcorba.robot.getJointNumberDof (jointName)

    ## Get joint number config size.
    def getJointConfigSize (self, jointName):
        return self.hppcorba.robot.getJointConfigSize (jointName)

    ## set bounds for the joint
    def setJointBounds (self, jointName, inJointBound):
        return self.hppcorba.robot.setJointBounds (jointName, inJointBound)

    ## Get bounds for a joint
    #
    #  \param jointName name of the joint
    #  \return sequence of bounds in order [v0_min,v0_max,v1_min,v1_max,...]
    #          where vi_min, vi_max are the bounds of the i-th degree of
    #          freedom of the joint if the degree of freedom is bounded, 1, 0
    #          otherwise.
    def getJointBounds(self, jointName):
        return self.hppcorba.robot.getJointBounds(jointName)

    ## Get joints that are saturated for a given configuration
    #
    #  \param q configuration
    #  \return list of triples joint names, dof id, value
    def getSaturated (self, q):
        saturated = []
        for j in self.jointNames:
            b = self.getJointBounds (j)
            r = self.rankInConfiguration [j]
            for m, M, i in zip (b [::2], b [1::2], range (100000)):
                if q [r+i] == m or q [r+i] == M:
                    saturated.append ((j, i, q [r+i]))
        return saturated

    ## Get link position in world frame
    #
    # Joints are oriented in a different way as in urdf standard since
    # rotation and uni-dimensional translation joints act around or along
    # their x-axis. This method returns the position of the urdf link in
    # world frame.
    #
    # \param jointName name of the joint
    # \return position of the link in world frame.
    def getLinkPosition (self, linkName):
        return self.hppcorba.robot.getLinkPosition (linkName)

    ## Get link name
    #
    # \param jointName name of the joint,
    # \return name of the link.
    def getLinkNames (self, jointName):
        return self.hppcorba.robot.getLinkNames (jointName)

    ## \}

    ## \name Configurations
    #\{

    ## Set current configuration of composite robot
    #
    #  \param q configuration of the composite robot
    def setCurrentConfig (self, q):
        self.hppcorba.robot.setCurrentConfig (q)

    ## Get current configuration of composite robot
    #
    #  \return configuration of the composite robot
    def getCurrentConfig (self):
        return self.hppcorba.robot.getCurrentConfig ()

    ## Set current velocity of composite robot
    #
    #  \param q velocity of the composite robot
    def setCurrentVelocity (self, v):
        self.hppcorba.robot.setCurrentVelocity (v)

    ## Get current velocity of composite robot
    #
    #  \return velocity of the composite robot
    def getCurrentVelocity (self):
        return self.hppcorba.robot.getCurrentVelocity ()

    ## Shoot random configuration
    #  \return dofArray Array of degrees of freedom.
    def shootRandomConfig(self):
        return self.hppcorba.robot.shootRandomConfig ()

    ## \}

    ## \name Bodies
    #  \{

    ##  Get the list of objects attached to a joint.
    #  \param inJointName name of the joint.
    #  \return list of names of CollisionObject attached to the body.
    def getJointInnerObjects (self, jointName):
        return self.hppcorba.robot.getJointInnerObjects (jointName)


    ##  Get list of collision objects tested with the body attached to a joint
    #  \param inJointName name of the joint.
    #  \return list of names of CollisionObject
    def getJointOuterObjects (self, jointName):
        return self.hppcorba.robot.getJointOuterObjects (jointName)

    ## Get position of robot object
    #  \param objectName name of the object.
    #  \return transformation as a hpp.Transform object
    def getObjectPosition (self, objectName):
        return Transform (self.hppcorba.robot.getObjectPosition (objectName))

    ## \brief Remove an obstacle from outer objects of a joint body
    #
    #  \param objectName name of the object to remove,
    #  \param jointName name of the joint owning the body,
    def removeObstacleFromJoint (self, objectName, jointName):
        return self.hppcorba.obstacle.removeObstacleFromJoint \
            (objectName, jointName, True, False)


    ## \}

    ## \name Collision checking and distance computation
    # \{

    ## Check the validity of a configuration.
    #
    # Check whether a configuration of robot is valid.
    # \param cfg a configuration
    # \return whether configuration is valid
    # \throw if config is not valid, raise an exception.
    def isConfigValid (self, cfg):
        return self.hppcorba.robot.isConfigValid (cfg)

    ## Compute distances between bodies and obstacles
    #
    # \return list of distances,
    # \return names of the objects belonging to a body
    # \return names of the objects tested with inner objects,
    # \return  closest points on the body,
    # \return  closest points on the obstacles
    # \note outer objects for a body can also be inner objects of another
    # body.
    def distancesToCollision (self):
        return self.hppcorba.robot.distancesToCollision ()

    ## Get the aligned axes bounding box around the robot.
    # \return a vector a 6 floats. The 3 first are one corner of the box (lowest in all dimensions),
    #         the 3 last are the opposite corner.
    def getRobotAABB(self):
        return self.hppcorba.robot.getRobotAABB ()
    ## \}
    ## \name Mass and inertia
    # \{

    ## Get mass of robot
    def getMass (self):
        return self.hppcorba.robot.getMass ()

    ## Get position of center of mass
    def getCenterOfMass (self):
        return self.hppcorba.robot.getCenterOfMass ()
    ## Get Jacobian of the center of mass
    def getJacobianCenterOfMass (self):
        return self.hppcorba.robot.getJacobianCenterOfMass ()
    ##\}

## This class provides tools to create static stability constraints
class StaticStabilityConstraintsFactory:
    def _getCOM (self, com):
        from numpy import array
        if com == "":
            return array(self.getCenterOfMass ())
        else:
            return array(self.hppcorba.robot.getPartialCom (com))

    ## Create static stability constraints where the robot slides on the ground,
    ## and store them into ProblemSolver
    ## \param prefix prefix of the names of the constraint as stored in
    ##        core::ProblemSolver,
    ## \param comName name of the PartialCOM in the problem solver map. Put "" for
    ##        a full COM computations.
    ## \param leftAnkle, rightAnkle: names of the ankle joints.
    ## \param q0 input configuration for computing constraint reference,
    ## \return a list of the names of the created constraints
    ##
    ## The constraints are stored in the core::ProblemSolver constraints map
    ## and are accessible through the method
    ## hpp::core::ProblemSolver::addNumericalConstraint:
    def createSlidingStabilityConstraint (self, prefix, comName, leftAnkle, rightAnkle, q0):
        robot = self.hppcorba.robot
        problem = self.hppcorba.problem

        _tfs = robot.getJointsPosition (q0, (leftAnkle, rightAnkle))
        Ml = Transform(_tfs[0])
        Mr = Transform(_tfs[1])
        self.setCurrentConfig (q0)
        x = self._getCOM (comName)
        result = []

        # COM wrt left ankle frame
        xloc = Ml.inverse().transform(x)
        result.append (prefix + "relative-com")
        problem.createRelativeComConstraint (result[-1], comName, leftAnkle, xloc.tolist(), (True,)*3)

        # Relative pose of the feet
        result.append (prefix + "relative-pose")
        problem.createTransformationConstraint2 (result[-1],
            leftAnkle, rightAnkle, (0,0,0,0,0,0,1), (Mr.inverse()*Ml).toTuple(), (True,)*6)

        # Pose of the left foot
        result.append (prefix + "pose-left-foot")
        problem.createTransformationConstraint2 (result[-1],
            "", leftAnkle, Ml.toTuple(), (0,0,0,0,0,0,1), (False,False,True,True,True,False))

        # Complement left foot
        result.append (prefix + "pose-left-foot-complement")
        problem.createTransformationConstraint2 (result[-1],
            "", leftAnkle, Ml.toTuple(), (0,0,0,0,0,0,1), (True,True,False,False,False,True))
        problem.setConstantRightHandSide (result[-1], False)

        return result

    ## Create static stability constraints where the feet are fixed on the ground,
    ## and store them into ProblemSolver
    ## \param prefix prefix of the names of the constraint as stored in
    ##        core::ProblemSolver,
    ## \param comName name of the PartialCOM in the problem solver map. Put "" for
    ##        a full COM computations.
    ## \param leftAnkle, rightAnkle: names of the ankle joints.
    ## \param q0 input configuration for computing constraint reference,
    ## \return a list of the names of the created constraints
    ##
    ## The constraints are stored in the core::ProblemSolver constraints map
    ## and are accessible through the method
    ## hpp::core::ProblemSolver::addNumericalConstraint:
    def createStaticStabilityConstraint (self, prefix, comName, leftAnkle, rightAnkle, q0):
        robot = self.hppcorba.robot
        problem = self.hppcorba.problem

        _tfs = robot.getJointsPosition (q0, (leftAnkle, rightAnkle))
        Ml = Transform(_tfs[0])
        Mr = Transform(_tfs[1])
        self.setCurrentConfig (q0)
        x = self._getCOM (comName)
        result = []

        # COM wrt left ankle frame
        xloc = Ml.inverse().transform(x)
        result.append (prefix + "relative-com")
        problem.createRelativeComConstraint (result[-1], comName, leftAnkle, xloc.tolist(), (True,)*3)

        # Pose of the left foot
        result.append (prefix + "pose-left-foot")
        problem.createTransformationConstraint2 (result[-1],
            "", leftAnkle, Ml.toTuple(), (0,0,0,0,0,0,1), (True,True,True,True,True,True))

        # Pose of the right foot
        result.append (prefix + "pose-right-foot")
        problem.createTransformationConstraint2 (result[-1],
            "", rightAnkle, Mr.toTuple(), (0,0,0,0,0,0,1), (True,True,True,True,True,True))

        return result

    ## Create static stability constraints where the COM is vertically projected
    ## on the line between the two ankles,  and the feet slide (or are fixed) on the ground.
    ## The constraints are stored into ProblemSolver
    ## \param prefix prefix of the names of the constraint as stored in
    ##        core::ProblemSolver,
    ## \param comName name of the PartialCOM in the problem solver map. Put "" for
    ##        a full COM computations.
    ## \param leftAnkle, rightAnkle: names of the ankle joints.
    ## \param q0 input configuration for computing constraint reference,
    ## \param sliding whether the feet slide or are fixed.
    ## \return a list of the names of the created constraints
    ##
    ## The constraints are stored in the core::ProblemSolver constraints map
    ## and are accessible through the method
    ## hpp::core::ProblemSolver::addNumericalConstraint:
    def createAlignedCOMStabilityConstraint (self, prefix, comName, leftAnkle, rightAnkle, q0, sliding):
        robot = self.hppcorba.robot
        problem = self.hppcorba.problem

        _tfs = robot.getJointsPosition (q0, (leftAnkle, rightAnkle))
        Ml = Transform(_tfs[0])
        Mr = Transform(_tfs[1])
        robot.setCurrentConfig (q0)
        x = self._getCOM (robot, comName)
        result = []

        # COM between feet
        result.append (prefix + "com-between-feet")
        problem.createComBetweenFeet (result[-1], comName, leftAnkle, rightAnkle,
            (0,0,0), (0,0,0), "", x.tolist(), (True,)*4)

        if sliding:
          mask = ( False, False, True, True, True, False )
        else:
          mask = ( True, ) * 6

        # Pose of the right foot
        result.append (prefix + "pose-right-foot")
        problem.createTransformationConstraint2 (result[-1],
            "", rightAnkle, Mr.toTuple(), (0,0,0,0,0,0,1), mask)

        # Pose of the left foot
        result.append (prefix + "pose-left-foot")
        problem.createTransformationConstraint2 (result[-1],
            "", leftAnkle, Ml.toTuple(), (0,0,0,0,0,0,1), mask)

        return result;

## Humanoid robot
#
#  Method loadModel builds a humanoid robot.
class HumanoidRobot (Robot, StaticStabilityConstraintsFactory):
    def __init__ (self, robotName = None, rootJointType = None, load = True, client = None, hppcorbaClient = None):
        Robot.__init__ (self, robotName, rootJointType, load, client)

    def loadModel (self, robotName, rootJointType):
        self.hppcorba.robot.loadHumanoidModel (robotName, rootJointType,
                                          self.packageName, self.urdfName,
                                          self.urdfSuffix, self.srdfSuffix)
        self.rebuildRanks()

class RobotXML (Robot):
    def __init__ (self, robotName, rootJointType, urdfString, srdfString = "",
            load = True, client = None, hppcorbaClient = None,):
        self.load = load
        self.urdfString = urdfString
        self.srdfString = srdfString
        Robot.__init__ (self, robotName, rootJointType, load, client)
    def loadModel (self, robotName, rootJointType):
        if self.load:
            self.hppcorba.robot.loadRobotModelFromString (
                    robotName, rootJointType, self.urdfString, self.srdfString)
        self.rebuildRanks()
    def urdfPath (self):
        return self.urdfString
