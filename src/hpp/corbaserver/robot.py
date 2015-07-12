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
#  Helper class to load a robot model in hpp::core::ProblemSolver instance.
#
#  This class is also used to initialize a client to rviz in order to
#  display configurations of a robot.
class Robot (object):
    def __init__ (self, robotName, rootJointType, load = True):
        self.name = robotName
        self.displayName = robotName
        self.tf_root = "base_link"
        self.rootJointType = rootJointType
        self.client = Client ()
        if load:
            self.loadModel (robotName, rootJointType)
        self.jointNames = self.client.robot.getJointNames ()
        self.allJointNames = self.client.robot.getAllJointNames ()
        self.rankInConfiguration = dict ()
        self.rankInVelocity = dict ()
        rankInConfiguration = rankInVelocity = 0
        for j in self.jointNames:
            self.rankInConfiguration [j] = rankInConfiguration
            rankInConfiguration += self.client.robot.getJointConfigSize (j)
            self.rankInVelocity [j] = rankInVelocity
            rankInVelocity += self.client.robot.getJointNumberDof (j)

    def loadModel (self, robotName, rootJointType):
        self.client.robot.loadRobotModel (robotName, rootJointType,
                                          self.packageName, self.urdfName,
                                          self.urdfSuffix, self.srdfSuffix)

    ## \name Degrees of freedom
    #  \{

    ## Get size of configuration
    # \return size of configuration
    def getConfigSize (self):
        return self.client.robot.getConfigSize ()

    # Get size of velocity
    # \return size of velocity
    def getNumberDof (self):
        return self.client.robot.getNumberDof ()
    ## \}

    ## \name Joints
    #\{

    ## Get joint names in the same order as in the configuration.
    def getJointNames (self):
        return self.client.robot.getJointNames ()

    ## Get joint names in the same order as in the configuration.
    def getAllJointNames (self):
        return self.client.robot.getAllJointNames ()

    ## Get joint position.
    def getJointPosition (self, jointName):
        return self.client.robot.getJointPosition (jointName)

    ## Get constant position of root joint in world frame in initial position
    def getRootJointPosition (self):
        return self.client.robot.getRootJointPosition ()

    ## Set position of root joint in world frame in initial configuration
    def setRootJointPosition (self, position):
        return self.client.robot.setRootJointPosition (position)

    ## Set the static position of joint WRT its parent
    def setJointPosition (self, jointName, position):
        return self.client.robot.setJointPosition (jointName, position)

    ## Get joint number degrees of freedom.
    def getJointNumberDof (self, jointName):
        return self.client.robot.getJointNumberDof (jointName)

    ## Get joint number config size.
    def getJointConfigSize (self, jointName):
        return self.client.robot.getJointConfigSize (jointName)

    ## set bounds for the joint
    def setJointBounds (self, jointName, inJointBound):
        return self.client.robot.setJointBounds (jointName, inJointBound)

    ## Get link position in world frame
    #
    # Joints are oriented in a different way as in urdf standard since
    # rotation and uni-dimensional translation joints act around or along
    # their x-axis. This method returns the position of the urdf link in
    # world frame.
    #
    # \param jointName name of the joint
    # \return position of the link in world frame.
    def getLinkPosition (self, jointName):
        return self.client.robot.getLinkPosition (jointName)

    ## Get link name
    #
    # \param jointName name of the joint,
    # \return name of the link.
    def getLinkName (self, jointName):
        return self.client.robot.getLinkName (jointName)

    ## \}

    ## \name Configurations
    #\{

    ## Set current configuration of composite robot
    #
    #  \param q configuration of the composite robot
    def setCurrentConfig (self, q):
        self.client.robot.setCurrentConfig (q)

    ## Get current configuration of composite robot
    #
    #  \return configuration of the composite robot
    def getCurrentConfig (self):
        return self.client.robot.getCurrentConfig ()

    ## Shoot random configuration
    #  \return dofArray Array of degrees of freedom.
    def shootRandomConfig(self):
        return self.client.robot.shootRandomConfig ()

    ## \}

    ## \name Bodies
    #  \{

    ##  Get the list of objects attached to a joint.
    #  \param inJointName name of the joint.
    #  \return list of names of CollisionObject attached to the body.
    def getJointInnerObjects (self, jointName):
        return self.client.robot.getJointInnerObjects (jointName)


    ##  Get list of collision objects tested with the body attached to a joint
    #  \param inJointName name of the joint.
    #  \return list of names of CollisionObject
    def getJointOuterObjects (self, jointName):
        return self.client.robot.getJointOuterObjects (jointName)

    ## Get position of robot object
    #  \param objectName name of the object.
    #  \return transformation as a hpp.Transform object
    def getObjectPosition (self, objectName):
        return Transform (self.client.robot.getObjectPosition (objectName))

    ## \brief Remove an obstacle from outer objects of a joint body
    #
    #  \param objectName name of the object to remove,
    #  \param jointName name of the joint owning the body,
    #  \param collision whether collision with object should be computed,
    #  \param distance whether distance to object should be computed.
    def removeObstacleFromJoint (self, objectName, jointName, collision,
                                 distance):
        return self.client.obstacle.removeObstacleFromJoint \
            (objectName, jointName, collision, distance)


    ## \}

    ## \name Collision checking and distance computation
    # \{

    ## Test collision with obstacles and auto-collision.
    #
    # Check whether current configuration of robot is valid by calling
    # CkwsDevice::collisionTest ().
    # \return whether configuration is valid
    # \note Deprecated. Use isConfigValid instead.
    def collisionTest (self):
        print "Deprecated. Use isConfigValid instead"
        return self.client.robot.collisionTest ()

    ## Check the validity of a configuration.
    #
    # Check whether a configuration of robot is valid.
    # \param cfg a configuration
    # \return whether configuration is valid
    # \throw if config is not valid, raise an exception.
    def isConfigValid (self, cfg):
        return self.client.robot.isConfigValid (cfg)

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
        return self.client.robot.distancesToCollision ()
    ## \}
    ## \name Mass and inertia
    # \{

    ## Get mass of robot
    def getMass (self):
        return self.client.robot.getMass ()

    ## Get position of center of mass
    def getCenterOfMass (self):
        return self.client.robot.getCenterOfMass ()
    ## Get Jacobian of the center of mass
    def getJacobianCenterOfMass (self):
        return self.client.robot.getJacobianCenterOfMass ()
    ##\}

## Humanoid robot
#
#  Method loadModel builds a humanoid robot.
class HumanoidRobot (Robot):

    def __init__ (self, robotName, rootJointType, load = True):
        Robot.__init__ (self, robotName, rootJointType, load)
    def loadModel (self, robotName, rootJointType):
        self.client.robot.loadHumanoidModel (robotName, rootJointType,
                                          self.packageName, self.urdfName,
                                          self.urdfSuffix, self.srdfSuffix)
