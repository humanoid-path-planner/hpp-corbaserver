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

    ## \brief Add goal configuration to specified problem.
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

    ## \name Constraints
    #  \{

    ## Reset Constraints
    #
    #  Reset all constraints, including numerical constraints and locked
    #  degrees of freedom.
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

    ## Lock degree of freedom with given value
    # \param jointName name of the joint
    # \param value value of the locked degree of freedom,
    # \param rankInConfiguration rank of the locked dof in the joint
    #        configuration vector
    # \param rankInVelocity rank of the locked dof in the joint
    #        velocity vector
    def lockDof (self, jointName, value, rankInConfiguration, rankInVelocity):
        return self.client.problem.lockDof (jointName, value,
                                            rankInConfiguration, rankInVelocity)
    ## Lock joint with one degree of freedom with given value 
    # \param jointName name of the joint
    # \param value value of the locked degree of freedom,
    def lockOneDofJoint (self, jointName, value):
        return self.client.problem.lockDof (jointName, value, 0, 0)

    ## \}
    ## \name Solve problem and get paths
    # \{

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
    def configAtDistance(self, inPathId, atDistance):
        return self.client.problem.configAtDistance(inPathId, atDistance)
    ## \}
