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

from hpp.corbaserver import Client

class Robot (object):
    """Helper class to enhance corba interface"""
    def __init__ (self, robotName, urdfSuffix = "", srdfSuffix = "",
                  rcpdfSuffix=""):
        self.client = Client ()
        self.client.robot.loadRobotModel (robotName, urdfSuffix, srdfSuffix,
                                          rcpdfSuffix)
        self.jointNames = self.client.robot.getJointNames ()
        self.rankInConfiguration = dict ()
        self.rankInVelocity = dict ()
        rankInConfiguration = rankInVelocity = 0
        for j in self.jointNames:
            self.rankInConfiguration [j] = rankInConfiguration
            rankInConfiguration += self.client.robot.getJointConfigSize (j)
            self.rankInVelocity [j] = rankInVelocity
            rankInVelocity += self.client.robot.getJointNumberDof (j)

    def setTranslationBounds (self, xmin, xmax, ymin, ymax, zmin, zmax):
        self.client.robot.setJointBounds (0, [xmin, xmax])
        self.client.robot.setJointBounds (1, [ymin, ymax])
        self.client.robot.setJointBounds (2, [zmin, zmax])
