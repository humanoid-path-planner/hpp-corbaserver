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

from hpp.corbaserver.robot import Robot as Parent

class Robot (Parent):
    urdfName = "hrp2_14_capsule"
    urdfSuffix = ""
    srdfSuffix = ""
    packageName = "hrp2_14_description"
    halfSitting = \
        {"base_joint_x": 0.0,
         "base_joint_y": 0.0,
         "base_joint_z": 0.648702,
         "base_joint_SO3": (1.0, 0.0, 0.0, 0.0),
         "CHEST_JOINT0": 0.0,
         "CHEST_JOINT1": 0.0,
         "HEAD_JOINT0": 0.0,
         "HEAD_JOINT1": 0.0,
         "LARM_JOINT0": 0.261799,
         "LARM_JOINT1": 0.17453,
         "LARM_JOINT2": 0.0,
         "LARM_JOINT3": -0.523599,
         "LARM_JOINT4": 0.0,
         "LARM_JOINT5": 0.0,
         "LARM_JOINT6": 0.1,
         "LHAND_JOINT0": 0.0,
         "LHAND_JOINT1": 0.0,
         "LHAND_JOINT2": 0.0,
         "LHAND_JOINT3": 0.0,
         "LHAND_JOINT4": 0.0,
         "RARM_JOINT0": 0.261799,
         "RARM_JOINT1": -0.17453,
         "RARM_JOINT2": 0.0,
         "RARM_JOINT3": -0.523599,
         "RARM_JOINT4": 0.0,
         "RARM_JOINT5": 0.0,
         "RARM_JOINT6": 0.1,
         "RHAND_JOINT0": 0.0,
         "RHAND_JOINT1": 0.0,
         "RHAND_JOINT2": 0.0,
         "RHAND_JOINT3": 0.0,
         "RHAND_JOINT4": 0.0,
         "LLEG_JOINT0": 0.0,
         "LLEG_JOINT1": 0.0,
         "LLEG_JOINT2": -0.453786,
         "LLEG_JOINT3": 0.872665,
         "LLEG_JOINT4": -0.418879,
         "LLEG_JOINT5": 0.0,
         "RLEG_JOINT0": 0.0,
         "RLEG_JOINT1": 0.0,
         "RLEG_JOINT2": -0.453786,
         "RLEG_JOINT3": 0.872665,
         "RLEG_JOINT4": -0.418879,
         "RLEG_JOINT5": 0.0
         }

    def __init__ (self):
        Parent.__init__ (self,  self.urdfName, "freeflyer")

    def getInitialConfig (self):
        q = []
        for n in self.jointNames:
            dof = self.halfSitting [n]
            if type (dof) is tuple:
                q += dof
            else:
                q.append (dof)
        return q

    def leftHandClosed (self) :
        dofs = {"LARM_JOINT6": 0.1,
                "LHAND_JOINT0": 0.0,
                "LHAND_JOINT1": 0.0,
                "LHAND_JOINT2": 0.0,
                "LHAND_JOINT3": 0.0,
                "LHAND_JOINT4": 0.0}
        res = []
        for name, value in dofs.iteritems ():
            res.append ((self.rankInConfiguration [name], value))
        return res

    def rightHandClosed (self) :
        dofs = {"RARM_JOINT6": 0.1,
                "RHAND_JOINT0": 0.0,
                "RHAND_JOINT1": 0.0,
                "RHAND_JOINT2": 0.0,
                "RHAND_JOINT3": 0.0,
                "RHAND_JOINT4": 0.0}
        res = []
        for name, value in dofs.iteritems ():
            res.append ((self.rankInConfiguration [name], value))
        return res

        
Jcom = ((0.99999999969999998, 0.0, 0.0, 0.0, 0.15958929917090708, -0.0015496712689910483, 0.006457034810302846, 0.0, -0.04114883029389188, -0.017070288099399783, -0.001911897308848219, 4.9860119674112868e-21, -0.006457034810302846, 0.0, -0.04114883029389188, -0.017070288099399783, -0.001911897308848219, -4.9860119674112868e-21, -0.0015496395551697997, 0.023029662476935154, -2.9590114765902141e-05, 0.0018998018895406933, -0.024104545096910745, -0.0012462787861598856, 0.00016866935359163523, -0.0094440717997870197, 3.8088868042355686e-05, -0.0017903568629651118, -2.132722755310925e-05, -0.024090010464800509, 0.0012449800380576245, -0.00016641985705804981, -0.0094289163150136652, -3.5491372847415007e-05, -0.0017903568629651118, -2.132722755310925e-05), (0.0, 0.99999999969999998, 0.0, -0.15958929917090708, 0.0, 0.013695401426293118, 0.005578348463571141, 0.041190213959531319, 0.0, -0.0, 0.0, 0.0018841001489199836, 0.0055783484635711427, 0.041190213959531333, 0.0, -0.0, 0.0, 0.0018841001489199836, -0.0080306604130318347, 0.0, 0.00022111055708152463, 0.0, -0.0, 0.023759807377560616, 0.0047122898941987781, -0.00085741003728384058, 4.2166566902055324e-05, -0.00017392402013282932, 3.1910832047119565e-06, -0.0, 0.023731348485939281, 0.0047615821148691458, 0.00086610143705453523, 9.9084328022212327e-05, 0.00017392402013282932, -3.1910832047119565e-06), (0.0, 0.0, 0.99999999969999998, 0.0015496712689910483, -0.013695401426293118, 0.0, 0.0, -0.0065571272884102517, -0.0058165247579172871, 0.0060679086157445893, -7.5940850137388351e-05, -0.00013512977104984366, 0.0, 0.0065571272884102517, -0.0058165247579172888, 0.0060679086157445893, -7.5940850137388351e-05, 0.00013512977104984366, 0.0, 0.0078121562466710627, 0.0, -0.00022687908475124469, 0.0018498822331865989, -0.0046511829652424027, -0.00090539490661791823, -0.0025037065064851039, 3.8257634520993619e-06, -0.00054146100493501043, 8.064784951399954e-05, 0.001794169430077033, 0.0046463359638198786, 0.00091379015499538864, -0.002558798456728665, 5.868235625132099e-06, -0.00054146100493501043, 8.064784951399954e-05))
