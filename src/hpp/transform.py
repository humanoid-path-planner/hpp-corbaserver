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

import numpy as np
from hpp import Quaternion

class Transform (object):
    def __init__ (self, *args):
        if len (args) == 0:
            self.translation = np.array ([0.,0.,0.])
            self.quaternion = Quaternion ()
        if len (args) == 7:
            self.translation = np.array (args [0:3])
            self.quaternion = Quaternion (args [3:7])
        if len (args) == 2:
            self.quaternion = Quaternion (args [0])
            self.translation = np.array (args [1])
        if len (args) == 1:
            if isinstance (args [0], Transform) :
                self.quaternion = args [0].quaternion
                self.translation = args [0].translation
            elif isinstance (args [0], list):
                self.translation = np.array (args [0][0:3])
                self.quaternion = Quaternion (args [0][3:7])
        self.quaternion.normalize ()

    def __mul__ (self, other):
        rot = self.quaternion * other.quaternion
        trans = self.quaternion.transform (other.translation) +\
            self.translation
        return Transform (rot, trans)

    def inverse (self):
        rot = self.quaternion.conjugate()
        trans = - self.quaternion.conjugate().transform (self.translation)
        return Transform (rot, trans)

    def transform (self, v):
        res = self.quaternion.transform (v) + self.translation
        return res
        
    def __str__ (self):
        return "quaternion:  %s,\ntranslation: %s"%(self.quaternion,
                                                    self.translation)

    def __getitem__ (self, i):
        if i<3: return self.translation [i]
        if i<7: return self.quaternion.toTuple () [i-3]
        raise IndexError ("index out of range")

    def __len__ (self):
        return 7

    def toHomogeneousMatrix (self):
        H = np.eye(4)
        H[:3,:3] = self.quaternion.toRotationMatrix()
        H[:3,3] = self.translation
        return H

    def toTuple (self):
        return tuple (self.translation) + self.quaternion.toTuple ()
