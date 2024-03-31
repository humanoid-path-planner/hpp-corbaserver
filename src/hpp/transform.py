# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

import numpy as np

from hpp import Quaternion


class Transform:
    def __init__(self, *args):
        if len(args) == 0:
            self.translation = np.array([0.0, 0.0, 0.0])
            self.quaternion = Quaternion()
        if len(args) == 7:
            self.translation = np.array(args[0:3])
            self.quaternion = Quaternion(args[3:7])
        if len(args) == 2:
            self.quaternion = Quaternion(args[0])
            self.translation = np.array(args[1])
        if len(args) == 1:
            if isinstance(args[0], Transform):
                self.quaternion = args[0].quaternion
                self.translation = args[0].translation
            elif isinstance(args[0], (list, tuple)):
                self.translation = np.array(args[0][0:3])
                self.quaternion = Quaternion(args[0][3:7])
        self.quaternion.normalize()

    def __mul__(self, other):
        rot = self.quaternion * other.quaternion
        trans = self.quaternion.transform(other.translation) + self.translation
        return Transform(rot, trans)

    def inverse(self):
        rot = self.quaternion.conjugate()
        trans = -self.quaternion.conjugate().transform(self.translation)
        return Transform(rot, trans)

    def transform(self, v):
        res = self.quaternion.transform(v) + self.translation
        return res

    def __str__(self):
        return f"quaternion:  {self.quaternion},\ntranslation: {self.translation}"

    def __getitem__(self, i):
        if i < 3:
            return self.translation[i]
        if i < 7:
            return self.quaternion.toTuple()[i - 3]
        raise IndexError("index out of range")

    def __len__(self):
        return 7

    def toHomogeneousMatrix(self):
        H = np.eye(4)
        H[:3, :3] = self.quaternion.toRotationMatrix()
        H[:3, 3] = self.translation
        return H

    def toTuple(self):
        return tuple(self.translation) + self.quaternion.toTuple()
