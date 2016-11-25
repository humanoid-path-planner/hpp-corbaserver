# Copyright (c) 2016 CNRS
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
import csv
from hpp import Quaternion

# Test construction from SO3 matrix
for _ in xrange (1000):
    q1 = Quaternion (np.random.sample (4)).normalize ()
    mat = q1.toRotationMatrix ()
    q2 = Quaternion (mat)
    if abs (q1-q2) > 1e-10 and abs (q1+q2) > 1e-10:
        raise RuntimeError ("Error in conversion quaternion matrix")

# test addition
with open ('./test/add-quaternions.csv', 'r') as f:
    r = csv.reader (f, delimiter=',')
    iline=0
    for line in r:
        iline+=1
        data = map (float, line)
        q1 = Quaternion (data [0:4])
        q2 = Quaternion (data [4:8])
        q3 = Quaternion (data [8:12])
        if abs(q1+q2-q3) > 1e-10:
            raise RuntimeError ("Error in add-quaternions, line {0}".format
                                (iline))

# test multiplication
with open ('./test/mul-quaternions.csv', 'r') as f:
    r = csv.reader (f, delimiter=',')
    iline=0
    for line in r:
        iline+=1
        data = map (float, line)
        q1 = Quaternion (data [0:4])
        q2 = Quaternion (data [4:8])
        q3 = Quaternion (data [8:12])
        if abs(q1*q2-q3) > 1e-10:
            raise RuntimeError ("Error in mul-quaternions, line {0}".format
                                (iline))

# test multiplication
with open ('./test/sub-quaternions.csv', 'r') as f:
    r = csv.reader (f, delimiter=',')
    iline=0
    for line in r:
        iline+=1
        data = map (float, line)
        q1 = Quaternion (data [0:4])
        q2 = Quaternion (data [4:8])
        q3 = Quaternion (data [8:12])
        if abs((q1-q2)-q3) > 1e-10:
            raise RuntimeError ("Error in sub-quaternions, line {0}".format
                                (iline))

# test multiplication
with open ('./test/div-quaternions.csv', 'r') as f:
    r = csv.reader (f, delimiter=',')
    iline=0
    for line in r:
        iline+=1
        data = map (float, line)
        q1 = Quaternion (data [0:4])
        q2 = Quaternion (data [4:8])
        q3 = Quaternion (data [8:12])
        if abs((q1/q2)-q3) > 1e-10:
            raise RuntimeError ("Error in div-quaternions, line {0}".format
                                (iline))

# test transformation
with open ('./test/transform-quaternions.csv', 'r') as f:
    r = csv.reader (f, delimiter=',')
    iline=0
    for line in r:
        iline+=1
        data = map (float, line)
        q = Quaternion (data [0:4])
        u = np.array (data [4:7])
        v = np.array (data [7:10])
        if np.linalg.norm (q.transform (u) - v) > 1e-10:
            raise RuntimeError \
                ("Error in transform-quaternions, line {0}".format (iline))
