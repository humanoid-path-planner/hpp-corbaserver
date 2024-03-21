# Copyright (c) 2016 CNRS
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

import csv

import numpy as np

from hpp import Quaternion

# Test construction from SO3 matrix
for _ in range(1000):
    rng = np.random.default_rng()
    q1 = Quaternion(rng.random(4)).normalize()
    mat = q1.toRotationMatrix()
    q2 = Quaternion(mat)
    if abs(q1 - q2) > 1e-10 and abs(q1 + q2) > 1e-10:
        raise RuntimeError("Error in conversion quaternion matrix")

# test addition
with open("./test/add-quaternions.csv") as f:
    r = csv.reader(f, delimiter=",")
    iline = 0
    for line in r:
        iline += 1
        data = list(map(float, line))
        q1 = Quaternion(data[0:4])
        q2 = Quaternion(data[4:8])
        q3 = Quaternion(data[8:12])
        if abs(q1 + q2 - q3) > 1e-10:
            raise RuntimeError(f"Error in add-quaternions, line {iline}")

# test multiplication
with open("./test/mul-quaternions.csv") as f:
    r = csv.reader(f, delimiter=",")
    iline = 0
    for line in r:
        iline += 1
        data = list(map(float, line))
        q1 = Quaternion(data[0:4])
        q2 = Quaternion(data[4:8])
        q3 = Quaternion(data[8:12])
        if abs(q1 * q2 - q3) > 1e-10:
            raise RuntimeError(f"Error in mul-quaternions, line {iline}")

# test multiplication
with open("./test/sub-quaternions.csv") as f:
    r = csv.reader(f, delimiter=",")
    iline = 0
    for line in r:
        iline += 1
        data = list(map(float, line))
        q1 = Quaternion(data[0:4])
        q2 = Quaternion(data[4:8])
        q3 = Quaternion(data[8:12])
        if abs((q1 - q2) - q3) > 1e-10:
            raise RuntimeError(f"Error in sub-quaternions, line {iline}")

# test multiplication
with open("./test/div-quaternions.csv") as f:
    r = csv.reader(f, delimiter=",")
    iline = 0
    for line in r:
        iline += 1
        data = list(map(float, line))
        q1 = Quaternion(data[0:4])
        q2 = Quaternion(data[4:8])
        q3 = Quaternion(data[8:12])
        if abs((q1 / q2) - q3) > 1e-10:
            raise RuntimeError(f"Error in div-quaternions, line {iline}")

# test transformation
with open("./test/transform-quaternions.csv") as f:
    r = csv.reader(f, delimiter=",")
    iline = 0
    for line in r:
        iline += 1
        data = list(map(float, line))
        q = Quaternion(data[0:4])
        u = np.array(data[4:7])
        v = np.array(data[7:10])
        if np.linalg.norm(q.transform(u) - v) > 1e-10:
            raise RuntimeError(f"Error in transform-quaternions, line {iline}")
