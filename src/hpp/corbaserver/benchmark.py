#!/usr/bin/env python
# Copyright (c) 2016 CNRS
# Author: Joseph Mirabel
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

## class to do benchmarking
class Benchmark (object):
    toSeconds = np.array ([60*60,60,1,1e-3])

    def __init__ (self, client, robot, problemSolver):
        self.client = client
        self.robot = robot
        self.ps = problemSolver
        self.seedRange = xrange (1)
        self.iterPerSeed = 1

    def do(self, caseDesc = "the current case", initializer = lambda robot, ps: None):
        time = []
        pathLength = []
        for seed in self.seedRange:
            for i in xrange(self.iterPerSeed):
                self.client.problem.clearRoadmap ()
                self.client.problem.setRandomSeed (seed)
                initializer (self.robot, self.ps)
                time.append (self.client.problem.solve ())
                pathLength.append (self.client.problem.pathLength (self.client.problem.numberPaths()-1))
                print "Solved iter", i, "for seed", seed,"in",time[-1]
        t = np.array (time).dot (self.toSeconds)
        pl = np.array (pathLength)
        print "====================================================="
        print "Result for", caseDesc
        print "Mean time (s):", np.mean(t)
        print "Std dev time (s):",  np.std(t)
        print "Average length:", np.mean(pl)
        print "std dev length:", np.std(pl)
        print "====================================================="
        return t
