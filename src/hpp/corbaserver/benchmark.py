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

## \cond
class _BenchmarkIter (object):
    def __init__ (self, seedI, caseI, iterPerCaseI, case = None):
        self.seedI = seedI
        self.caseI = caseI
        self.case = case
        self.newCase = True;
        self.iterPerCaseI = iterPerCaseI

    def __repr__ (self):
        if self.case is None:
            return "iteration " + str(self.iterPerCaseI) + " for case " + str(self.caseI) + " at seed " + str(self.seedI)
        else:
            return "iteration " + str(self.iterPerCaseI) + " for case " + str(case) + " at seed " + str(self.seedI)

class _BenchmarkIterator (object):
    def __init__(self, seedRange, cases, iterPerCase, initCase, startAt = None):
        self.seedRangeL = len(seedRange)
        self.casesL = len(cases)
        self.iterPerCase = iterPerCase
        if startAt is None:
            self.current = _BenchmarkIter (0, 0, 0)
        else:
            self.current = startAt
        self.start = True

    def __iter__(self):
        return self

    def next(self): # Python 3: def __next__(self)
        if self.start:
            self.newCase = True
            self.start = False
            return self.current

        self.current.newCase = False
        self.current.iterPerCaseI += 1
        if self.current.iterPerCaseI >= self.iterPerCase:
            self.current.seedI += 1
            if self.current.seedI >= self.seedRangeL:
                self.current.newCase = True
                self.current.caseI += 1
                if self.current.caseI >= self.casesL:
                    raise StopIteration
                self.current.seedI = 0
            self.current.iterPerCaseI = 0
        return self.current
## \endcond

## class to do benchmarking
#
# Basic usage
# \code{.py}
# # Here goes your script to load the problem.
# robot = hpp.corbaserver.robot.Robot (...)
# ps = hpp.corbaserver.problem_solver.ProblemSolver (...)
# ...
# 
# from hpp.corbaserver import Benchmark
# benchmark = Benchmark (robot.client, robot, ps)
# benchmark.iterPerCase = 10
# results = benchmark.do()
# \endcode
#
# \sa hpp.corbaserver.benchmark.Benchmark.do
# hpp.corbaserver.benchmark.Benchmark.seedRange
# 
# Advanced usage
# \code{.py}
# from hpp.corbaserver import Benchmark
# b = Benchmark (robot.client, robot, ps)
# b.seedRange = range (10)
# b.iterPerCase = 10
#
# b.cases = list()
# for type in ["Progressive", "Global"]:
#     for param in [0.1, 0.2]:
#         b.cases.append((type,param))
# b.cases.append(("None",0.1))
#
# b.tryResumeAndDelete ()
#
# def initialize (bench, case, iter):
#     bench.ps.selectPathProjector (case[0], case[1])
#
# results = b.do(initCase = initialize)
# \endcode
#
# \sa
# hpp.corbaserver.benchmark.Benchmark.cases
# hpp.corbaserver.benchmark.Benchmark.iterPerCase
# hpp.corbaserver.benchmark.Benchmark.tryResumeAndDelete
class Benchmark (object):
    ## Used to transform HPP output into seconds
    toSeconds = np.array ([60*60,60,1,1e-3])
    ## The filename of the crash file.
    crashFile = "/tmp/resume.hpp.corbaserver.benchmark.pickle"

    def __init__ (self, client, robot, problemSolver):
        ## A list of seed to initialize the random generator.
        self.seedRange = range (1)
        ## A list of cases for which benchmarking will be done.
        self.cases = [None]
        ## Number of times one case is repeated (for one seed).
        self.iterPerCase = 1

        self.client = client
        self.robot = robot
        self.ps = problemSolver

        self.current = None
        # internal datas
        self.results = dict ()
        self.results['user'] = []
        self.results['time'] = []
        self.results['pathLength'] = []

    ## Solve the same problem for the specified cases, for various random seed.
    # \param initCase a function of 3 arguments:
    #        - the calling Benchmark instance
    #        - the current element of the list of Benchmark.cases
    #        - the current _BenchmarkIterator (Normally not useful).
    def do(self, initCase = lambda this, case, iter: None):
        for iter in _BenchmarkIterator (self.seedRange, self.cases, self.iterPerCase, initCase, startAt = self.current):
            self.client.problem.clearRoadmap ()
            self.client.problem.setRandomSeed (self.seedRange[iter.seedI])
            try:
                if iter.newCase:
                    print "======================================================="
                    print "Case ", self.getCase (iter)
                    print "======================================================="
                    self.results['user'].append (initCase (self, self.getCase(iter), iter))
                self.results['time'].append (self.client.problem.solve ())
                self.results['pathLength'].append (self.client.problem.pathLength (self.client.problem.numberPaths()-1))
            except:
                # write current data to restart at the same point
                self.current = iter
                self.writeResume ()
                print "\nOops, something went wrong.\nTo resume at the benchmark just before the crash, use method thisobject.tryResumeAndDelete () before calling method do()\n"
                raise

            print "Solved", iter, "in", self.results['time'][-1]
        i = 0
        nb = self.iterPerCase * len(self.seedRange)
        for c in self.cases:
            t = np.array (self.results['time'][i:i+nb]).dot (self.toSeconds)
            pl = np.array (self.results['pathLength'][i:i+nb])
            print "====================================================="
            print "Case", c
            print "Mean time (s):", np.mean(t)
            print "Std dev time (s):",  np.std(t)
            print "Average length:", np.mean(pl)
            print "std dev length:", np.std(pl)
            print "====================================================="
            i += nb
        t = np.array (self.results['time']).dot (self.toSeconds)
        pl = np.array (self.results['pathLength'])
        print "====================================================="
        print "All cases together"
        print "Mean time (s):", np.mean(t)
        print "Std dev time (s):",  np.std(t)
        print "Average length:", np.mean(pl)
        print "std dev length:", np.std(pl)
        print "====================================================="
        return t, pl

    def getCase (self, iter):
        return self.cases[iter.caseI]

    ## Write data to file.
    # \param filename if None, it uses member Benchmark.crashFile
    def writeResume (self, filename = None):
        if filename is None: fname = self.crashFile
        else: fname = filename
        import pickle as pk
        with open (fname, 'w') as f:
            pk.dump(self.cases, f)
            pk.dump(self.seedRange, f)
            pk.dump(self.iterPerCase, f)
            pk.dump(self.current, f)
            pk.dump(self.results, f)

    ## In case of crash of HPP, the benchmark class writes temporary datas to a file.
    #  The method will check if the crash file exists and:
    #  - if it exists, the benchmarking will be initialized at the state before the crash. No datas are lost.
    #    The crash file is deleted after having been loaded.
    #  - if it does not exist, the method does nothing.
    # \param filename if None, it uses member Benchmark.crashFile
    def tryResumeAndDelete (self, filename = None):
        if filename is None: fname = self.crashFile
        else: fname = filename
        import os
        if os.path.isfile (fname):
            print "Retrieving datas from file", fname
            self.resumeFrom (fname)
            os.remove (fname)

    def resumeFrom (self, fname):
        import pickle as pk
        with open (fname, 'r') as f:
            cases = pk.load(f)
            if not cases == self.cases:
                print "Cases are different.\nValue in file is :", cases, "\nValue in this instance was:\n", self.cases
            self.cases = cases

            seedRange = pk.load(f)
            if not seedRange == self.seedRange:
                print "Seed range is different.\nValue in file is :", seedRange, "\nValue in this instance was:\n", self.seedRange

            iterPerCase = pk.load(f)
            if not iterPerCase == self.iterPerCase:
                print "Number of iteration per case is different.\nValue in file is :", iterPerCase, "\nValue in this instance was:\n", self.iterPerCase
            self.iterPerCase = iterPerCase

            self.current = pk.load(f)

            self.results = pk.load(f)

    def plotTime(self, axes):
        # Generate datas
        times = list()
        i = 0
        nb = self.iterPerCase * len(self.seedRange)
        for c in self.cases:
            times.append(np.array (self.results['time'][i:i+nb]).dot (self.toSeconds))
            i += nb

        self._boxplot (axes, times, "Time (s)")
        return times

    def plotPathLength(self, axes):
        # Generate datas
        pls = list()
        i = 0
        nb = self.iterPerCase * len(self.seedRange)
        for c in self.cases:
            pls.append(np.array (self.results['pathLength'][i:i+nb]))
            i += nb

        self._boxplot (axes, pls, "Path length")
        return pls

    def _boxplot(self, axes, datas, ylabel):
        import matplotlib.pyplot as plt
        # rectangular box plot
        bplot = axes.boxplot(datas,
                vert=True,   # vertical box aligmnent
                patch_artist=True)   # fill with color

        # adding horizontal grid lines
        axes.yaxis.grid(True)
        axes.set_xticks([x+1 for x in range(len(self.cases))], )
        axes.set_ylabel(ylabel)

        # add x-tick labels
        plt.setp(axes, xticks=[x+1 for x in range(len(self.cases))],
                xticklabels=[str(c) for c in self.cases])
