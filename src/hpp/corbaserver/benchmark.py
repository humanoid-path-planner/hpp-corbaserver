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

from __future__ import print_function
import numpy as np
import sys

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

    def next(self): # Python 2 iteration
        return self.__next__ ()

    def __next__(self): # Python 3 iteration
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
# ## Basic usage ##
#
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
# If you wish to replot datas stored in a file:
# \code{.py}
# from hpp.corbaserver import Benchmark
# import matplotlib.pyplot as plt
# b = Benchmark (None, None, None)
# b.resumeFrom ("datafile")
#
# fig, axes = plt.subplots(nrows=1, ncols=2)
# b.plotTime (axes[0])
# b.plotPathLength (axes[1])
# plt.show ()
#
# \endcode
#
# \sa
# hpp.corbaserver.benchmark.Benchmark.plotTime
# hpp.corbaserver.benchmark.Benchmark.plotPathLength
#
# ## Advanced usage ##
#
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
#
# ## What if HPP crashes ##
#
# \note This section assumes you have installed
# https://github.com/humanoid-path-planner/hpp-tools
#
# You can do the following
# \code{.py}
# try:
#   b.do ()
# except:
#   import sys
#   sys.exit(1)
#
# b.writeResume (filename = "yourresults")
# \endcode
#
# Then, launch your server with this:
# \code{bash}
# hppautorestart hppcorbaserver
# \endcode
#
# Finally, launch your script with this:
# \code{bash}
# hpp_run_benchmark path_to_python_script_file.py
# \endcode
# This will restart the server whenever it crashes and will resume
# the benchmarks where it stopped.
class Benchmark (object):
    ## Used to transform HPP output into seconds
    toSeconds = np.array ([60*60,60,1,1e-3])
    ## The filename of the crash file.
    crashFile = "/tmp/resume.hpp.corbaserver.benchmark.pickle"

    def __init__ (self, client, robot, problemSolver):
        ## A list of seed to initialize the random generator.
        self.seedRange = list(range(1))
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
        self.results['states']=[]

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
                    print("=======================================================")
                    print("Case ", self.getCase (iter))
                    print("=======================================================")
                    self.results['user'].append (initCase (self, self.getCase(iter), iter))
                self.results['time'].append (self.client.problem.solve ())
                self.results['pathLength'].append (self.client.problem.pathLength (self.client.problem.numberPaths()-1))
                self.results['states'].append(self.ps.numberNodes())
            except Exception as err:
                # write current data to restart at the same point
                self.current = iter
                self.writeResume ()
                print(err)
                print("\nOops, something went wrong.\nTo resume at the benchmark just before the crash, use method thisobject.tryResumeAndDelete () before calling method do()\n")
                raise

            print("Solved", iter, "in", self.results['time'][-1])
        i = 0
        nb = self.iterPerCase * len(self.seedRange)
        for c in self.cases:
            t = np.array (self.results['time'][i:i+nb]).dot (self.toSeconds)
            pl = np.array (self.results['pathLength'][i:i+nb])
            nodes = np.array (self.results['states'])
            print("=====================================================")
            print("Case", c)
            print("Mean time (s):", np.mean(t))
            print("Std dev time (s):",  np.std(t))
            print("Mean number of nodes:", np.mean(nodes))
            print("Std dev nb nodes:",  np.std(nodes))
            print("Average length:", np.mean(pl))
            print("std dev length:", np.std(pl))
            print("=====================================================")
            i += nb
        t = np.array (self.results['time']).dot (self.toSeconds)
        nodes = np.array (self.results['states'])
        pl = np.array (self.results['pathLength'])
        print("=====================================================")
        print("All cases together")
        print("Mean time (s):", np.mean(t))
        print("Std dev time (s):",  np.std(t))
        print("Mean number of nodes:", np.mean(nodes))
        print("Std dev nb nodes:",  np.std(nodes))
        print("Average length:", np.mean(pl))
        print("std dev length:", np.std(pl))
        print("=====================================================")
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
            print("Retrieving datas from file", fname)
            self.resumeFrom (fname)
            os.remove (fname)

    def resumeFrom (self, fname):
        import pickle as pk
        with open (fname, 'r') as f:
            cases = pk.load(f)
            if not cases == self.cases:
                print("Cases are different.\nValue in file is :", cases, "\nValue in this instance was:\n", self.cases)
            self.cases = cases

            seedRange = pk.load(f)
            if not seedRange == self.seedRange:
                print("Seed range is different.\nValue in file is :", seedRange, "\nValue in this instance was:\n", self.seedRange)

            iterPerCase = pk.load(f)
            if not iterPerCase == self.iterPerCase:
                print("Number of iteration per case is different.\nValue in file is :", iterPerCase, "\nValue in this instance was:\n", self.iterPerCase)
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

    ## This method create a database which store the benchmark results. 
    # You can then use it in the platform http://plannerarena.org/ to plot your results.
    # \param nameDatabase the name of the created file (extension must be in .db)
    # \param experimentName the name of the current scenario/problem (used when you append several scenario in the same database)
    # \param nameLogFile the name of the text file writed 
    # \param append if True, the current result will be added in the given database.
    def writeDatabase(self,nameDatabase,experimentName = 'default',nameLogFile = 'temp.log',append = False):
        import os
        if os.path.isfile(nameLogFile) : 
            os.remove(nameLogFile)

        if not append and os.path.isfile(nameDatabase):
            os.remove(nameDatabase)

        # write log file :
        # write experiment header :
        log = open(nameLogFile,'w')
        log.write('Experiment '+experimentName+'\n')
        log.write('0 experiment properties\n')
        p = os.popen('hostname')
        log.write('Running on '+p.read())
        p = os.popen('date --rfc-3339=seconds')
        log.write('Starting at '+p.read())
        log.write('<<<| \n')
        log.write('Configuration of the experiment : \n')
        log.write('Robot name : '+self.robot.displayName+'\n')
        log.write('Initial config : '+str(self.ps.getInitialConfig())+'\n')
        for qgoal in self.ps.getGoalConfigs():
            log.write('Goal config : '+str(qgoal)+'\n')

        log.write('Joints bounds : \n')
        for jointName in self.robot.allJointNames :
            if len(self.robot.client.robot.getJointBounds(jointName)) > 0 :
                log.write(jointName+' : '+str(self.robot.client.robot.getJointBounds(jointName))+'\n')

        log.write('|>>> \n')
        log.write('<<<| \n')
        p = os.popen('cat /proc/cpuinfo')
        log.write(p.read())
        log.write('|>>> \n')
        log.write('0 is the random seed\n')
        # hardcoded value : time and memory limit for all the benchmark (required by the parser)
        log.write('0 seconds per run\n')
        log.write('8192 MB per run\n')
        nbRuns = len(self.seedRange) * self.iterPerCase
        log.write(str(nbRuns)+' runs per planner\n')
        t = np.array (self.results['time']).dot (self.toSeconds)
        log.write(str(t.sum()) +' seconds spent to collect the data\n')
        log.write('1 enum type\n')
        log.write('status|Timeout|solved|Crash|Unknown status\n')
        log.write(str(len(self.cases))+" planners\n")

        i=0
        # for each algorithm (case ) : 
        for c in self.cases:
            if c == 'None' :
                log.write('Default\n')
            else :
                log.write(str(c)+'\n') # need a better way to display this
           
            log.write('0 common properties\n')
            log.write('6 properties for each run\n')
            # solved, status, seed and time are mandatory for the parser
            log.write('solved BOOLEAN\n')
            log.write('status ENUM\n')  # for now it's always 1 (= solved)
            log.write('seed INTEGER\n')
            log.write('pathLenght INTEGER\n')
            log.write('graph_states INTEGER\n')
            log.write('time REAL\n')
            log.write(str(nbRuns)+' runs\n')
            nbSeed = len(self.seedRange)
            for s in range(nbSeed):
                for j in range(self.iterPerCase):
                # write a line for each run 
                    log.write('1; 1; '+str(self.seedRange[s])+'; '+str(self.results['pathLength'][i+j])+'; '+str(self.results['states'][ i+j])+"; "+str(t[i+j])+'; \n')
                i += self.iterPerCase

            log.write('. \n')
        
        log.close()
        # compute the database :
        from hpp.corbaserver import ompl_benchmark_statistics as omplBench
        omplBench.readBenchmarkLog(nameDatabase,[nameLogFile],False)
        omplBench.computeViews(nameDatabase,False)
        if nameLogFile=='temp.log' : 
            os.remove(nameLogFile)

    def _printStats(self, times, nodes, pathLengths):
        return "Mean time (s): " + str(np.mean(times)) + '\n' \
            + "Std dev time (s): " + str(np.std(times)) + '\n' \
            + "Mean number of nodes: " + str(np.mean(nodes)) + '\n' \
            + "Std dev nb nodes: " +  str(np.std(nodes)) + '\n' \
            + "Average length: " + str(np.mean(pathLengths)) + '\n' \
            + "std dev length: " + str(np.std(pathLengths))

    def __str__(self):
        res = ""
        for i in range(len(self.results['time'])):
            res += "Time (s): " + str(np.array (self.results['time'][i]).dot (self.toSeconds)) \
                    + "\nNumber of nodes: " + str(self.results['states'][i]) \
                    + "\nLength: " + str(self.results['pathLength'][i]) + '\n'
        times = np.array (self.results['time']).dot (self.toSeconds)
        nodes = np.array (self.results['states'])
        pathLengths = np.array (self.results['pathLength'])
        res += self._printStats(times, nodes, pathLengths)
        return res
