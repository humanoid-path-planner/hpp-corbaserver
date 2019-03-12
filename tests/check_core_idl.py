# coding: utf-8
from hpp.corbaserver.ur5 import Robot
from hpp.corbaserver import ProblemSolver, newProblem
from hpp.corbaserver.tools import Tools

newProblem()
robot = Robot ("ur5")
ps = ProblemSolver (robot)

# As of now, this isn't safe because
# it is hard to know when C++ class ProblemSolver
# deletes the current problem and create a new one.
problem = ps.hppcorba.problem.getProblem()

distance = problem.getDistance()
sm = problem.getSteeringMethod ()
pv = problem.getPathValidation ()
problem.setDistance (distance) # does nothing because the distance did not change.

tools = Tools()

q0 = robot.getCurrentConfig()
qr = robot.shootRandomConfig()
v1 = distance.value (q0, qr)
weights = distance.getWeights()
weights[0] = 0
distance.setWeights (weights)
v2 = distance.value (q0, qr)
assert v1 != v2

path = sm.call (q0, qr)

path.value(0)
path.derivative(0.5, 1)
path.outputSize()

isValid, validPart, report = pv.validate (path, False)

# Decrease the object ref count of the underlying HPP object
tools.deleteServantFromObject(distance)
tools.deleteServantFromObject(path)
tools.deleteServantFromObject(sm)
tools.deleteServantFromObject(pv)
# As of now, there is no ref count on class Problem
tools.deleteServantFromObject(problem)

tools.shutdown()
