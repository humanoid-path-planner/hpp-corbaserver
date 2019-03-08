# coding: utf-8
from hpp.corbaserver.ur5 import Robot
from hpp.corbaserver import ProblemSolver, newProblem
from hpp.corbaserver.tools import Tools

newProblem()
robot = Robot ("ur5")
ps = ProblemSolver (robot)
distance = ps.hppcorba.problem.getDistance()
tools = Tools()

q0 = robot.getCurrentConfig()
qr = robot.shootRandomConfig()
distance.value (q0, qr)
weights = distance.getWeights()
weights[0] = 0
distance.setWeights (weights)
distance.value (q0, qr)

sm = ps.hppcorba.problem.getSteeringMethod ()

path = sm.call (q0, qr)

path.value(0)
path.derivative(0.5, 1)
path.outputSize()

tools.deleteServantFromObject(distance)
tools.deleteServantFromObject(path)
tools.deleteServantFromObject(sm)

tools.shutdown()