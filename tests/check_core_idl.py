from hpp.corbaserver import ProblemSolver, newProblem
from hpp.corbaserver.tools import Tools
from hpp.corbaserver.ur5 import Robot

newProblem()
robot = Robot("ur5")
ps = ProblemSolver(robot)

problem = ps.hppcorba.problem.getProblem()

distance = problem.getDistance()
sm = problem.getSteeringMethod()
pv = problem.getPathValidation()
problem.setDistance(distance)  # does nothing because the distance did not change.

tools = Tools()

q0 = robot.getCurrentConfig()
qr = robot.shootRandomConfig()
v1 = distance.value(q0, qr)
weights = distance.getWeights()
weights[0] = 0
distance.setWeights(weights)
v2 = distance.value(q0, qr)
assert v1 != v2

path = sm.call(q0, qr)

path.value(0)
path.derivative(0.5, 1)
path.outputSize()
print(path.str())

isValid, validPart, report = pv.validate(path, False)

# Decrease the object ref count of the underlying HPP object
tools.deleteServantFromObject(distance, path, sm, pv, problem)

tools.shutdown()
