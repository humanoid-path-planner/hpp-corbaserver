#/usr/bin/env python
import time
from hpp.corbaserver import Client
from hpp_corbaserver.hpp import Configuration
from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.client import Client as WsClient
from hrp2 import Robot
from math import pi

#################################################################
robot = Robot ()
robot.setTranslationBounds (-3, 3, -3, 3, 0, 1)
cl = robot.client
physical_robot = robot.client.robot
#################################################################
# publish half sitting position in rviz
#################################################################
r = ScenePublisher (robot.jointNames [4:])
q0 = robot.getInitialConfig ()
r(q0)
#################################################################
# Add constraints
#################################################################
#wcl = WsClient ()
#wcl.problem.addStaticStabilityConstraints (q0)
# lock hands in closed position
lockedDofs = robot.leftHandClosed ()
for index, value in lockedDofs:
    cl.problem.lockDof (index, value)

lockedDofs = robot.rightHandClosed ()
for index, value in lockedDofs:
    cl.problem.lockDof (index, value)
#################################################################
# create init and goal config (hand movement)
#################################################################
q1 = [0.0, 0.0, 0.705, 1.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]
q1 = cl.problem.applyConstraints (q1)

q2 = [0.0, 0.0, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]
q2 = cl.problem.applyConstraints (q2)

#################################################################
# plan trajectory
#################################################################
cl.problem.directPath (q1, q2)
#print "RRT computation started..."
#cl.problem.setInitialConfig (q1)
#cl.problem.addGoalConfig (q2)
##cl.problem.solve ()
#################################################################
pathId = cl.problem.numberPaths () - 1
length = cl.problem.pathLength (pathId)

t = 0
dt = 0.01
while t < length :
  q = cl.problem.configAtDistance (pathId, t)
  r(q)
  t += dt
  print cl.robot.getCenterOfMass()
  time.sleep (dt)
#################################################################
