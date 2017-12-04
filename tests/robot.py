from hpp.corbaserver import Client

cl = Client()

cl.robot.createRobot ("test")
cl.robot.appendJoint ("", "root_joint", "planar", [0,0,0,0,0,0,1])
cl.robot.createSphere ("root_body", 0.001)
cl.robot.addObjectToJoint ("root_joint", "root_body", [0,0,1,0,0,0,1])

from hpp.corbaserver.robot import Robot
robot = Robot()
