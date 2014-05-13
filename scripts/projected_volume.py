#/usr/bin/env python
import time
from hpp.corbaserver import Client
from hpp_corbaserver.hpp import Configuration
from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.client import Client as WsClient
from hrp2 import Robot
import rospy

class ProjectedVolume ():
        def __init__ (self):
                self.robot_interface = Robot ()
                self.robot_interface.setTranslationBounds (-3, 3, -3, 3, 0, 1)
                self.cl = self.robot_interface.client
                self.robot = self.robot_interface.client.robot
                self.scene_publisher = ScenePublisher (self.robot_interface.jointNames [4:])
                self.q0 = self.robot_interface.getInitialConfig ()
                self.q1 = [0.0, 0.0, 0.705, 1.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]
                self.q2 = [0.0, 0.0, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]


        def setConfig(self,q):
                self.robot.setCurrentConfig(q)
        def setRandomConfig(self):
                self.robot.setCurrentConfig(qrand)

        def display_full(self, q):
                r = rospy.Rate(1)

                self.robot.setCurrentConfig(q)
                r.sleep()
                self.scene_publisher(q)

                capsulePos = self.robot.computeVolume()
                #access 3 elements at a time (x,y,z)
                scale = 0.5
                for i in range(0,len(capsulePos),3):
                        self.scene_publisher.addSphere(scale*capsulePos[i], scale*capsulePos[i+1], scale*capsulePos[i+2])
                self.scene_publisher.publishObjects()
                r.sleep()
        def display(self, q):
                r = rospy.Rate(1)

                self.robot.setCurrentConfig(q)
                r.sleep()
                self.scene_publisher(q)

                capsulePos = self.robot.computeVolume()
                #access 3 elements at a time (x,y,z)
                scale = 0.5
                for i in range(0,len(capsulePos),3):
                        self.scene_publisher.addSphere(0.5, scale*capsulePos[i+1], scale*capsulePos[i+2])
                self.scene_publisher.publishObjects()
                r.sleep()
