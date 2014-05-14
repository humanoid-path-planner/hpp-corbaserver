#/usr/bin/env python
import time
from hpp.corbaserver import Client
from hpp_corbaserver.hpp import Configuration
from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.client import Client as WsClient
from hrp2 import Robot
import rospy
import numpy

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
                self.q = self.q0
                self.distanceToProjection = 1

        def setConfig(self,q):
                self.q = q
                self.robot.setCurrentConfig(self.q)

        def setRandomConfig(self):
                self.robot.setCurrentConfig(qrand)

        def displayConvexHullProjection(self):
                from scipy.spatial.qhull import Delaunay
                from convex_hull import convex_hull

                #self.idx = numpy.concatenate([self.hull[:,0:1],self.hull[:,2:3]],1)
                #use y,z coordinates only
                self.cap2d = numpy.concatenate([self.capsule[:,1:2],self.capsule[:,2:3]],1) 

                #make cap2d hashable
                self.cap2dhash = map(tuple, self.cap2d)
                self.hull = convex_hull(self.cap2dhash)

                r = rospy.Rate(1)
                r.sleep()
                #for i in range(0,len(self.hull),1):
                #        self.scene_publisher.addSphere(0.5,
                #                        self.hull[i][0],
                #                        self.hull[i][1],
                #                        0.5,
                #                        0.01,
                #                        0.01)
                #self.scene_publisher.addPolygon(self.distanceToProjection, self.hull)
                self.scene_publisher.addPolygonFilled(self.distanceToProjection, self.hull)
                self.scene_publisher.publishObjects()
                r.sleep()
                r.sleep()

        def computeConvexHullProjection(self):
                self.q_grad = self.robot.gradientConfigurationWrtProjection (self.q)
                self.q_grad.insert(3,0)
                self.q_grad[0]=0
                self.q_grad[1]=0
                self.q_grad[2]=0
                #make sure that quaternions are not changed
                self.q_grad[3]=0
                self.q_grad[4]=0
                self.q_grad[5]=0
                self.q_grad[6]=0
                self.q_new = [x+y for x,y in zip(self.q,self.q_grad)]
                print self.q_new[0:8]
                self.setConfig(self.q_new)
                self.displayRobot()
                
        def displayRobot(self):
                r = rospy.Rate(1)
                r.sleep()
                self.scene_publisher(self.q)
        def computeCapsuleProjection(self):
                capsulePos = self.robot.computeVolume()
                #access 3 elements at a time (x,y,z)
                self.capsule = numpy.empty((len(capsulePos)/5, 5))
                ctr = 0
                for i in range(0,len(capsulePos),5):
                        #capsule containts x,y,z,radius,length
                        self.capsule[ctr] = numpy.array(capsulePos[i:i+5])
                        ctr+=1

        def displayCapsuleProjection(self):
                r = rospy.Rate(1)
                r.sleep()
                self.scene_publisher(self.q)
                for i in range(0,len(self.capsule),1):
                        self.scene_publisher.addSphere(self.distanceToProjection,
                                        self.capsule[i][1],
                                        self.capsule[i][2],
                                        0.001,
                                        self.capsule[i][3],
                                        self.capsule[i][3]
                                        )

                self.scene_publisher.publishObjects()
                r.sleep()
                r.sleep()
