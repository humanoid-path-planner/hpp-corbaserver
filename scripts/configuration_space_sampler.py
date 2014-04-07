#/usr/bin/env python
import time
from hpp.corbaserver import Client
from hpp_corbaserver.hpp import Configuration
from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.wholebody_step.client import Client as WsClient
from hrp2 import Robot
from math import pi

class ConfigurationSpaceSampler:
  robot = []
  current_sample = []
  rviz = []

  def __init__(self):
    self.robot = Robot ()
    self.robot.setTranslationBounds (-3, 3, -3, 3, 0, 1)
    self.cl = self.robot.client

    self.current_sample = self.robot.getInitialConfig()
    self.rviz = ScenePublisher (self.robot.jointNames [4:])

    #################################################################
    # Add constraints
    #################################################################
    self.wcl = WsClient ()
    self.wcl.problem.addStaticStabilityConstraints (self.current_sample)
    # lock hands in closed position
    lockedDofs = self.robot.leftHandClosed ()
    for index, value in lockedDofs:
        self.cl.problem.lockDof (index, value)

    lockedDofs = self.robot.rightHandClosed ()
    for index, value in lockedDofs:
        self.cl.problem.lockDof (index, value)

    self.display(self.current_sample)

  def project(self,q_in):
    status, q_proj = self.cl.problem.applyConstraints (q_in)
    if status == 0:
      return q_proj
    else:
      return null

  def display(self,q_in):
    q=self.project(q_in)
    #for i in range(1,100):
    self.rviz(q)
    

