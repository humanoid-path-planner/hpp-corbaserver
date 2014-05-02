import omniORB
omniORB.updateModule("hpp.corbaserver")

import robot_idl
import common_idl
import obstacle_idl
import problem_idl

from client import Client
Configuration = common_idl._0_hpp.Configuration
