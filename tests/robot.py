from subprocess import Popen
import unittest, sys

from hpp.corbaserver import Client
from hpp.corbaserver.robot import Robot

class Test (unittest.TestCase):
    @classmethod
    def setUpClass (cls):
        cls.server = Popen (["../src/hppcorbaserver"], stdout = sys.stdout)
        # Give some time to the server to start
        from time import sleep
        sleep(0.01)
        cls.server.poll()
        if cls.server.returncode is not None: print "Process died"

    def setUp (self):
        self.client = Client()

    @classmethod
    def tearDownClass (cls):
        cls.server.kill()
        # print "stop hppcorbaserver", cls.server.pid
        cls.server.wait()

    def test_createRobotFromPython (self):
        self.client.robot.createRobot ("test")
        self.assertEqual (self.client.robot.getRobotName(), "test")

        types = ["FreeFlyer", "Planar",
                "RX", "RY", "RZ", "RUBX", "RUBY", "RUBZ", "PX", "PY", "PZ",
                "Translation"]

        parent = ""
        for i, type in zip (range(len(types)), types):
            jn = "joint_{}_{}".format(type, i)
            bn = "body_{}_{}".format(type, i)
            self.client.robot.appendJoint (parent, jn, type, [0,0,0,0,0,0,1])
            self.client.robot.createSphere (bn, 0.001)
            self.client.robot.addObjectToJoint (jn, bn, [0,0,1,0,0,0,1])
            parent = jn

        robot = Robot(client = self.client)

        n = len("JointModel")
        _types = [ self.client.robot.getJointType(jn)[n:] for jn in robot.jointNames ]
        self.assertListEqual (types, _types)

if __name__ == '__main__':
    unittest.main()
