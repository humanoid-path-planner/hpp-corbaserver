import unittest

# Check imports
from hpp.corbaserver import Client
from hpp.corbaserver.robot import Robot
from hpp.utils import ServerManager


class Test(unittest.TestCase):
    def test_createRobotFromPython(self):
        with ServerManager("../src/hppcorbaserver"):
            self.client = Client()
            self.client.robot.createRobot("test")
            self.assertEqual(self.client.robot.getRobotName(), "test")

            types = [
                "FreeFlyer",
                "Planar",
                "RX",
                "RY",
                "RZ",
                "RUBX",
                "RUBY",
                "RUBZ",
                "PX",
                "PY",
                "PZ",
                "Translation",
            ]

            parent = ""
            for i, type in enumerate(types):
                jn = "joint_{}_{}".format(type, i)
                bn = "body_{}_{}".format(type, i)
                self.client.robot.appendJoint(parent, jn, type, [0, 0, 0, 0, 0, 0, 1])
                self.client.robot.createSphere(bn, 0.001)
                self.client.robot.addObjectToJoint(jn, bn, [0, 0, 1, 0, 0, 0, 1])
                parent = jn

            robot = Robot(client=self.client)

            n = len("JointModel")
            _types = [self.client.robot.getJointType(jn)[n:] for jn in robot.jointNames]
            self.assertListEqual(types, _types)


if __name__ == "__main__":
    unittest.main()
