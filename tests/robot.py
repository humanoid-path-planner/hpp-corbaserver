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

    def test_pinocchio_device(self):
        with ServerManager("../src/hppcorbaserver"):
            self.client = Client()
            self.client.robot.createRobot("test")
            self.client.robot.appendJoint(
                "", "joint", "FreeFlyer", [0, 0, 0, 0, 0, 0, 1]
            )

            self.assertEqual(self.client.robot.getRobotName(), "test")

            c_problem = self.client.problem.getProblem()
            c_robot = c_problem.robot()

            assert c_robot.configSize() == 7
            assert c_robot.numberDof() == 6

            q = [0, 0, 0, 0, 0, 0, 1]
            v = [0.0] * 6
            a = [0.0] * 6
            assert c_robot.getCurrentConfiguration() == q
            assert c_robot.getCurrentVelocity() == v
            assert c_robot.getCurrentAcceleration() == a
            q[0] = 1.0
            c_robot.setCurrentConfiguration(q)
            assert c_robot.getCurrentConfiguration() == q
            c_robot.setCurrentVelocity(v)
            c_robot.setCurrentAcceleration(a)


if __name__ == "__main__":
    unittest.main()
