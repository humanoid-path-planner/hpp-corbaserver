import unittest

# Check imports
from hpp.corbaserver import Client
from hpp.corbaserver.robot import Robot
from hpp.utils import ServerManager


class Test(unittest.TestCase):
    def test_createConstraints(self):
        with ServerManager("../src/hppcorbaserver"):
            self.client = Client()
            self.client.robot.createRobot("test")
            self.assertEqual(self.client.robot.getRobotName(), "test")

            c_problem = self.client.problem.getProblem()
            c_robot = c_problem.robot()

            c_constraint_set = self.client.problem.createConstraintSet(c_robot, "myset")
            assert c_constraint_set.getConfigProjector() is None
            c_config_proj = self.client.problem.createConfigProjector(
                c_robot, "myproj", 1e-4, 40
            )
            c_constraint_set.addConstraint(c_config_proj)
            assert c_constraint_set.getConfigProjector() is not None
            assert c_constraint_set.getConfigProjector().name() == c_config_proj.name()


if __name__ == "__main__":
    unittest.main()
