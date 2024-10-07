import unittest

from hpp.corbaserver import Client

# Check imports
from hpp.corbaserver.tools import Tools
from hpp.utils import ServerManager


class Test(unittest.TestCase):
    def test_tools(self):
        with ServerManager("../src/hppcorbaserver"):
            tools = Tools()
            cl = Client()
            cl.robot.createRobot("foo")
            pb = cl.problem.getProblem()

            object_ids = tools.getAllServants()
            assert len(object_ids) == 1
            assert object_ids[0] == cl.orb.object_to_string(pb)

            pb2 = cl.orb.string_to_object(object_ids[0])
            assert type(pb) is type(pb2)
            pb2.persistantStorage(False)

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
