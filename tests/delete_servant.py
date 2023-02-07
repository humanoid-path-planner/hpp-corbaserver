"""Test deleting twice the same object does not throw."""
import unittest

from hpp.corbaserver import Client
from hpp.corbaserver.tools import Tools
from hpp.utils import ServerManager

class Test(unittest.TestCase):
    def test_delete_servant_from_object_twice(self):
        with ServerManager("../src/hppcorbaserver"):
            self.client = Client()
            self.client.robot.createRobot("name")
            obj = self.client.problem.getProblem()
            self.tools = Tools()
            self.tools.deleteServantFromObject(obj)
            # This one should not throw
            self.tools.deleteServantFromObject(obj)
