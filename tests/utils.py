"""Test the content of hpp/utils.py."""
import unittest

from hpp.corbaserver import Client
from hpp.utils import ServerManager


class Test(unittest.TestCase):
    def test_server_manager(self):
        # Check that we can't instanciate a client if a server isn't running
        with self.assertRaises(Exception):  # omniORB.CORBA._omni_sys_exc
            Client()

        # Check that we can instanciate a working client if a server is running
        with ServerManager("../src/hppcorbaserver"):
            client = Client()
            client.robot.createRobot("HRP-7")
            self.assertEqual(client.robot.getRobotName(), "HRP-7")

        # Check that we can't instanciate a client anymore if the server was killed
        with self.assertRaises(Exception):  # omniORB.CORBA._omni_sys_exc
            Client()


if __name__ == "__main__":
    unittest.main()
