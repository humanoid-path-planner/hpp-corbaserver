"""Test running hppcorbaserver on another port."""
import unittest

import os, time
from hpp.corbaserver import Client
from hpp.corbaserver.tools import Tools
import subprocess
from subprocess import DEVNULL, run


class Test(unittest.TestCase):
    def test_server_manager(self):
        """Run the server in background on port 13332

        stdout and stderr outputs of the child process are redirected to devnull.
        preexec_fn is used to ignore ctrl-c signal send to the main script
        (otherwise they are forwarded to the child process)
        """
        # Check that we can't instanciate a client if a server isn't running
        with self.assertRaises(Exception):  # omniORB.CORBA._omni_sys_exc
            Client(port=13332)

        self.process = subprocess.Popen(
            "./hppcorbaserver.sh", stdout=DEVNULL, stderr=DEVNULL, preexec_fn=os.setpgrp
        )
        # give it some time to start
        time.sleep(3)

        client = Client(port=13332)
        client.robot.createRobot("HRP-7")
        self.assertEqual(client.robot.getRobotName(), "HRP-7")
        os.environ["HPP_PORT"] = "13332"
        client = Client()
        self.assertEqual(client.robot.getRobotName(), "HRP-7")
        tools = Tools(port="13332")
        tools.shutdown()


if __name__ == "__main__":
    unittest.main()
