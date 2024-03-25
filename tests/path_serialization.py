"""Test deleting twice the same object does not throw."""

import unittest

from hpp.corbaserver import Client
from hpp.utils import ServerManager

urdf_string = """
<robot name="test_robot">
<link name="base_link"/>
<link name="child_link"/>
<joint name="joint" type="revolute">
  <parent link="base_link"/>
  <child link="child_link"/>
  <limit effort="50000" lower="-1" upper="1" velocity="10000"/>
</joint>
</robot>
"""
filename = "./tmp-path.bin"


class Test(unittest.TestCase):
    def test_save_then_load_path(self):
        with ServerManager("../src/hppcorbaserver"):
            self.client = Client()
            self.client.robot.loadRobotModelFromString(
                "robot", "anchor", urdf_string, ""
            )
            ok, pid, msg = self.client.problem.directPath(
                [
                    -0.3,
                ],
                [0.3],
                False,
            )
            assert ok
            path = self.client.problem.getPath(pid)
            self.client.problem.savePath(path, filename)

        with ServerManager("../src/hppcorbaserver"):
            self.client = Client()
            self.client.robot.loadRobotModelFromString(
                "robot", "anchor", urdf_string, ""
            )
            path = self.client.problem.loadPath(filename)

            assert path.initial() == [-0.3]
            assert path.end() == [0.3]


if __name__ == "__main__":
    unittest.main()
