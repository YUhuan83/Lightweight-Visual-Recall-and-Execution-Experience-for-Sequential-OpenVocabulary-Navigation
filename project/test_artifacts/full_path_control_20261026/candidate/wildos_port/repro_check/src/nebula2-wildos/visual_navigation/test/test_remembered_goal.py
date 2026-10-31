import importlib.util
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as Box


SOURCE = (Path(__file__).resolve().parents[1] / "visual_navigation" /
          "wildos" / "remembered_goal.py")
SPEC = importlib.util.spec_from_file_location("remembered_goal", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def observation(x=733.0, y=-12.0, yaw=1.2, frame="spot1/odom"):
    orientation = Box(x=0.0, y=0.0, z=math.sin(yaw / 2),
                      w=math.cos(yaw / 2))
    pose = Box(position=Box(x=x, y=y, z=53.0), orientation=orientation)
    return Box(header=Box(frame_id=frame), pose=Box(pose=pose))


class RememberedGoalTests(unittest.TestCase):
    def test_replays_drivable_observation_position_and_viewing_yaw(self):
        odom = observation()
        result = MODULE.observed_viewpoint_pose(odom, "spot1/odom")
        self.assertEqual((result.position.x, result.position.y), (733.0, -12.0))
        self.assertAlmostEqual(result.orientation.z, math.sin(0.6))
        self.assertIsNot(result, odom.pose.pose)

    def test_rejects_frame_mismatch_or_invalid_orientation(self):
        self.assertIsNone(MODULE.observed_viewpoint_pose(
            observation(frame="map"), "spot1/odom"))
        odom = observation()
        odom.pose.pose.orientation.w = 0.0
        odom.pose.pose.orientation.z = 0.0
        self.assertIsNone(MODULE.observed_viewpoint_pose(odom, "spot1/odom"))

    def test_rejects_nonfinite_position(self):
        self.assertIsNone(MODULE.observed_viewpoint_pose(
            observation(x=float("nan")), "spot1/odom"))


if __name__ == "__main__":
    unittest.main()
