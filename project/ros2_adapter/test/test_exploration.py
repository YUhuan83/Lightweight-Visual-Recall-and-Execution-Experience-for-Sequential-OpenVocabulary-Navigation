import importlib.util
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "exploration", HERE / "wildos_ros2_adapter" / "exploration.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ExplorationTests(unittest.TestCase):
    def test_left_and_right_frontier_bearings(self):
        left = np.zeros((10, 40), dtype=float)
        left[:, :8] = 1.0
        right = np.zeros((10, 40), dtype=float)
        right[:, -8:] = 1.0
        self.assertLess(MODULE.exploration_bearing(left, np.ones_like(left)), 0.0)
        self.assertGreater(MODULE.exploration_bearing(right, np.ones_like(right)), 0.0)

    def test_shape_mismatch_rejected(self):
        with self.assertRaises(ValueError):
            MODULE.exploration_bearing(np.ones((4, 4)), np.ones((3, 4)))

    def test_bearing_step_is_limited(self):
        result = MODULE.smooth_bearing(0.0, np.deg2rad(60.0))
        self.assertAlmostEqual(np.rad2deg(result), 15.0)


if __name__ == "__main__":
    unittest.main()
    