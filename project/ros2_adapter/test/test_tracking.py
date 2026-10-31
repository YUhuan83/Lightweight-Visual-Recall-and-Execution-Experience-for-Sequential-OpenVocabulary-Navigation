import importlib.util
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "tracking", HERE / "wildos_ros2_adapter" / "tracking.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TrackingTests(unittest.TestCase):
    def test_three_consistent_observations_confirm(self):
        track = {"observations": [], "count": 0, "misses": 0, "confirmed": None}
        self.assertIsNone(MODULE.update_track(track, [1, 2, 3]))
        self.assertIsNone(MODULE.update_track(track, [1.5, 2, 3]))
        self.assertEqual(MODULE.update_track(track, [1, 2.5, 3]), [1.0, 2.0, 3.0])

    def test_large_jump_restarts_unconfirmed_track(self):
        track = {"observations": [], "count": 0, "misses": 0, "confirmed": None}
        MODULE.update_track(track, [1, 2, 3])
        MODULE.update_track(track, [30, 30, 3])
        self.assertEqual(track["count"], 1)

    def test_confirmed_target_clears_after_long_loss(self):
        track = {"observations": [], "count": 0, "misses": 0, "confirmed": None}
        for point in ([1, 2, 3], [1.1, 2, 3], [1, 2.1, 3]):
            MODULE.update_track(track, point)
        result = None
        for _ in range(11):
            result = MODULE.update_track(track, None)
        self.assertIsNone(result)
        self.assertIsNone(track["confirmed"])


if __name__ == "__main__":
    unittest.main()
