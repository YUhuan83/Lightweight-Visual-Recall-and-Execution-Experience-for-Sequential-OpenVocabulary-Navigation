import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("local_wildos_demo_under_test", HERE / "local_wildos_demo.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TargetTrackTests(unittest.TestCase):
    def test_requires_consecutive_confirmations(self):
        track = {"candidate": None, "count": 0, "confirmed": None, "observations": []}
        self.assertIsNone(MODULE.target_track_update(track, (10.0, 10.0), 8.0, 3))
        self.assertIsNone(MODULE.target_track_update(track, None, 8.0, 3))
        self.assertIsNone(MODULE.target_track_update(track, (10.5, 10.0), 8.0, 3))
        self.assertEqual(MODULE.target_track_update(track, (10.0, 10.5), 8.0, 3), (10.0, 10.0))

    def test_large_jump_restarts_confirmation(self):
        track = {"candidate": None, "count": 0, "confirmed": None, "observations": []}
        MODULE.target_track_update(track, (1.0, 1.0), 8.0, 2)
        self.assertEqual(MODULE.target_track_update(track, (1.5, 1.0), 8.0, 2), (1.25, 1.0))
        self.assertEqual(MODULE.target_track_update(track, (30.0, 30.0), 8.0, 2), (1.25, 1.0))
        self.assertEqual(track["count"], 2)

    def test_missing_candidate_clears_confirmation(self):
        track = {"candidate": None, "count": 0, "confirmed": None, "observations": []}
        MODULE.target_track_update(track, (1.0, 1.0), 8.0, 1)
        self.assertIsNotNone(track["confirmed"])
        self.assertIsNotNone(MODULE.target_track_update(track, None, 8.0, 1))
        self.assertIsNotNone(MODULE.target_track_update(track, None, 8.0, 1))
        self.assertIsNone(MODULE.target_track_update(track, None, 8.0, 1))
        self.assertEqual(track["count"], 0)

    def test_candidate_survives_short_detection_gap(self):
        track = {"candidate": None, "count": 0, "confirmed": None, "misses": 0}
        MODULE.target_track_update(track, (1.0, 1.0), 8.0, 1)
        self.assertEqual(MODULE.target_track_update(track, None, 8.0, 1), (1.0, 1.0))
        self.assertEqual(track["misses"], 1)

    def test_confirmed_target_is_locked_against_large_depth_jump(self):
        track = {"candidate": None, "count": 0, "confirmed": None,
                 "misses": 0, "observations": []}
        MODULE.target_track_update(track, (10.0, 10.0), 8.0, 1)
        self.assertEqual(MODULE.target_track_update(track, (30.0, 30.0), 8.0, 1), (10.0, 10.0))
        self.assertEqual(track["confirmed"], (10.0, 10.0))

    def test_confirmed_target_is_immutable_for_small_drift(self):
        track = {"candidate": None, "count": 0, "confirmed": None,
                 "misses": 0, "observations": []}
        MODULE.target_track_update(track, (10.0, 10.0), 8.0, 1)
        self.assertEqual(MODULE.target_track_update(track, (10.5, 10.5), 8.0, 1), (10.0, 10.0))

    def test_target_progress_delta_is_positive_when_distance_decreases(self):
        before = float(np.linalg.norm(np.array([0.0, 0.0]) - np.array([10.0, 0.0])))
        after = float(np.linalg.norm(np.array([2.0, 0.0]) - np.array([10.0, 0.0])))
        self.assertGreater(before - after, 0.0)

    def test_direct_target_path_requires_clear_short_segment(self):
        path = MODULE.direct_target_path([0.0, 0.0], [4.0, 0.0], [], 2.0)
        np.testing.assert_allclose(path[-1], [2.0, 0.0])
        obstacles = np.array([[1.0, 0.0]], dtype=float)
        self.assertIsNone(MODULE.direct_target_path([0.0, 0.0], [4.0, 0.0], obstacles, 2.0))


class HeadMapTests(unittest.TestCase):
    def test_rejects_nonfinite_or_wrong_shape(self):
        self.assertIsNone(MODULE.finite_head_map(np.array([1.0, 2.0]), "frontier"))
        self.assertIsNone(MODULE.finite_head_map(np.array([[np.nan]]), "frontier"))
        result = MODULE.finite_head_map(np.ones((2, 3)), "frontier")
        self.assertEqual(result.shape, (2, 3))

    def test_image_size_comparison_handles_numpy_values(self):
        self.assertTrue(MODULE.image_size_matches(np.array([960, 640]), 960, 640))
        self.assertFalse(MODULE.image_size_matches(np.array([640, 360]), 960, 640))


class MotionChunkTests(unittest.TestCase):
    def test_bearing_deg_absolute_quadrants(self):
        self.assertAlmostEqual(MODULE.bearing_deg([1.0, 0.0]), 0.0)
        self.assertAlmostEqual(MODULE.bearing_deg([0.0, 1.0]), 90.0)
        self.assertAlmostEqual(MODULE.bearing_deg([-1.0, 0.0]), 180.0)
        self.assertAlmostEqual(MODULE.bearing_deg([0.0, -1.0]), -90.0)
    def test_long_path_is_clipped_to_one_short_endpoint(self):
        endpoint = MODULE.short_endpoint([(0.0, 0.0), (1.0, 0.0), (5.0, 0.0)], 2.0)
        np.testing.assert_allclose(endpoint, [2.0, 0.0])

    def test_short_path_is_not_extended(self):
        endpoint = MODULE.short_endpoint([(0.0, 0.0), (1.5, 1.0)], 2.0)
        np.testing.assert_allclose(endpoint, [1.5, 1.0])

    def test_drive_segment_sends_two_point_short_line(self):
        class FakeCar:
            def __init__(self):
                self.position = np.array([0.0, 0.0, 1.0])
                self.state = {"pos": self.position.copy(), "vel": [0.0, 0.0, 0.0]}
                self.sensors = SimpleNamespace(poll=lambda: None)
                self.ai = SimpleNamespace()
                self.ai.set_mode = lambda mode: setattr(self, "mode", mode)
                self.ai.set_line = self.set_line
                self.control_calls = []

            def control(self, **kwargs):
                self.control_calls.append(kwargs)

            def set_line(self, line, cling):
                self.line = line
                self.cling = cling
                self.state["pos"] = np.array([line[-1]["pos"][0], line[-1]["pos"][1], 1.0])

        class FakeBng:
            def resume(self):
                pass

            class Control:
                @staticmethod
                def resume():
                    pass

            control = Control()

        car = FakeCar()
        with patch.object(MODULE.time, "sleep", return_value=None):
            moved = MODULE.drive_segment(
                car, FakeBng(), [(0.0, 0.0), (1.0, 0.0), (10.0, 0.0)],
                seconds=0.01, speed=5.0, chunk_metres=2.0)
        self.assertEqual(car.mode, "traffic")
        self.assertFalse(car.cling)
        self.assertEqual(len(car.line), 2)
        self.assertLessEqual(np.linalg.norm(np.asarray(car.line[-1]["pos"][:2])), 2.0 + 1e-6)
        self.assertGreater(moved, 0.0)

    def test_drive_segment_propagates_ai_failure(self):
        class FailingAI:
            def set_mode(self, mode):
                pass

            def set_line(self, line, cling):
                raise TimeoutError("fake AiLineSet timeout")

        car = SimpleNamespace(
            state={"pos": [0.0, 0.0, 1.0]},
            sensors=SimpleNamespace(poll=lambda: None),
            ai=FailingAI(),
            control=lambda **kwargs: None,
        )
        bng = SimpleNamespace(control=SimpleNamespace(resume=lambda: None))
        with self.assertRaises(TimeoutError):
            MODULE.drive_segment(car, bng, [(0.0, 0.0), (1.0, 0.0)], 0.1, 3.0, 2.0)

    def test_ai_mode_is_enabled_only_once(self):
        class FakeAI:
            def __init__(self):
                self.mode_calls = 0
                self.line_calls = 0

            def set_mode(self, mode):
                self.mode_calls += 1

            def set_line(self, line, cling):
                self.line_calls += 1

        class FakeCar:
            def __init__(self):
                self.state = {"pos": [0.0, 0.0, 1.0]}
                self.sensors = SimpleNamespace(poll=lambda: None)
                self.ai = FakeAI()

            def control(self, **kwargs):
                pass

        bng = SimpleNamespace(control=SimpleNamespace(resume=lambda: None))
        car = FakeCar()
        with patch.object(MODULE.time, "sleep", return_value=None):
            MODULE.drive_segment(car, bng, [(0.0, 0.0), (1.0, 0.0)], 0.01, 2.0, 2.0)
            MODULE.drive_segment(car, bng, [(0.0, 0.0), (1.0, 0.0)], 0.01, 2.0, 2.0)
        self.assertEqual(car.ai.mode_calls, 1)
        self.assertEqual(car.ai.line_calls, 2)

    def test_exploration_bearing_has_hysteresis(self):
        first = MODULE.smooth_exploration_bearing(None, np.radians(60.0))
        second = MODULE.smooth_exploration_bearing(first, np.radians(-60.0))
        self.assertAlmostEqual(np.degrees(first), 60.0, places=5)
        self.assertAlmostEqual(np.degrees(second), 45.0, places=5)

    def test_mask_bearing_uses_visible_centroid(self):
        mask = np.zeros((10, 20), dtype=bool)
        mask[2:5, 14:18] = True
        self.assertGreater(MODULE.mask_bearing(mask, 80.0), 0.0)
        self.assertIsNone(MODULE.mask_bearing(np.zeros((10, 20), dtype=bool), 80.0))

    def test_mask_lidar_diagnostics_counts_projected_points(self):
        cloud = np.array([[0.0, 0.0, 5.0], [3.0, 0.0, 5.0]], dtype=float)
        mask = np.zeros((100, 100), dtype=bool)
        mask[45:55, 45:55] = True
        K = np.array([[10.0, 0.0, 50.0], [0.0, 10.0, 50.0], [0.0, 0.0, 1.0]])
        result = MODULE.mask_lidar_diagnostics(cloud, mask, K, np.eye(3), [0, 0, 0])
        self.assertEqual(result, {"front": 2, "in_image": 2, "in_mask": 1})

    def test_localize_mask_lidar_returns_median_point(self):
        cloud = np.array([[0.0, 0.0, 5.0], [0.0, 0.0, 5.2]], dtype=float)
        mask = np.zeros((100, 100), dtype=bool)
        mask[45:55, 45:55] = True
        K = np.array([[10.0, 0.0, 50.0], [0.0, 10.0, 50.0], [0.0, 0.0, 1.0]])
        point, diag = MODULE.localize_mask_lidar(cloud, mask, K, np.eye(3), [0, 0, 0], 2)
        self.assertIsNotNone(point)
        self.assertEqual(diag["in_mask"], 2)
        self.assertTrue(np.isfinite(point).all())

    def test_image_height_name_is_not_reused_for_lidar_obstacle_height(self):
        source = (HERE / "local_wildos_demo.py").read_text(encoding="utf-8")
        self.assertIn("obstacle_height = near[:, 2] - ground", source)
        main_source = source.split("def main", 1)[1]
        self.assertNotIn("\n            height = near[:, 2] - ground", main_source)


if __name__ == "__main__":
    unittest.main()
