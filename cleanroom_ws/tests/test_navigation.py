import math
import unittest

from wildos_nav2_adapter.navigation import (
    State,
    Task,
    RecallCandidatePolicy,
    arrived,
    route_candidates,
)
from wildos_nav2_adapter.drive_math import steering_from_angular
from beamng_wildos_adapter.geometry import body_rotation, shifted_body_position
from visual_navigation.explorfm_triangulation.target_estimate import TargetEstimate
import numpy as np
from visual_navigation.explorfm_triangulation.localization_geometry import (
    nearest_component_depth,
)


class NearestInstanceTests(unittest.TestCase):
    def fixture(self):
        k = np.array([[100, 0, 100], [0, 100, 100], [0, 0, 1]], float)
        mask = np.zeros((200, 200), np.uint8)
        mask[90:95, 60:65] = 1
        mask[90:99, 140:149] = 1

        def returns(us, vs, depth):
            return [
                [(u - 100) * depth / 100, (v - 100) * depth / 100, depth]
                for u in us
                for v in vs
            ]

        near = returns(range(60, 65), range(90, 95), 10)
        far = returns(range(140, 149), range(90, 99), 20)
        rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
        return k, mask, near, far, rotation

    def test_small_near_region_beats_large_far_region(self):
        k, mask, near, far, rotation = self.fixture()
        result = nearest_component_depth(near + far, k, mask, rotation, np.zeros(3))
        self.assertEqual(result["position"][2], 10)
        self.assertEqual(result["selection_policy"], "nearest_observed_xy")
        self.assertEqual(len(result["candidate_distances_xy_m"]), 2)

    def test_distance_uses_observer_xy_not_camera_optical_depth(self):
        k, mask, near, far, rotation = self.fixture()
        result = nearest_component_depth(
            near + far, k, mask, rotation, np.array([0, 0, 30])
        )
        self.assertEqual(result["position"][2], 20)

    def test_ambiguous_component_is_not_confirmed_as_nearest(self):
        k, mask, near, far, rotation = self.fixture()
        # Two separate supported depths, at distinct pixels in the small region.
        near = [
            [(u - 100) * d / 100, (v - 100) * d / 100, d]
            for d, us in ((10, range(60, 62)), (13, range(62, 65)))
            for u in us
            for v in range(90, 95)
        ]
        result = nearest_component_depth(near + far, k, mask, rotation, np.zeros(3))
        self.assertEqual(result["position"][2], 20)


class NavigationTests(unittest.TestCase):
    def test_refinement_keeps_same_task(self):
        task = Task()
        task.begin("Green House")
        self.assertEqual(task.state, State.EXPLORING)
        self.assertTrue(task.refine((20, 0, 58)))
        self.assertTrue(task.refine((20.1, 0.02, 58.2)))
        self.assertEqual(task.state, State.APPROACHING_TARGET)
        self.assertEqual(task.query, "green house")
        task.begin("bus stop")
        self.assertIsNone(task.target)

    def test_arrival_uses_xy_strict_boundary_and_current_visual_evidence(self):
        evidence = dict(
            query="house",
            search_confirmed=True,
            object_position=[3, 0, 58],
            observation_stamp_ns=10_000_000_000,
        )
        self.assertFalse(arrived((0, 0), (3, 0, 58), evidence, "house", 10_000_000_000))
        self.assertTrue(
            arrived((0.01, 0), (3, 0, 58), evidence, "house", 10_000_000_000)
        )
        self.assertFalse(
            arrived((0.01, 0), (3, 0, 58), evidence, "bus", 10_000_000_000)
        )
        # This is an accepted, current-task object. Its observed point does
        # not expire merely because the car takes time to reach it.
        self.assertTrue(
            arrived((0.01, 0), (3, 0, 58), evidence, "house", 16_000_000_000)
        )

    def test_forward_candidates_remain_available_after_bad_topology_direction(self):
        candidates = route_candidates([(0, 0), (-10, 0)], (0, 0), 0, (25, 0, 58))
        self.assertGreater(candidates[0][0], 0)
        self.assertLessEqual(len(candidates), 10)

    def test_final_candidates_are_inside_arrival_disk(self):
        candidates = route_candidates([], (0, 0), 0, (5, 0, 58))
        self.assertTrue(candidates)
        self.assertTrue(all(math.hypot(x - 5, y) < 3 for x, y, _ in candidates))

    def test_terminal_pose_is_requested_before_six_meter_deadzone(self):
        candidates = route_candidates([], (0, 0), 0, (9, 0, 58))
        # A rear axle 2m from a wall may need to be parallel to it, not
        # pointing its 3.4m front into it. Offer that pose before reaching6m.
        self.assertEqual(candidates[0], (7, 0, math.pi / 2))
        self.assertTrue(all(math.hypot(x - 9, y) < 3 for x, y, _ in candidates[:8]))
        # At9m both prior probe families use gap-2=7m and coincide here.
        for old in ((7, 0, 0), (7, 2, 0), (7, -2, 0)):
            self.assertIn(old, candidates)
        self.assertLessEqual(len(candidates), 21)

    def test_recorded_approach_keeps_reachable_vehicle_heading_candidate(self):
        # The three target-bearing requests all failed on this recorded map.
        # Smac and RPP accepted this forward/right alternative at actual yaw.
        position = (735.6538579674551, -103.48907037016262)
        heading = -1.2486966154092018
        target = (751.0565794363943, -130.0871281595088, 58.43878929212809)
        candidates = route_candidates([], position, heading, target)
        reachable = (736.2891842023795, -111.7107709591659, heading)
        self.assertTrue(
            any(
                math.hypot(c[0] - reachable[0], c[1] - reachable[1]) < 1e-6
                and abs(math.remainder(c[2] - reachable[2], 2 * math.pi)) < 1e-6
                for c in candidates[:4]
            )
        )

    def test_approach_detour_does_not_discard_existing_route_candidates(self):
        points = [(0, 0), (40, 0)]
        original = route_candidates(points, (0, 0), 0)
        candidates = route_candidates(points, (0, 0), 0, (50, 5, 58))
        self.assertEqual(candidates[0], original[0])
        self.assertTrue(all(c in candidates for c in original))
        self.assertIn((8, -2, 0), candidates[:4])
        self.assertLessEqual(len(candidates), 13)

    def test_vehicle_forward_probe_does_not_move_away_from_target(self):
        candidates = route_candidates([], (0, 0), 0, (-30, 0, 58))
        self.assertTrue(all(c[0] < 0 for c in candidates))

    def test_sparse_route_keeps_reachable_prefixes(self):
        candidates = route_candidates([(0, 0), (12.65, 0)], (0, 0), 0)
        self.assertEqual([round(c[0], 2) for c in candidates], [12.65, 12, 10, 8, 6, 4])
        self.assertTrue(all(abs(c[1]) < 1e-9 and abs(c[2]) < 1e-9 for c in candidates))

    def test_projection_discards_traversed_prefix_and_follows_corner(self):
        candidates = route_candidates([(0, 0), (10, 0), (10, 10)], (6, 0), 0)
        self.assertEqual(candidates[0], (10, 10, math.pi / 2))
        self.assertIn((10, 0, 0.0), candidates)
        self.assertTrue(all(x >= 6 for x, _, _ in candidates))

    def test_short_endpoint_and_distinct_arrival_headings_survive(self):
        self.assertEqual(
            route_candidates([(0, 0), (4.25, 0)], (0, 0), 0), [(4.25, 0, 0), (4, 0, 0)]
        )
        candidates = route_candidates([], (0, 0), 0, (5, 0, 58))
        self.assertEqual(len(candidates), 8)

    def test_recall_policy_prefers_reachable_direction_and_distance(self):
        views = [
            dict(id="far", position=[30, 0, 52]),
            dict(id="near", position=[8, 0, 52]),
        ]
        self.assertEqual(RecallCandidatePolicy.rank(views, (0, 0), 0)[0]["id"], "near")

    def test_consensus_keeps_raw_xyz_and_rejects_outlier(self):
        estimate = TargetEstimate()
        self.assertIsNone(estimate.update((10, 0, 58), 1))
        self.assertIsNone(estimate.update((10.2, 0, 58.1), 2))
        self.assertIsNone(estimate.update((-20, 50, 70), 3))
        result = estimate.update((10.1, 0.1, 58), 4)
        self.assertEqual(result["inliers"], 3)
        self.assertGreater(result["position"][2], 57)
        estimate.clear()
        self.assertIsNone(estimate.update((50, 0, 52), 5))

    def test_confirmed_target_does_not_switch_after_observation_gap(self):
        estimate = TargetEstimate()
        original = np.array([750.7376709, -130.0871133, 58.2054939])
        other = np.array([746.4059449, -135.6274724, 54.4456482])
        for second in (1, 2, 3):
            result = estimate.update(original, second * 1_000_000_000)
        np.testing.assert_allclose(result["position"], original)
        # The real failure replaced the target after the prior samples aged
        # out. Repeated support elsewhere cannot redefine the same target.
        for second in (45, 46, 47):
            self.assertIsNone(estimate.update(other, second * 1_000_000_000))
        # A fresh observation of the original XY can change its measured Z:
        # roof and facade returns do not have to be at the same height.
        refined = original + [0.2, -0.1, -4.0]
        for second in (48, 49, 50):
            result = estimate.update(refined, second * 1_000_000_000)
        np.testing.assert_allclose(result["position"], refined)

    def test_query_clear_allows_a_different_target_location(self):
        estimate = TargetEstimate()
        for stamp in (1, 2, 3):
            estimate.update((10, 0, 58), stamp)
        estimate.clear()
        for stamp in (4, 5, 6):
            result = estimate.update((-30, 20, 54), stamp)
        np.testing.assert_allclose(result["position"], (-30, 20, 54))

    def test_steering_preserves_curvature_in_both_gears(self):
        forward = steering_from_angular(0.1, 1.2, 2.501, 0.54)
        reverse = steering_from_angular(-0.1, -1.2, 2.501, 0.54)
        self.assertAlmostEqual(forward, reverse)
        self.assertAlmostEqual(steering_from_angular(0, 0, 2.501, 0.54), 0)

    def test_rigid_body_offset_rotates_with_body(self):
        from types import SimpleNamespace

        rotation = body_rotation(
            SimpleNamespace(x=0, y=1, z=0), SimpleNamespace(x=0, y=0, z=1)
        )
        result = shifted_body_position(
            np.array([100, 200, 52]), rotation, (-2.172, 0.33, 0)
        )
        np.testing.assert_allclose(result, [99.67, 197.828, 52])
