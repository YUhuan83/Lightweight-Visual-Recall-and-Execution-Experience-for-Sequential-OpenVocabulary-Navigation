import json
import math
from pathlib import Path
import unittest

from wildos_nav2_adapter.arrival_criteria import (
    MASK_CONFIRMATION_MAX_SKEW_NS,
    OBJECT_SURFACE_ARRIVAL_DISTANCE_M,
    PASSIVE_SEMANTIC_MIN_ASSOCIATION_DISTANCE_M,
    STANDOFF_ARRIVAL_TOLERANCE_M,
    arrival_status_matches,
    effective_object_arrival_distance,
    mask_matches_confirmation,
    matched_arrival_evidence,
    memory_region_metrics,
    planar_distance,
)


class ArrivalCriteriaTests(unittest.TestCase):
    def setUp(self):
        self.confirmation = {
            "query": "gas station",
            "observation_stamp_ns": 100_000_000_000,
            "object_position": [10.0, 4.0, 2.0],
            "radius": 0.5,
        }

    def arrival_status(self, **changes):
        status = {
            "status": "object_found",
            "query": "gas station",
            "confirmation": dict(self.confirmation),
            "standoff_distance_m": 0.5,
            "standoff_tolerance_m": STANDOFF_ARRIVAL_TOLERANCE_M,
            "object_distance_m": 4.0,
            "object_arrival_tolerance_m": OBJECT_SURFACE_ARRIVAL_DISTANCE_M,
        }
        status.update(changes)
        return status

    def test_arrival_requires_exact_object_found_confirmation_and_standoff(self):
        self.assertTrue(arrival_status_matches(
            self.arrival_status(), "gas station", self.confirmation))
        for changes in (
            {"status": "confirmation_outside_memory_region"},
            {"query": "house"},
            {"standoff_distance_m": 1.51},
            {"standoff_tolerance_m": 1.0},
            {"confirmation": {"query": "gas station", "observation_stamp_ns": 99}},
            {"standoff_distance_m": math.inf},
            {"object_distance_m": 4.51},
            {"object_distance_m": math.inf},
            {"object_arrival_tolerance_m": 3.0},
        ):
            with self.subTest(changes=changes):
                self.assertFalse(arrival_status_matches(
                    self.arrival_status(**changes), "gas station", self.confirmation))

    def test_arrival_without_measured_standoff_is_not_accepted(self):
        status = self.arrival_status()
        del status["standoff_distance_m"]
        self.assertFalse(arrival_status_matches(status, "gas station", self.confirmation))

    def test_radius_arrival_uses_confirmed_object_not_standoff(self):
        status=self.arrival_status(arrival_basis='confirmed_object_radius',
            vehicle_position=[5.,4.],object_distance_m=5.,
            object_arrival_tolerance_m=5.,standoff_distance_m=None)
        self.assertTrue(arrival_status_matches(status,'gas station',self.confirmation))
        for changes in ({'vehicle_position':[4.99,4.],'object_distance_m':5.01},
                        {'object_arrival_tolerance_m':5.1},
                        {'object_distance_m':4.},
                        {'vehicle_position':None},
                        {'confirmation':dict(self.confirmation,observation_stamp_ns=99)}):
            with self.subTest(changes=changes):
                self.assertFalse(arrival_status_matches(dict(status,**changes),
                    'gas station',self.confirmation))

    def test_bounded_history_accepts_exact_observation_after_latest_slot_changes(self):
        c = dict(self.confirmation, received_at=10., search_confirmed=True)
        newer = dict(c, observation_stamp_ns=101_000_000_000)
        item = dict(status=self.arrival_status(), received_at=10.)
        m = dict(query='gas station', observation_stamp_ns=100_000_000_000,
                 at=10., count=2, pixels=50)
        result = matched_arrival_evidence([item], [c, newer], [m], 'gas station',
                                         99_000_000_000, 102_000_000_000, 12., 2)
        self.assertIsNotNone(result)
        self.assertEqual(result['confirmation']['observation_stamp_ns'], c['observation_stamp_ns'])
        for changes in (dict(query='house'), dict(pixels=0),
                        dict(observation_stamp_ns=101_000_000_000), dict(at=6.)):
            self.assertIsNone(matched_arrival_evidence(
                [item], [c], [dict(m, **changes)], 'gas station',
                99_000_000_000, 102_000_000_000, 12., 2))
        self.assertIsNone(matched_arrival_evidence(
            [item], [c], [m], 'gas station', 101_000_000_000, 102_000_000_000, 12., 2))
        self.assertIsNone(matched_arrival_evidence(
            [item], [c], [m], 'gas station', 99_000_000_000, 106_000_000_000, 12., 2))

    def test_arrival_without_measured_object_distance_is_not_accepted(self):
        status = self.arrival_status()
        del status["object_distance_m"]
        self.assertFalse(arrival_status_matches(status, "gas station", self.confirmation))

    def test_clamped_standoff_tolerance_tracks_the_reachable_safe_pose(self):
        confirmation = dict(self.confirmation, standoff_clamped=True,
                            standoff_object_distance_m=5.2)
        tolerance = effective_object_arrival_distance(
            OBJECT_SURFACE_ARRIVAL_DISTANCE_M, True, 5.2)
        self.assertAlmostEqual(tolerance, 5.8)
        status = self.arrival_status(
            confirmation=confirmation,
            standoff_distance_m=0.5,
            object_distance_m=5.6,
            object_arrival_tolerance_m=tolerance)
        self.assertTrue(arrival_status_matches(status, "gas station", confirmation))

        for changes in (
            {"object_distance_m": 6.53},
            {"standoff_distance_m": 0.61},
            {"object_arrival_tolerance_m": 6.1},
        ):
            with self.subTest(changes=changes):
                self.assertFalse(arrival_status_matches(
                    dict(status, **changes), "gas station", confirmation))

    def test_unclamped_or_missing_standoff_metadata_keeps_strict_object_radius(self):
        self.assertEqual(effective_object_arrival_distance(4.5, False, 5.2), 4.5)
        self.assertEqual(effective_object_arrival_distance(4.5, True, None), 4.5)
        self.assertEqual(effective_object_arrival_distance(4.5, True, 9.0), 6.0)

    def test_mask_and_confirmation_must_be_from_the_same_observation_window(self):
        self.assertTrue(mask_matches_confirmation(
            "gas station", self.confirmation["observation_stamp_ns"] + 400_000_000,
            self.confirmation))
        self.assertFalse(mask_matches_confirmation(
            "gas station", self.confirmation["observation_stamp_ns"] + 600_000_000,
            self.confirmation))
        self.assertFalse(mask_matches_confirmation(
            "house", self.confirmation["observation_stamp_ns"], self.confirmation))
        self.assertEqual(MASK_CONFIRMATION_MAX_SKEW_NS, 500_000_000)

    def test_memory_region_metrics_are_finite_and_use_generic_radius_tolerance(self):
        view = {"object_position": [10.0, 0.0], "object_radius": 1.0}
        gap, tolerance = memory_region_metrics([14.0, 0.0], 0.5, view)
        self.assertEqual(gap, 4.0)
        self.assertEqual(tolerance, 4.5)
        passive = dict(view, evidence="passive_semantic_landmarks")
        self.assertEqual(memory_region_metrics([17.0, 0.0], 0.5, passive),
                         (7.0, PASSIVE_SEMANTIC_MIN_ASSOCIATION_DISTANCE_M))
        station_view = {
            "object_position": [719.4112548828125, -19.88218116760254],
            "object_radius": 0.6200116276741028,
            "evidence": "passive_semantic_landmarks",
        }
        station_observation = [726.3143043073594, -25.24847588707724]
        station_gap, station_tolerance = memory_region_metrics(
            station_observation, 0.5, station_view)
        self.assertAlmostEqual(station_gap, 8.7435, places=3)
        self.assertGreaterEqual(station_tolerance, station_gap)
        too_far_gap, too_far_tolerance = memory_region_metrics(
            [730.0, -25.24847588707724], 0.5, station_view)
        self.assertGreater(too_far_gap, too_far_tolerance)
        self.assertEqual(memory_region_metrics(
            [17.0, 0.0], 0.5, view), (7.0, 4.5))
        self.assertTrue(math.isinf(planar_distance([0.0], [0.0, 0.0])))

    def test_saved_house_station_run_is_rejected_despite_its_legacy_pass_flag(self):
        root = Path(__file__).resolve().parents[2]
        evidence = (root / "test_artifacts/server_validation_20261004"
                    / "unseeded_house_station_2_generic_20261004")
        report_path = evidence / "full_test.json"
        events_path = evidence / "events.jsonl"
        if not report_path.exists() or not events_path.exists():
            self.skipTest("saved failed-pass fixture is unavailable")
        report = json.loads(report_path.read_text(encoding="utf-8"))
        events = [json.loads(line) for line in events_path.read_text(encoding="utf-8").splitlines()]
        self.assertTrue(report["passed"], "fixture must retain the original, flawed report")
        accepted = {
            event.get("query") for event in events
            if event.get("kind") == "plan_status" and event.get("status") == "object_found"
        }
        self.assertNotIn("house", accepted)
        self.assertNotIn("gas station", accepted)
        self.assertTrue(any(event.get("status") == "confirmation_outside_memory_region"
                            for event in events if event.get("kind") == "plan_status"))


if __name__ == "__main__":
    unittest.main()
