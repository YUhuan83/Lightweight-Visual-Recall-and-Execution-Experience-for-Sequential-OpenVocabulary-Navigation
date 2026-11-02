import unittest

from scripts.trial_audit import independent_arrival_audit, motion_metrics


class TrialAuditTests(unittest.TestCase):
    def setUp(self):
        self.start = 10_000_000_000
        self.now = self.start + 1_000_000_000
        self.evidence = {
            "query": "Green House",
            "frame": "spot1/odom",
            "search_confirmed": True,
            "object_position": [2.9, 0.0, 58.0],
            "observation_stamp_ns": self.now,
        }

    def audit(self, vehicle_x, query="green house", evidence=None):
        return independent_arrival_audit(
            (vehicle_x, 0.0),
            self.now,
            (0.0, 0.0, 52.0),
            self.now,
            self.evidence if evidence is None else evidence,
            query,
            self.start,
            self.now,
            self.now,
        )

    def test_accepted_current_task_evidence_survives_approach_time(self):
        result = independent_arrival_audit(
            (0.0, 0.0),
            self.now + 120_000_000_000,
            (2.9, 0.0, 58.0),
            self.now,
            self.evidence,
            "green house",
            self.start,
            self.now + 120_000_000_000,
            self.now + 1_000_000_000,
        )
        self.assertTrue(result["verified"])
        self.assertEqual(result["age_s"], 120.0)
        self.assertEqual(result["receipt_age_s"], 1.0)
        # Replaying the same old message for the first time now is invalid.
        delayed = independent_arrival_audit(
            (0.0, 0.0),
            self.now + 120_000_000_000,
            (2.9, 0.0, 58.0),
            self.now,
            self.evidence,
            "green house",
            self.start,
            self.now + 120_000_000_000,
            self.now + 120_000_000_000,
        )
        self.assertFalse(delayed["verified"])

    def test_arrival_uses_strict_xy_three_meter_boundary(self):
        self.assertFalse(self.audit(3.0)["verified"])
        self.assertTrue(self.audit(2.999)["verified"])

    def test_arrival_uses_current_query_evidence(self):
        self.assertTrue(self.audit(2.0)["verified"])
        self.assertFalse(self.audit(2.0, query="bus stop")["verified"])
        pre_query = dict(self.evidence, observation_stamp_ns=self.start - 1)
        self.assertFalse(self.audit(2.0, evidence=pre_query)["verified"])
        stale = dict(self.evidence, observation_stamp_ns=self.now - 5_000_000_001)
        self.assertFalse(self.audit(2.0, evidence=stale)["verified"])

    def test_unconfirmed_or_wrong_frame_evidence_is_rejected(self):
        unconfirmed = dict(self.evidence, search_confirmed=False)
        self.assertFalse(self.audit(2.0, evidence=unconfirmed)["verified"])
        wrong_frame = dict(self.evidence, frame="map")
        self.assertFalse(self.audit(2.0, evidence=wrong_frame)["verified"])
        wrong_object = dict(self.evidence, object_position=[4.0, 0.0, 58.0])
        self.assertFalse(self.audit(2.0, evidence=wrong_object)["verified"])
        target_boundary = dict(self.evidence, object_position=[3.0, 0.0, 58.0])
        self.assertFalse(self.audit(0.0, evidence=target_boundary)["verified"])
        vehicle_boundary = dict(self.evidence, object_position=[-0.1, 0.0, 58.0])
        self.assertFalse(self.audit(2.9, evidence=vehicle_boundary)["verified"])

    def test_motion_metrics_clip_at_exploration_horizon(self):
        samples = [
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (1.0, 1.0, 0.0, 1.0, 0.0),
            (2.0, 1.0, 0.0, 2.0, 0.0),
            (3.0, 1.0, 0.0, 3.0, 0.0),
        ]
        result = motion_metrics(samples, 2.5)
        self.assertEqual(result["observed_s"], 2.5)
        self.assertAlmostEqual(result["distance_m"], 2.5)
        self.assertAlmostEqual(result["net_displacement_m"], 2.5)

    def test_motion_metrics_excludes_unobserved_gaps_from_stop_fraction(self):
        samples = [
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (0.5, 1.0, 0.0, 0.0, 0.0),
            (2.0, 1.0, 0.0, 1.5, 0.0),
            (2.5, 0.0, 0.0, 2.0, 0.0),
        ]
        result = motion_metrics(samples, 3.0)
        self.assertEqual(result["odometry_gaps_over_1s"], 1)
        self.assertAlmostEqual(result["gap_s"], 1.5)
        self.assertAlmostEqual(result["observed_s"], 1.0)
        self.assertAlmostEqual(result["near_stop_fraction"], 0.5)


if __name__ == "__main__":
    unittest.main()
