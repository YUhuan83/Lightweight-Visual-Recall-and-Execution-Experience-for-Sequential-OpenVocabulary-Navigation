"""Regression checks for recorded ownership counterexamples; no field acceptance."""
import unittest
from offline_extension_ownership_audit import audit


class ExtensionOwnershipAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = audit()

    def test_existing_short_join_fix_matches_swept_endpoint_model(self):
        counts = self.report['short_join_sweep']['counts']
        self.assertEqual(sum(counts.values()), 343)
        self.assertEqual(counts['accepted_reached'], 220)
        self.assertEqual(counts['rejected_failed'], 123)
        self.assertEqual(counts['accepted_failed'], 0)
        self.assertEqual(counts['rejected_reached'], 0)

    def test_extension_timeout_preserves_and_processes_success(self):
        case = self.report['extension_timeout_success']
        self.assertEqual(case['after_failure']['epoch'], case['old_epoch'])
        self.assertTrue(case['after_failure']['original_follow_handle_retained'])
        result = case['after_original_result']
        self.assertFalse(result['following'])
        self.assertFalse(result['original_follow_handle_retained'])
        self.assertEqual(result['queued_sections'], 0)
        self.assertEqual(result['start_segment_calls'], 1)

    def test_original_abort_is_processed_after_extension_timeout(self):
        result = self.report['extension_timeout_abort']['after_original_result']
        self.assertFalse(result['following'])
        self.assertEqual(result['status']['status'], 'follow_failed')

    def test_new_graph_route_waits_for_original_action_completion(self):
        result = self.report['extension_timeout_new_route']['after_original_result']
        self.assertFalse(result['following'])
        self.assertFalse(result['extending_path'])
        self.assertFalse(result['pending'])
        self.assertEqual(result['start_segment_calls'], 1)
        self.assertEqual(result['follow_goals'], 0)

    def test_non_timeout_extension_failure_preserves_original_result(self):
        case = self.report['extension_non_timeout_control']
        self.assertEqual(case['after_failure']['epoch'], case['old_epoch'])
        result = case['after_original_result']
        self.assertFalse(result['following'])
        self.assertEqual(result['queued_sections'], 0)
        self.assertEqual(result['start_segment_calls'], 1)

    def test_foreground_timeout_still_cancels_late_planner_acceptance(self):
        self.assertGreaterEqual(self.report['foreground_timeout_control']
                                ['after_original_result']['old_planner_cancel_calls'], 1)

    def test_timeout_ownership_is_preserved_in_ideal_point_callbacks(self):
        case = self.report['ideal_point_timeout_control']
        self.assertEqual(case['carrier_model'], 'ideal_point')
        self.assertEqual(case['after_failure']['epoch'], case['old_epoch'])
        result = case['after_original_result']
        self.assertFalse(result['following'])
        self.assertEqual(result['queued_sections'], 0)
        self.assertEqual(result['start_segment_calls'], 1)


if __name__ == '__main__':
    unittest.main()
