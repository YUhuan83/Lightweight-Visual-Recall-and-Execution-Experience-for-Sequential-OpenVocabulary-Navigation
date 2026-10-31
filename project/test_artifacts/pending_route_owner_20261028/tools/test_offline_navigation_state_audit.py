"""Transaction regression and remaining diagnostic defects; no field acceptance."""
import unittest
from offline_navigation_state_audit import audit


class NavigationStateAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = audit()

    def assert_healthy_trace(self, trace):
        self.assertEqual(trace[-1]['elapsed_sec'], 60)
        for step in trace:
            self.assertIsNone(step['input_issue'])
            self.assertIsNone(step['local_issue'])
            self.assertIsNone(step['transport_issue'])
            self.assertEqual(step['odom_stamp_ns'], (100+step['elapsed_sec'])*10**9)
            self.assertEqual(step['map_stamp_ns'], step['odom_stamp_ns'])

    def test_missing_acceptance_fails_closed_in_both_profiles(self):
        for case in self.report['follow_acceptance']:
            self.assert_healthy_trace(case['trace'])
            self.assertFalse(case['reproduced'])
            self.assertEqual(case['before_late_acceptance']['status']['reason'],'follow_acceptance_timeout')
            self.assertFalse(case['after_late_acceptance']['following'])

    def test_replacement_gap_retires_motion_without_resuming_old_action(self):
        case = self.report['replacement_acceptance']
        self.assert_healthy_trace(case['trace'])
        self.assertFalse(case['reproduced'])
        self.assertEqual(case['before_late_acceptance']['status']['reason'],'follow_replacement_timeout')
        self.assertFalse(case['after_late_acceptance']['deferred_result'])
        self.assertFalse(case['after_late_acceptance']['replacement_pending'])

    def test_missing_result_retires_action_without_visual_success(self):
        case = self.report['follow_result_response']
        self.assert_healthy_trace(case['trace'])
        self.assertFalse(case['reproduced'])
        self.assertEqual(case['before_late_result']['status']['reason'],'follow_result_timeout')
        self.assertFalse(case['after_late_result']['following'])
        self.assertEqual(case['after_late_result']['queued_sections'], 0)

    def test_duplicate_terminal_cannot_clear_new_unaccepted_motion(self):
        case = self.report['duplicate_follow_result']
        self.assertFalse(case['reproduced'])
        self.assertTrue(case['after_duplicate']['following'])
        self.assertTrue(case['new_acceptance_still_pending'])
        self.assertEqual(case['before_duplicate']['active_serial'],
                         case['after_duplicate']['active_serial'])

    def test_reset_response_gap_uses_bounded_retry_and_drains_late_responses(self):
        case = self.report['graph_reset_response']
        self.assert_healthy_trace(case['trace'])
        self.assertFalse(case['reproduced'])
        self.assertEqual(case['service_calls'], 3)
        self.assertEqual(case['before_late_response']['status']['status'],'graph_reset_failed')
        self.assertTrue(case['after_late_first_response']['query_reset_pending'])
        self.assertFalse(case['after_late_response']['query_reset_pending'])

    def test_controls_and_source_freeze_pass(self):
        self.assertTrue(self.report['normal_action']['passed'])
        self.assertTrue(self.report['rotated_map']['passed'])
        self.assertTrue(self.report['production_unchanged'])
        self.assertGreaterEqual(len(self.report['sources']), 100)

    def test_existing_handoff_budget_and_candidate_owner_defects_remain(self):
        case = self.report['known_callback_contracts']
        last = case['repeated_handoff']['observations'][-1]
        self.assertEqual(last['start_segment_calls'], 8)
        self.assertEqual(last['recovery_attempts'], 0)
        self.assertEqual(case['initial_infeasible_candidate']['next_candidate_calls'], 0)

    def test_supplied_alternative_can_dispatch_but_first_failure_does_not_try_it(self):
        case = self.report['candidate_fallback']
        self.assertTrue(case['reproduced'])
        self.assertEqual(case['first_rejection']['status']['status'], 'follow_failed')
        self.assertEqual(case['alternative_status']['status'], 'planned')

    def test_path_curvature_and_terrain_gaps_remain_distinct(self):
        self.assertEqual(self.report['returned_path']['offline_free_map_rejection'], 'excess_curvature')
        self.assertIsNone(self.report['terrain_corridor']['clear_obstacle_map_rejection'])
        self.assertEqual(self.report['terrain_corridor']['marked_obstacle_rejection'], 'collision_cell')

    def test_lookahead_changes_prediction_on_same_geometric_cases(self):
        case = self.report['lookahead']
        self.assertEqual(case['samples'], 45)
        self.assertGreater(case['differing_classifications'], 0)


if __name__ == '__main__':
    unittest.main()
