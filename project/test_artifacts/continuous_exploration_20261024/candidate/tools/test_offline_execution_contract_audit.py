"""Validate diagnostic reproducibility; passing these does not repair production."""
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest

from offline_execution_contract_audit import endpoint_replay, ROOT


class ExecutionAuditTests(unittest.TestCase):
    def test_both_straight_directions_reach(self):
        for direction in (-1, 1):
            r = endpoint_replay((0., 0.), 0., (direction*1.7, 0.), 0., direction=direction)
            self.assertTrue(r['reached'])
            self.assertFalse(r['direction_guard'])
            self.assertLessEqual(r['endpoint_distance_m'], .2)

    def test_stopped_endpoint_is_checked_before_direction_guard(self):
        for direction in (-1, 1):
            r = endpoint_replay((0., 0.), 0., (-direction*.1, 0.), 0., direction=direction)
            self.assertTrue(r['reached'])
            self.assertEqual(r['traveled_m'], 0.)

    def test_side_offset_beyond_stop_disk_triggers_same_direction_guard(self):
        for direction in (-1, 1):
            r = endpoint_replay((0., 0.), 0., (-direction*.01, -.205), 0., direction=direction)
            self.assertFalse(r['reached'])
            self.assertTrue(r['direction_guard'])

    def test_recorded_failure_is_stable_to_integration_step_and_rigid_frame_transform(self):
        case = json.loads((ROOT/'nav2_adapter/test/fixtures/physical_short_cusp_join_20261012.json').read_text())
        sample = case['observations'][0]
        baseline = endpoint_replay(sample['position'], sample['heading'], case['points'][-1],
                                   case['headings'][-1], direction=-1)
        angle = .8
        def move(p):
            # Remove the large world offset before rotation.
            x, y = p[0]-700., p[1]+20.
            return x*math.cos(angle)-y*math.sin(angle), x*math.sin(angle)+y*math.cos(angle)
        for step in (.005, .001):
            r = endpoint_replay(move(sample['position']), sample['heading']+angle,
                move(case['points'][-1]), case['headings'][-1]+angle, direction=-1, step=step)
            self.assertTrue(r['direction_guard'])
            self.assertFalse(r['reached'])
            self.assertAlmostEqual(r['minimum_distance_m'], baseline['minimum_distance_m'], delta=1e-4)

    def test_nominal_start_remains_a_reachable_control(self):
        case = json.loads((ROOT/'nav2_adapter/test/fixtures/physical_short_cusp_join_20261012.json').read_text())
        r = endpoint_replay(case['points'][0], case['headings'][0], case['points'][-1],
                            case['headings'][-1], direction=-1)
        self.assertTrue(r['reached'])

    def test_report_reproduces_known_violations_in_isolated_process(self):
        out = ROOT/'test_artifacts/offline_execution_audit_20261013'
        with (out/'audit_console.json').open('w', encoding='utf8') as stream:
            result = subprocess.run([sys.executable, str(ROOT/'tools/offline_execution_contract_audit.py')],
                                    cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, timeout=30)
        self.assertEqual(result.returncode, 0)
        report = json.loads((out/'audit.json').read_text())
        for sample in report['short_cusp']['samples']:
            self.assertTrue(sample['previous_cusp_disk_accepts'])
            self.assertTrue(sample['legacy_join_accepts'])
            self.assertFalse(sample['current_join_accepts'])
            self.assertTrue(sample['zero_delay_bicycle']['direction_guard'])
            self.assertTrue(sample['full_3d_offset_replay']['direction_guard'])
        callback = report['callback_contracts']
        last = callback['repeated_handoff']['observations'][-1]
        self.assertEqual(last['start_segment_calls'], 8)
        self.assertEqual(last['recovery_attempts'], 0)
        self.assertIsNone(last['blocked_endpoint'])
        initial = callback['initial_infeasible_candidate']
        self.assertEqual(initial['status']['status'], 'follow_failed')
        self.assertEqual(initial['next_candidate_calls'], 0)
        self.assertEqual(initial['follow_goals'], 0)
        self.assertEqual(report['returned_path']['offline_free_map_rejection'], 'excess_curvature')
        corridor = report['graph_corridor']
        self.assertAlmostEqual(corridor['maximum_distance_from_graph_edge_m'], 8.)
        self.assertIsNone(corridor['clear_obstacle_map_rejection'])
        self.assertEqual(corridor['marked_obstacle_rejection'], 'collision_cell')


if __name__ == '__main__':
    unittest.main()
