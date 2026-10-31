"""Validate claims against the recorded real SBR handoff, not test doubles."""
import hashlib
import json
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'test_artifacts/server_validation_20261010'
COMPONENT = EVIDENCE / 'cusp_stopped_component'


class PhysicalCuspEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = json.loads((COMPONENT / 'cusp_stopped_component1/result.json').read_text())
        cls.witness = [json.loads(line) for line in
                       (COMPONENT / 'cusp_stopped_component_witness.jsonl').read_text().splitlines()]
        cls.stop = json.loads((EVIDENCE / 'cusp_stopped_component_stack_stop.json').read_text())

    def test_archival_and_owned_process_cleanup(self):
        archive = EVIDENCE / 'cusp_stopped_component_evidence.zip'
        self.assertEqual(hashlib.sha256(archive.read_bytes()).hexdigest(), self.stop['archive_sha256'])
        self.assertEqual(self.stop['remaining'], [])
        self.assertTrue(self.stop['pause_acknowledged'])

    def test_handoff_has_brake_commands_and_measured_stopped_pose(self):
        handoffs = [e for e in self.witness if e['kind'] == 'status'
                    and e['status']['status'] == 'maneuver_done']
        self.assertTrue(handoffs)
        for event in handoffs:
            actual, plan = event['actual'], event['selected']
            cusp_indices = [end for _, end in plan['sections'][:-1]]
            index = min(cusp_indices, key=lambda i: math.dist(plan['points'][i], actual['position']))
            self.assertLessEqual(math.dist(plan['points'][index], actual['position']), .2)
            self.assertLess(abs(math.remainder(plan['headings'][index] - actual['heading'], 2 * math.pi)), .35)
            self.assertLessEqual(abs(actual['linear_mps']), .05)
            self.assertLessEqual(abs(actual['angular_radps']), .05)
            self.assertLess((event['receipt_ns'] - actual['stamp_ns']) / 1e9, .5)
            braking = [e for e in self.witness if e['kind'] == 'cmd_vel'
                       and 0 <= event['receipt_ns'] - e['receipt_ns'] < 300000000]
            self.assertGreaterEqual(len(braking), 2)
            self.assertTrue(all(e['linear_mps'] == e['angular_radps'] == 0 for e in braking))

    def test_commands_respect_turning_radius_and_route_completes(self):
        self.assertTrue(self.result['passed'], self.result['errors'])
        commands = [e for e in self.witness if e['kind'] == 'cmd_vel']
        self.assertTrue(any(e['linear_mps'] < 0 for e in commands))
        self.assertTrue(any(e['linear_mps'] > 0 for e in commands))
        for e in commands:
            self.assertLessEqual(abs(e['angular_radps']), abs(e['linear_mps']) / 8. + 1e-9)
        states = {e['status'] for e in self.result['events'] if e.get('kind') == 'plan_status'}
        self.assertIn('route_done', states)
        self.assertFalse(states & {'follow_failed', 'replanning_after_maneuver'})

    def test_real_body_and_tires_remain_enabled_without_target_claim(self):
        for physics in (self.result['physics_before'], self.result['physics_after'],
                        self.stop['physics_after_stop']):
            self.assertEqual(physics['fixedNodeCount'], 0)
            self.assertGreater(physics['bodyCollisionNodeCount'], 0)
            self.assertFalse(physics['configKinematic'])
            self.assertFalse(physics['configBodyDisabled'])
            self.assertEqual(len(physics['wheels']), 4)
            for wheel in physics['wheels']:
                self.assertEqual(wheel['fixedNodeCount'], 0)
                self.assertGreater(wheel['collisionNodeCount'], 0)
        self.assertLess(self.result['physics_after']['speed'], .05)
        self.assertFalse(self.result['terminal']['requires_visual_confirmation'])


if __name__ == '__main__':
    unittest.main()
