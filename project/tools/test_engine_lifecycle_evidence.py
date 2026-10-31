"""Verify archived lifecycle and short execution evidence, not search success."""
import hashlib
import json
from pathlib import Path
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT / 'test_artifacts/server_validation_20261012/engine_lifecycle_execution_evidence.zip'


class EngineLifecycleEvidenceTests(unittest.TestCase):
    def document(self, name):
        with zipfile.ZipFile(ARCHIVE) as archive:
            return json.loads(archive.read(name))

    def test_archive_is_closed_and_contains_no_video(self):
        self.assertEqual(ARCHIVE.stat().st_size, 779244)
        self.assertEqual(hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),
                         '4779f4e82784a45e294c9cb3b1d44fe799e934c3839f1c39458c86ac121800cc')
        with zipfile.ZipFile(ARCHIVE) as archive:
            manifest = json.loads(archive.read('manifest.json'))
            self.assertEqual(len(manifest), 133)
            self.assertEqual(set(archive.namelist()), set(manifest) | {'manifest.json'})
            for name, meta in manifest.items():
                data = archive.read(name)
                self.assertEqual(len(data), meta['bytes'], name)
                self.assertEqual(hashlib.sha256(data).hexdigest(), meta['sha256'], name)
            self.assertFalse(any(n.endswith('.mp4') for n in archive.namelist()))

    def test_crash_and_recovery_are_not_claimed_as_memory_continuity(self):
        before = self.document('engine_recovery/before.json')
        self.assertTrue(before['engine_pid_missing'])
        self.assertFalse(before['sensor_owner_remaining'])
        self.assertFalse(before['map_memory_continuity'])
        crash = self.document('engine_recovery/crash_analysis.json')
        self.assertEqual(crash['exception_code'], 11)
        self.assertEqual(crash['instruction_module'][0]['offset'], '0x87c5')
        self.assertTrue(crash['instruction_module'][0]['name'].endswith('/libmimalloc.so.3'))
        self.assertFalse(crash['unwound'])
        recovered = self.document('engine_recovery/bootstrap_result.json')
        self.assertTrue(recovered['passed'])
        self.assertEqual(recovered['map_loads_preparation'], 1)
        self.assertFalse(recovered['memory_continuity'])
        self.assertFalse(recovered['physics_overrides'])

    def test_actual_sigterm_and_sigint_cleanup_preserve_the_vehicle(self):
        first = self.document('engine_recovery/sensor_lifecycle.json')
        self.assertIn('context is not valid', first['bridge_cleanup_error'])
        self.assertFalse(self.document('engine_recovery/sensor_lifecycle_cleanup_attempt1.json')['passed'])
        result = self.document('engine_recovery/lifecycle_verified2/result.json')
        self.assertTrue(result['passed'])
        self.assertEqual(result['errors'], [])
        self.assertEqual([p['signal'] for p in result['phases']], ['SIGTERM', 'SIGINT'])
        for phase in result['phases']:
            self.assertTrue(phase['passed'])
            self.assertEqual(phase['exit'], 0)
            self.assertTrue(phase['attachment']['stopped'])
            self.assertEqual(phase['after']['vehicleID'], result['before']['vehicleID'])
            self.assertEqual(phase['after']['position'], result['before']['position'])
            self.assertFalse(phase['after']['cameras'])
            self.assertFalse(phase['after']['lidars'])

    def test_concurrency_counterexample_is_retained_with_source_and_field_failure(self):
        with zipfile.ZipFile(ARCHIVE) as archive:
            log = archive.read('engine_recovery/poll_counterexample_before.log').decode()
        self.assertIn('concurrent poll() invocation', log)
        self.assertIn('FAILED (errors=1)', log)
        self.assertFalse(self.document('physical_execution_probe2/result.json')['passed'])
        first = self.document('physical_execution_probe2/sensor_attachment.json')
        self.assertEqual(first['error'], 'RuntimeError: concurrent poll() invocation')
        self.assertTrue(first['stopped'])
        self.assertEqual(self.document('physical_execution_probe2/execution/result.json')['plans'], [])

    def test_both_real_short_executions_pass_with_collisions_and_radius_preserved(self):
        for prefix in ('physical_execution_probe3', 'physical_execution_curve_probe1'):
            with self.subTest(prefix=prefix):
                driver = self.document(prefix + '/result.json')
                execution = self.document(prefix + '/execution/result.json')
                self.assertTrue(driver['passed'])
                self.assertTrue(execution['passed'])
                self.assertEqual(driver['errors'], [])
                self.assertEqual(driver['remaining_owned'], [])
                self.assertEqual(driver['sensor_exit'], 0)
                self.assertEqual(driver['map_reloads'], 0)
                self.assertEqual(driver['teleports'], 0)
                self.assertFalse(driver['vision_started'])
                self.assertFalse(execution['visual_arrival_validated'])
                self.assertGreater(execution['displacement_m'], .5)
                self.assertLessEqual(execution['endpoint_distance_m'], 1.)
                self.assertEqual(execution['terminal']['status'], 'route_done')
                self.assertEqual(execution['actual_parameters']['/wildos_graph_path_bridge']['minimum_turning_radius'], 8.)
                for state in (execution['physics_before'], execution['physics_after']):
                    self.assertEqual(state['fixedNodeCount'], 0)
                    self.assertEqual(state['bodyCollisionNodeCount'], 467)
                    self.assertEqual([w['collisionNodeCount'] for w in state['wheels']], [32]*4)
                self.assertLess(execution['physics_after']['speed'], .05)
        curve = self.document('physical_execution_curve_probe1/execution/result.json')
        self.assertNotEqual(curve['selected']['relative_goal'][1], 0.)
        self.assertNotEqual(curve['selected']['relative_goal'][2], 0.)

    def test_deployment_matches_archived_sources_and_final_state_has_no_orphan(self):
        verified = self.document('engine_recovery/verified_after.json')
        self.assertEqual(verified['production_status'], '')
        with zipfile.ZipFile(ARCHIVE) as archive:
            for name, meta in verified['sources'].items():
                data = archive.read('engine_recovery/frozen/' + name)
                self.assertEqual(hashlib.sha256(data).hexdigest(), meta['sha256'], name)
        final = self.document('engine_recovery/final_safe_state.json')
        self.assertTrue(final['passed'])
        self.assertEqual(final['remaining_vehicle_clients'], [])
        self.assertTrue(final['simulation_paused'])
        self.assertTrue(final['error_log_stable'])
        self.assertLess(final['snapshot']['speed'], .05)
        self.assertFalse(final['snapshot']['cameras'])
        self.assertFalse(final['snapshot']['lidars'])

    def test_tracking_frame_correction_and_remaining_limits_are_explicit(self):
        audit = self.document('pilot5_tracking_body_audit.json')
        self.assertIn('invalid', audit['supersedes'])
        self.assertIn('not an exact', audit['scope'])
        self.assertEqual(audit['summary']['samples'], 90)
        self.assertEqual(audit['summary']['saturated_samples'], 90)
        self.assertTrue(all(row['frame'] == 'spot1/nav_base_link' for row in audit['rows']))
        verified = self.document('engine_recovery/verified_after.json')
        self.assertTrue(verified['no_new_complete_search'])
        self.assertIn('UNRESOLVED', verified['engine_crash_root_cause'])
        self.assertIn('UNRESOLVED', verified['tracking_and_cusp'])
        self.assertEqual(verified['full_search_acceptance'], 'PENDING')


if __name__ == '__main__':
    unittest.main()
