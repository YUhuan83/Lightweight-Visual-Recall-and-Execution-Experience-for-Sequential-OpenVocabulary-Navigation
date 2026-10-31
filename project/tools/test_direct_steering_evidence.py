"""Check frozen steering evidence without promoting failed motion to success."""
import ast
import hashlib
import json
from pathlib import Path
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT / 'test_artifacts/server_validation_20261012/direct_steering_evidence.zip'


class DirectSteeringEvidenceTests(unittest.TestCase):
    def data(self, name):
        with zipfile.ZipFile(ARCHIVE) as archive:
            return archive.read(name)

    def document(self, name):
        return json.loads(self.data(name))

    def test_closed_members_and_no_video(self):
        self.assertEqual(ARCHIVE.stat().st_size, 3328064)
        self.assertEqual(hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),
                         'd5ecfe949c0d29bb134ad81ee3a740ff668f6fd56ed245c47187075511872462')
        with zipfile.ZipFile(ARCHIVE) as archive:
            manifest = json.loads(archive.read('manifest.json'))
            self.assertEqual(len(manifest), 124)
            self.assertEqual(set(archive.namelist()), set(manifest) | {'manifest.json'})
            for name, expected in manifest.items():
                data = archive.read(name)
                self.assertEqual(len(data), expected['bytes'], name)
                self.assertEqual(hashlib.sha256(data).hexdigest(), expected['sha256'], name)
            self.assertFalse(any(n.endswith('.mp4') for n in archive.namelist()))

    def test_stationary_input_comparison_keeps_physical_actuation(self):
        report = self.document('steering_input_probe2/result.json')
        self.assertTrue(report['passed'])
        self.assertEqual(report['errors'], [])
        self.assertEqual(report['map_reloads'], 0)
        self.assertEqual(report['teleports'], 0)
        self.assertEqual([p['mode'] for p in report['phases']], ['sdk_pad']*2 + ['direct']*2)
        for phase in report['phases']:
            samples = phase['samples']; target = phase['target']
            if phase['mode'] == 'sdk_pad':
                self.assertGreater(abs(samples[0]['filtered_input']-target), .5)
            else:
                self.assertAlmostEqual(samples[0]['filtered_input'], target)
                self.assertLess(samples[0]['elapsed'], .02)
                self.assertEqual(samples[0]['filter'], 2)
                self.assertTrue(any(abs(h['state']-h['cmd'])>.1 for h in samples[0]['hydros']))
            for sample in samples:
                self.assertTrue(all(h['inRate']==1.25 and h['outRate']==1.25 for h in sample['hydros']))
        self.assertLess(report['after']['speed'], .05)
        self.assertTrue(report['paused'])

    def test_actual_new_rpc_still_passes_brake_held_delay_and_recovery(self):
        driver = self.document('direct_steering_freshness1/result.json')
        report = self.document('direct_steering_freshness1/execution/result.json')
        self.assertTrue(driver['passed'])
        self.assertTrue(report['passed'])
        self.assertEqual(driver['remaining_owned'], [])
        self.assertEqual(driver['sensor_exit'], 0)
        for name, rows in report['phases'].items():
            native = [r['data'] for r in rows if r['kind']=='native']
            self.assertTrue(native)
            self.assertTrue(all(n['steering_control_interface']=='beamng_direct_input_v1'
                                and n['sent']['throttle']==0. and n['sent']['brake']==1. for n in native))
            if name.startswith('delayed_'):
                self.assertTrue(any(not n['telemetry_ok'] for n in native))
                self.assertTrue(any(r['kind']=='consumer' and
                                    r['data']['reason']=='native_control_unavailable' for r in rows))

    def test_movement_failure_common_gap_and_independent_stop_are_preserved(self):
        prefix = 'direct_steering_short_curve1/'
        driver = self.document(prefix+'result.json')
        report = self.document(prefix+'execution/result.json')
        self.assertFalse(driver['passed'])
        self.assertFalse(report['passed'])
        self.assertIn('AssertionError: Native receipt stale', report['errors'])
        audit = self.document(prefix+'rpc_timing_audit.json')
        self.assertTrue(any(r['kind']=='QueueLuaCommandVE' and r['phase']=='recv_error'
                            and r['duration_sec']>2. for r in audit['slow_requests']))
        self.assertTrue(any(r['kind']=='SensorRequest' and r['phase']=='recv_end'
                            and r['duration_sec']>4. for r in audit['slow_requests']))
        samples = [json.loads(row) for row in self.data(prefix+'execution/native_samples.jsonl').splitlines()]
        self.assertTrue(any(n['actuator_ok'] is False and n['actual']['brake']==0.
                            and n['speed']>.5 for n in samples))
        stop = self.document(prefix+'independent_stop.json')
        self.assertTrue(stop['passed'])
        self.assertLess(stop['after']['speed'], .05)
        for prefix in ('direct_steering_curve1/', 'direct_steering_curve2/'):
            self.assertFalse(self.document(prefix+'execution/result.json')['passed'])

    def test_version_boundary_is_explicit_and_frozen_sources_match(self):
        report = self.document('direct_steering_repair/verified_sources.json')
        self.assertEqual(report['tests'], {'count':52, 'passed':True})
        for name, meta in report['sources'].items():
            if name == 'runtime_helper_before_doc_correction':
                data = self.data('direct_steering_repair/running_direct_vehicle_control.py')
            else:
                data = self.data('direct_steering_repair/sources/'+name)
            self.assertEqual(hashlib.sha256(data).hexdigest(), meta['sha256'], name)
        old = ast.parse(self.data('direct_steering_repair/running_direct_vehicle_control.py'))
        current = ast.parse(self.data('direct_steering_repair/sources/nav2_adapter/wildos_nav2_adapter/direct_vehicle_control.py'))
        old.body.pop(0); current.body.pop(0)  # Only the module docstring changed.
        self.assertEqual(ast.dump(old), ast.dump(current))
        final = self.document('direct_steering_repair/final_safe_state.json')
        self.assertTrue(final['passed'])
        self.assertEqual(final['production_status'], '')
        self.assertEqual(final['remaining_motion_clients'], [])
        self.assertEqual(final['snapshot']['fixedNodeCount'], 0)
        self.assertEqual(final['snapshot']['bodyCollisionNodeCount'], 467)
        self.assertEqual([w['collisionNodeCount'] for w in final['snapshot']['wheels']], [32]*4)
        self.assertFalse(final['snapshot']['cameras'])
        self.assertFalse(final['snapshot']['lidars'])

    def test_ideal_counterevidence_does_not_claim_field_success(self):
        report = self.document('rpp_closed_loop_replay1/result.json')
        self.assertFalse(report['passed'])  # Failed prediction is retained.
        log = self.data('rpp_closed_loop_replay1/result.log').decode()
        self.assertIn('REPLAY mode=0 reached=1', log)
        self.assertIn('REPLAY mode=1 reached=1', log)
        summary = self.document('summary.json')
        self.assertEqual(summary['full_search_acceptance'], 'PENDING')
        self.assertFalse(summary['new_commit'])
        self.assertTrue(summary['no_videos'])


if __name__ == '__main__':
    unittest.main()
