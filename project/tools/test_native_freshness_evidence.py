"""Check archived component evidence without claiming a search succeeded."""
import hashlib
import json
from pathlib import Path
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT / 'test_artifacts/server_validation_20261012/freshness_interface_evidence.zip'


class NativeFreshnessEvidenceTests(unittest.TestCase):
    def document(self, name):
        with zipfile.ZipFile(ARCHIVE) as archive:
            return json.loads(archive.read(name))

    def test_every_closed_member_matches_and_no_video_exists(self):
        self.assertEqual(ARCHIVE.stat().st_size, 1853406)
        self.assertEqual(hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),
                         '33031426f58eb75b1cd5bd7024411111b03dea5a1722d339bd8bb4a126b06ddb')
        with zipfile.ZipFile(ARCHIVE) as archive:
            manifest = json.loads(archive.read('manifest.json'))
            self.assertEqual(len(manifest), 51)
            for name, expected in manifest.items():
                data = archive.read(name)
                self.assertEqual(len(data), expected['bytes'], name)
                self.assertEqual(hashlib.sha256(data).hexdigest(), expected['sha256'], name)
            self.assertFalse(any(n.endswith('.mp4') for n in archive.namelist()))

    def test_real_dds_rejects_combined_age_duplicates_and_missing_ack_then_recovers(self):
        report = self.document('freshness_repair/dds_result.json')
        self.assertTrue(report['passed'])
        self.assertEqual(report['errors'], [])
        for phase in ('native_combined_age', 'native_duplicate', 'native_unacknowledged'):
            last = report['phases'][phase][-1]
            self.assertEqual(last['reason'], 'native_control_unavailable')
            self.assertEqual(last['brake'], 1.)
        recovered = report['phases']['native_recovered'][-1]
        self.assertEqual(recovered['reason'], 'tracking')
        self.assertTrue(recovered['native_health_ok'])

    def test_real_connection_delay_brakes_and_recovers_with_no_vehicle_motion(self):
        report = self.document('freshness_repair/beamng_result2.json')
        self.assertTrue(report['passed'])
        self.assertEqual(report['errors'], [])
        self.assertFalse(report['moving_test'])
        self.assertTrue(report['simulation_paused'])
        for phase in ('delayed_control', 'delayed_poll'):
            samples = report['phases'][phase]
            self.assertTrue(any(r['kind'] == 'native' and not r['data']['telemetry_ok'] for r in samples))
            self.assertTrue(any(r['kind'] == 'consumer' and r['data']['brake'] == 1.
                                and r['data']['reason'] == 'native_control_unavailable' for r in samples))
        for phase in ('baseline', 'control_recovered', 'poll_recovered'):
            last = [r['data'] for r in report['phases'][phase] if r['kind'] == 'consumer'][-1]
            self.assertEqual(last['reason'], 'tracking')
        for state in (report['before'], report['after']):
            self.assertLess(state['speed'], .05)
            self.assertEqual(state['fixedNodeCount'], 0)
            self.assertEqual(state['bodyCollisionNodeCount'], 467)
            self.assertEqual([w['collisionNodeCount'] for w in state['wheels']], [32]*4)

    def test_frozen_deployment_matches_archived_source_and_production_remains_clean(self):
        report = self.document('freshness_repair/verified_after.json')
        self.assertEqual(report['remaining_motion_clients'], [])
        self.assertEqual(report['production_status'], '')
        # Historical evidence certifies its frozen version, not future edits.
        with zipfile.ZipFile(ARCHIVE) as archive:
            for name, meta in report['sources'].items():
                data = archive.read('freshness_repair/sources/' + name)
                self.assertEqual(hashlib.sha256(data).hexdigest(), meta['sha256'], name)

    def test_diagnostic_failures_and_unresolved_engine_gap_are_preserved(self):
        first = self.document('freshness_repair/beamng_result1.json')
        self.assertFalse(first['passed'])
        self.assertIn('AssertionError: delayed_poll', first['errors'])
        summary = self.document('summary.json')
        self.assertIn('UNRESOLVED', summary['engine_gap_root_cause'])
        self.assertIn('PENDING', summary['full_search_acceptance'])
        baseline = self.document('rpc_probe1/audit.json')
        self.assertEqual(baseline['actuator_errors'], 0)
        self.assertFalse(baseline['original_diagnostic_result_retained'])
        self.assertEqual(self.document('rpc_probe1/cleanup_verified.json')['remaining_owned'], [])


if __name__ == '__main__': unittest.main()
