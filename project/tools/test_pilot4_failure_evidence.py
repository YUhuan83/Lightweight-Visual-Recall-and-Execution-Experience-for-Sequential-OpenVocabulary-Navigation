"""Check the closed failed trial without treating diagnostics as arrival."""
import hashlib
import json
from pathlib import Path
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT / 'test_artifacts/server_validation_20261011/pilot4_closed_evidence.zip'


class Pilot4FailureEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.archive = zipfile.ZipFile(ARCHIVE)

    @classmethod
    def tearDownClass(cls):
        cls.archive.close()

    def document(self, name):
        return json.loads(self.archive.read(name))

    def trial(self, name):
        return self.document('physical_green_house_pilot4/' + name)

    def test_closed_members_and_every_frozen_source(self):
        self.assertEqual(hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),
                         'd961fec210e0c053e7a372251fd5c79b819e8b0f51f92d186a2b2affd5992729')
        manifest = self.document('manifest.json')
        self.assertFalse(manifest['video_recording_enabled'])
        self.assertFalse(any(n.endswith('.mp4') for n in self.archive.namelist()))
        for name, expected in manifest['files'].items():
            with self.subTest(name=name):
                data = self.archive.read(name)
                self.assertEqual(len(data), expected['size'])
                self.assertEqual(hashlib.sha256(data).hexdigest(), expected['sha256'])
        freeze = self.document('pilot4_session/freeze.json')
        for name, digest in freeze['sources'].items():
            self.assertEqual(hashlib.sha256(self.archive.read('frozen_sources/' + name)).hexdigest(), digest)

    def test_acknowledged_query_and_full_timeout_are_not_success(self):
        result = self.trial('full_test.json')
        self.assertTrue(self.trial('server_preflight.json')['passed'])
        self.assertFalse(result['passed'])
        self.assertFalse(result['automatic_arrival'])
        self.assertEqual(result['failure_reason'], 'search_timeout_without_strict_arrival')
        self.assertGreaterEqual(result['search_elapsed_s'], result['search_timeout_s'])
        self.assertEqual(result['queries_sent'], ['green residential house'])
        self.assertEqual(result['map_reload_count'], 0)
        self.assertEqual(result['vehicle_teleports_during_sequence'], 0)
        self.assertFalse(result['station_preseeded'])
        acknowledgements = [e for e in result['events'] if e.get('kind') == 'vision_query_accepted']
        self.assertEqual(len(acknowledgements), 1)
        self.assertEqual(acknowledgements[0]['payload']['query'], result['query'])
        self.assertFalse(any(e.get('kind') in ('object_mask', 'object_confirmation')
                             for e in result['events']))
        self.assertFalse(result['failure_image']['is_arrival_evidence'])

    def test_stalls_have_fresh_command_and_supported_wheel_evidence(self):
        audit = self.trial('stall_interface_audit.json')
        self.assertEqual(len(audit['stalls']), 25)
        supported = []
        for row in audit['stalls']:
            self.assertEqual(row['event']['reason'], 'execution_stalled')
            for name in ('/spot1/odom', '/global_costmap/costmap', '/local_costmap/costmap',
                         '/wildos/cmd_vel', '/wildos/control_status'):
                self.assertLess(row['streams'][name]['receipt_age_s'], 1.)
            native = row['native']
            if native['speed'] < .01 and native['actual']['throttle'] > .1:
                wheels = (native.get('wheel_contact') or {}).get('wheels', [])
                if len(wheels) == 4 and all(w['downForceRaw'] > 500 for w in wheels):
                    self.assertEqual(native['actual']['gear'], 'R')
                    self.assertEqual(native['actual']['brake'], 0.)
                    self.assertFalse(native['watchdog_stop'])
                    self.assertTrue(native['telemetry_ok'])
                    self.assertTrue(native['actuator_ok'])
                    self.assertLess(native['telemetry_age_sec'], .5)
                    supported.append(row)
        self.assertGreaterEqual(len(supported), 10)
        # Original BeamNG source-clock offsets are not receipt latency.
        self.assertGreater(supported[0]['streams']['/wildos/control_status']['payload']['state_source_age_ros_sec'], 1000.)

    def test_static_structure_and_published_maps_disagree_without_contact_claim(self):
        surface = self.trial('static_surface_probe.json')
        self.assertEqual(surface['before']['position'], surface['after']['position'])
        self.assertEqual(surface['before']['vehicleID'], surface['after']['vehicleID'])
        tall = [p for p in surface['rows'] if p[4] > 52.8]
        self.assertEqual(len(tall), 31)
        self.assertTrue(any(p[2] == -1.25 and p[3] == 0 for p in tall))
        for topic, grid in self.trial('surface_vs_recorded_maps.json').items():
            with self.subTest(topic=topic):
                self.assertEqual(len(grid['high_static_hits']), len(tall))
                self.assertTrue(all(p['published_cost'] == 0 for p in grid['high_static_hits']))
                self.assertTrue(all(not p['blockers'] for p in grid['short_reverse_body_blockers']))
        raw = self.trial('post_trial_raw_probe.json')
        self.assertIn('not historical', raw['scope'])
        self.assertEqual(raw['frame'], 'map')
        self.assertGreater(raw['tall_hits'], 0)
        # Later visibility proves the mesh can yield returns; it cannot prove
        # the earlier scan saw it, or identify which filtering stage lost it.

    def test_original_physics_stop_and_owned_cleanup_remain_intact(self):
        result = self.trial('full_test.json')
        self.assertTrue(result['stationary_after_stop'])
        self.assertTrue(result['simulation_paused'])
        self.assertLess(result['physics_after']['speed'], .05)
        for state in (result['physics_before'], result['physics_after']):
            self.assertFalse(state['configKinematic'])
            self.assertFalse(state['configBodyDisabled'])
            self.assertFalse(state['collisionOverrideLoaded'])
            self.assertEqual(state['fixedNodeCount'], 0)
            self.assertGreater(state['bodyCollisionNodeCount'], 0)
            self.assertEqual(len(state['wheels']), 4)
            self.assertTrue(all(w['collisionNodeCount'] > 0 for w in state['wheels']))
            self.assertTrue(state['cameras'])
            self.assertTrue(state['lidars'])
        self.assertEqual(self.document('pilot4_session/cleanup.json')['remaining_after_sigint'], [])
        original = self.document('pilot4_session/production_unchanged.json')
        self.assertEqual(original['status'], '')
        self.assertEqual(original['head'], 'f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0')
        rows = self.document('pilot4_session/processes_before_cleanup.json')
        libraries = [lib for row in rows for lib in row['libs']]
        self.assertTrue(any(lib['sha256'] == '975d5bae5fd4b1fa51c54b3a61c314adf7fa40411d80af4d5c5300a2a24b1faa' for lib in libraries))
        self.assertTrue(any(lib['sha256'] == 'b6aacf2af48914085f132df76b6c8d69bb2e9b7817927dc863790d5cdf20810e' for lib in libraries))


if __name__ == '__main__':
    unittest.main()
