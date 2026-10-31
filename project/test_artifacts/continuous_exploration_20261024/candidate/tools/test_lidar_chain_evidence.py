"""Check frozen input-chain evidence; diagnostic teleports are not search success."""
import hashlib
import io
import json
from pathlib import Path
import unittest
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / 'test_artifacts/server_validation_20261011'


class LidarChainEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.archive = zipfile.ZipFile(DIRECTORY / 'lidar_chain_evidence.zip')
        with zipfile.ZipFile(DIRECTORY / 'pilot4_closed_evidence.zip') as original:
            surface = json.loads(original.read('physical_green_house_pilot4/static_surface_probe.json'))
        cls.site = np.array([p for p in surface['rows'] if p[4] > 52.8])

    @classmethod
    def tearDownClass(cls):
        cls.archive.close()

    def document(self, name):
        return json.loads(self.archive.read(name))

    def hits(self, name):
        with np.load(io.BytesIO(self.archive.read(name))) as arrays:
            points = arrays['raw']
        distance = np.sqrt(np.min(np.sum((points[:, None, :2] - self.site[None, :, :2])**2, axis=2), axis=1))
        return len(points), int(((distance < .4) & (points[:, 2] > 52.8) & (points[:, 2] < 53.4)).sum())

    def test_archive_and_every_member_are_closed_and_unchanged(self):
        self.assertEqual(hashlib.sha256((DIRECTORY / 'lidar_chain_evidence.zip').read_bytes()).hexdigest(),
                         '534b8b40280308bde3dea209bd81d19280daa3aa2117cd48d4ea6d990db56563')
        manifest = self.document('manifest.json')
        self.assertFalse(manifest['video_recording_enabled'])
        self.assertFalse(any(name.endswith('.mp4') for name in self.archive.namelist()))
        self.assertEqual(len(manifest['files']), 91)
        for name, expected in manifest['files'].items():
            with self.subTest(name=name):
                data = self.archive.read(name)
                self.assertEqual(len(data), expected['size'])
                self.assertEqual(hashlib.sha256(data).hexdigest(), expected['sha256'])

    def test_running_brake_held_reconstruction_retains_missing_returns(self):
        run = self.document('mounted_running_scan.json')
        self.assertTrue(run['success'])
        self.assertEqual(run['stages']['raw']['structure_hits'], 0)
        self.assertEqual(run['fresh_attached']['raw']['structure_hits'], 0)
        self.assertTrue(all(row['raw']['structure_hits'] > 1000 for row in run['paired_static']))
        self.assertLess(run['after']['speed'], .05)
        self.assertLess(run['position_restore_error_m'], .01)
        self.assertEqual(run['before']['vehicleID'], run['after']['vehicleID'])
        self.assertEqual(run['after']['cameras'], [0])
        self.assertEqual(run['after']['lidars'], [1])
        self.assertFalse(run['after']['configKinematic'])
        self.assertEqual(run['after']['fixedNodeCount'], 0)
        self.assertEqual(run['after']['bodyCollisionNodeCount'], 467)
        self.assertTrue(all(wheel['collisionNodeCount'] == 32 for wheel in run['after']['wheels']))
        self.assertFalse(self.document('mounted_running_scan_attempt1.json')['success'])
        self.assertFalse(self.document('mounted_running_scan_attempt2.json')['success'])

    def test_parameter_controls_and_raw_point_counts_agree(self):
        result = self.document('mounted_config_controls.json')
        self.assertTrue(result['success'])
        rows = {row['key']: row for row in result['config_controls']}
        self.assertEqual(set(rows), {'high_horizontal', 'high_90', 'roof_horizontal', 'roof_tilted', 'high_orthogonal_up'})
        for name, row in rows.items():
            with self.subTest(name=name):
                count, hits = self.hits('config_' + name + '.npz')
                self.assertEqual((count, hits), (row['raw']['total'], row['raw']['structure_hits']))
        for name in ('high_90', 'roof_tilted', 'high_orthogonal_up'):
            self.assertEqual(rows[name]['raw']['structure_hits'], 0)
        self.assertGreater(rows['high_horizontal']['raw']['structure_hits'], 0)
        self.assertGreater(rows['roof_horizontal']['raw']['structure_hits'], 1000)
        stages = self.document('config_filter_stages.json')
        self.assertEqual(stages['roof_horizontal']['markings']['structure_hits'], 33)
        self.assertEqual(stages['roof_tilted']['markings']['structure_hits'], 0)

    def test_four_heading_controls_are_not_autonomous_vehicle_turning(self):
        result = self.document('mounted_heading_controls.json')
        self.assertTrue(result['success'])
        self.assertIn('teleports', result['scope'])
        rows = result['config_controls']
        self.assertEqual(len(rows), 12)
        for row in rows:
            with self.subTest(name=row['key']):
                count, hits = self.hits('yaw_' + row['key'] + '.npz')
                self.assertEqual((count, hits), (row['raw']['total'], row['raw']['structure_hits']))
                if row['key'].startswith('level_roof_'):
                    self.assertGreater(hits, 400)
                    self.assertTrue(all(count > 1000 for count in row['azimuth_quadrants']))
        self.assertEqual(result['position_restore_error_m'], 0.)
        self.assertLess(result['after']['speed'], .05)

    def test_map_observation_and_failed_diagnostic_exit_are_separate(self):
        rows = self.document('horizontal_costmap_replay_result.json')
        roof = next(row for row in rows if row['case'] == 'roof_horizontal')
        self.assertTrue(roof['success'])  # Observation collection only.
        self.assertEqual(len(roof['static_hit_costs']), 31)
        self.assertGreater(roof['lethal_static_hits'], 0)
        self.assertEqual(roof['free_static_hits'], 0)
        for row in rows:
            self.assertTrue(row['lifecycle_cleanup_completed'])
            self.assertEqual(row['child_returncode'], -11)

    def test_candidate_only_changes_lidar_mount_and_level_direction(self):
        original = self.document('original_rig.json')['vehicles'][0]['sensors']
        candidate = self.document('candidate_rig.json')['vehicles'][0]['sensors']
        self.assertEqual(candidate, json.loads((ROOT / 'config/sbr_physical_sensor_rig.json').read_text())['vehicles'][0]['sensors'])
        for before, after in zip(original, candidate):
            if before['type'].startswith('Lidar.'):
                self.assertEqual({key: value for key, value in before.items() if key not in ('pos', 'dir')},
                                 {key: value for key, value in after.items() if key not in ('pos', 'dir')})
                self.assertEqual(after['pos'], [0., 0., 1.7])
                self.assertEqual(after['dir'], [0., -1., 0.])
            else:
                self.assertEqual(before, after)


if __name__ == '__main__':
    unittest.main()
