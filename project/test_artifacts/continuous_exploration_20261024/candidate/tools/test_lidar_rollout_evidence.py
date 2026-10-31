"""Independent checks of actual sensor deployment, maps and limited execution."""
import hashlib
import io
import json
from pathlib import Path
import unittest
import zipfile
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / 'test_artifacts/server_validation_20261011'


class LidarRolloutEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.archive = zipfile.ZipFile(DIRECTORY / 'lidar_rollout_evidence.zip')
        with zipfile.ZipFile(DIRECTORY / 'pilot4_closed_evidence.zip') as prior:
            cls.site = [p for p in json.loads(prior.read(
                'physical_green_house_pilot4/static_surface_probe.json'))['rows'] if p[4] > 52.8]

    @classmethod
    def tearDownClass(cls):
        cls.archive.close()

    def document(self, name):
        return json.loads(self.archive.read(name))

    def test_snapshot_members_and_unmodified_production(self):
        self.assertEqual(hashlib.sha256((DIRECTORY / 'lidar_rollout_evidence.zip').read_bytes()).hexdigest(),
                         '1e2cc12d388143aa7f793e5405204ee2adb377cebe79ce594541353396b1ac95')
        manifest = self.document('manifest.json')
        self.assertFalse(manifest['video_recording_enabled'])
        self.assertEqual(len(manifest['files']), 42)
        for name, item in manifest['files'].items():
            data = self.archive.read(name)
            with self.subTest(name=name):
                self.assertEqual(len(data), item['size'])
                self.assertEqual(hashlib.sha256(data).hexdigest(), item['sha256'])
        production = self.document('lidar_profile_fix/production_unchanged.json')
        self.assertEqual(production['head'], 'f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0')
        self.assertEqual(production['status'], '')

    def test_actual_sensor_profile_and_single_owner(self):
        attachment = self.document('lidar_profile_fix/sensor_attachment.json')
        profile = json.loads((ROOT / 'config/sbr_physical_sensor_rig.json').read_text())
        self.assertTrue(attachment['ready'])
        self.assertEqual(attachment['map_reload_count'], 0)
        self.assertEqual(attachment['sensors'], profile['vehicles'][0]['sensors'])
        self.assertEqual(attachment['before']['vehicleID'], attachment['after']['vehicleID'])
        self.assertEqual(len(attachment['after']['cameras']), 1)
        self.assertEqual(len(attachment['after']['lidars']), 1)
        self.assertFalse(attachment['after']['configBodyDisabled'])
        notes = self.document('lidar_profile_fix/startup_notes.json')
        self.assertTrue(notes['old_rig_process_exited'])
        self.assertNotEqual(notes['launcher_pid'], notes['actual_sensor_owner_pid'])
        self.assertIn('Lua empty sensor table', notes['empty_sensor_assertion_failed'])

    def test_live_input_filter_and_both_map_grids(self):
        result = self.document('lidar_profile_fix/live_map_check.json')
        self.assertTrue(result['passed'])
        self.assertEqual(result['errors'], [])
        self.assertEqual(result['competing_processes'], [])
        self.assertTrue(all(value is False for value in result['actual_parameters'].values()))
        for rows in result['streams'].values():
            self.assertGreaterEqual(len({row['source_ns'] for row in rows}), 3)
        for stage in ('raw', 'mark', 'clear'):
            with np.load(io.BytesIO(self.archive.read('lidar_profile_fix/live_' + stage + '.npz'))) as data:
                points = data['world']
            site = np.asarray(self.site)
            distance = np.sqrt(np.min(np.sum((points[:, None, :2] - site[None, :, :2])**2, axis=2), axis=1))
            hits = int(((distance < .4) & (points[:, 2] > 52.8) & (points[:, 2] < 53.4)).sum())
            self.assertEqual(hits, result['stages'][stage]['structure_hits'])
            self.assertGreater(hits, 0)
        for kind in ('global', 'local'):
            with np.load(io.BytesIO(self.archive.read('lidar_profile_fix/live_' + kind + '_map.npz'))) as data:
                origin = data['origin']; resolution = float(data['resolution'])
                width, height = int(data['width']), int(data['height'])
                costs = []
                for row in self.site:
                    x, y = int((row[0] - origin[0]) / resolution), int((row[1] - origin[1]) / resolution)
                    self.assertTrue(0 <= x < width and 0 <= y < height)
                    costs.append(int(data['data'][y*width + x]))
            self.assertEqual(costs, result[kind]['static_hit_costs'])
            self.assertNotIn(0, costs)
            self.assertGreater(sum(cost == 100 for cost in costs), 0)
        self.assertEqual(result['stack_exit'], 0)
        self.assertEqual(result['remaining_processes'], [])
        self.assertLess(result['position_restore_error_m'], .01)
        self.assertLess(result['after']['speed'], .05)

    def test_first_preparation_timeout_is_preserved(self):
        failed = self.document('lidar_execution_component/execution/result.json')
        self.assertFalse(failed['passed'])
        self.assertIn('RuntimeError: Component service/action timeout', failed['errors'])
        self.assertNotIn('physics_before', failed)
        self.assertFalse(any(event.get('status') == 'following' for event in failed['events']))
        self.assertFalse(failed['failure_image']['is_arrival_evidence'])
        self.assertEqual(self.document('lidar_execution_component/cleanup.json')['remaining'], [])

    def test_real_short_execution_and_sustained_stop_are_not_visual_arrival(self):
        passed = self.document('lidar_execution_component2/execution/result.json')
        self.assertTrue(passed['passed'])
        self.assertEqual(passed['errors'], [])
        self.assertFalse(passed['visual_arrival_validated'])
        self.assertFalse(passed['video_recording_enabled'])
        self.assertEqual((passed['map_reloads'], passed['teleports']), (0, 0))
        self.assertGreater(passed['displacement_m'], 2.)
        self.assertLessEqual(passed['endpoint_distance_m'], 1.)
        self.assertEqual(passed['selected']['status'], 4)
        for reason in ('global_reason', 'local_reason', 'join_reason'):
            self.assertIsNone(passed['selected'][reason])
        self.assertEqual(passed['terminal']['status'], 'route_done')
        self.assertFalse(any(event.get('status') in ('follow_failed', 'replanning_after_maneuver', 'no_vehicle_path')
                             for event in passed['events']))
        for snapshot in (passed['physics_before'], passed['physics_after']):
            self.assertFalse(snapshot['configKinematic'])
            self.assertEqual(snapshot['fixedNodeCount'], 0)
            self.assertEqual(snapshot['nodeCount'], 723)
            self.assertEqual(snapshot['bodyCollisionNodeCount'], 467)
            self.assertTrue(all(wheel['collisionNodeCount'] == 32 for wheel in snapshot['wheels']))
        samples = passed['stop_samples']
        quiet = []
        for row in reversed(samples):
            if row['speed'] >= .05:
                break
            quiet.append(row)
        self.assertGreaterEqual(quiet[0]['elapsed'] - quiet[-1]['elapsed'], .5)
        self.assertLess(passed['physics_after']['speed'], .05)
        self.assertTrue(passed['simulation_paused'])
        self.assertEqual(self.document('lidar_execution_component2/cleanup.json')['remaining'], [])


if __name__ == '__main__':
    unittest.main()
