"""Check the closed live start/anchor/partial execution evidence independently."""
import hashlib
import json
import math
from pathlib import Path
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT/'test_artifacts/server_validation_20261011/reconnect_component_evidence.zip'


class StartConnectorFieldEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.archive = zipfile.ZipFile(ARCHIVE)

    @classmethod
    def tearDownClass(cls):
        cls.archive.close()

    def document(self, name):
        return json.loads(self.archive.read(name))

    def test_closed_members_match_without_video_or_full_acceptance_claim(self):
        self.assertEqual(hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),
                         'dd7289deb2f68dcb8bb2cad90894746968a12cc6d91217c535a2a1bd8f66ae57')
        manifest = self.document('manifest.json')
        for name, entry in manifest['files'].items():
            data = self.archive.read(name)
            with self.subTest(name=name):
                self.assertEqual(len(data), entry['size'])
                self.assertEqual(hashlib.sha256(data).hexdigest(), entry['sha256'])
        self.assertFalse(any(n.lower().endswith('.mp4') for n in self.archive.namelist()))
        self.assertFalse(manifest['automatic_start_connector_integrated'])
        self.assertFalse(manifest['video_recording_enabled'])

    def test_actual_loading_and_live_parameters_match_the_local_repair(self):
        freeze = self.document('loaded_source_freeze.json')['files']
        # Historical evidence must remain tied to its frozen deployment,
        # rather than fail as current production legitimately changes later.
        with zipfile.ZipFile(ARCHIVE.parent/'candidate_patch.zip') as deployed:
            for key, name in (
                    ('share_config', 'nav2_adapter/config/nav2_ackermann_control.yaml'),
                    ('loaded_graph_route', 'nav2_adapter/wildos_nav2_adapter/graph_route.py'),
                    ('loaded_runtime_contract', 'nav2_adapter/wildos_nav2_adapter/runtime_contract.py')):
                self.assertEqual(hashlib.sha256(deployed.read(name)).hexdigest(),
                                 freeze[key]['sha256'])
        self.assertEqual(hashlib.sha256((ROOT/'ackermann_rpp/src/controller.cpp').read_bytes()).hexdigest(),
                         freeze['rpp_source']['sha256'])
        actual = self.document('stationary_components.json')['actual_parameters']
        self.assertEqual(actual['/global_costmap/global_costmap']['inflation_layer.inflation_radius'], 3.75)
        self.assertEqual(actual['/planner_server']['CarLike.minimum_turning_radius'], 8.)
        self.assertTrue(actual['/controller_server']['FollowPath.use_collision_detection'])
        self.assertFalse(actual['/wildos_cmd_vel_to_control']['enable'])
        self.assertFalse(actual['/wildos_graph_path_bridge']['follow_path'])
        for name, count in (('runtime_contract.log', 19), ('graph_route.log', 24)):
            self.assertIn(f'Ran {count} tests', self.archive.read(name).decode())
            self.assertTrue(self.archive.read(name).decode().rstrip().endswith('OK'))

    def test_live_start_failure_and_continuous_lattice_disagreement(self):
        state = self.document('stationary_components.json')
        self.assertFalse(state['passed'])
        self.assertEqual(state['errors'], ['CarLike did not produce successful path'])
        self.assertEqual(state['plan_status'], 6)
        self.assertTrue(all(state['counts'][k] >= 2 for k in
                            ('odom', 'image', 'camera_info', 'obstacles', 'clearing', 'global', 'local', 'native')))
        self.assertIn('Starting point in lethal space', self.archive.read('stationary_stack.log').decode())
        collision = self.document('live_collision.log')
        self.assertEqual(collision['actual']['continuous_boundary_cost'], 253)
        self.assertEqual(collision['returned_first']['continuous_boundary_cost'], 254)
        self.assertTrue(collision['published_map_not_private_planner_map'])
        # This diagnostic's shortcut metadata is the historical 2 m formula;
        # the actual 3.75 m live parameter is checked separately above.
        self.assertTrue(self.document('manifest.json')['diagnostic_collision_shortcut_radius_is_historical'])

    def test_same_goal_direct_failure_and_real_anchor_planning(self):
        direct = self.document('direct_same_goal.json')
        anchored = self.document('live_anchor_actions.json')
        self.assertEqual(direct['goal'], anchored['goal'])
        self.assertFalse(direct['use_start'])
        self.assertEqual(direct['status'], 6)
        self.assertEqual(direct['path_poses'], 0)
        self.assertEqual(anchored['accepted_count'], 4)
        self.assertEqual(len(anchored['results']), 5)
        for entry in anchored['results']:
            self.assertEqual(entry['status'], 4)
            self.assertEqual(entry['path_frame'], 'spot1/odom')
            if entry['accepted']:
                self.assertLess(entry['anchor_gap_m'], 1e-6)
                self.assertLess(entry['anchor_heading_gap_rad'], 1e-5)
                self.assertIsNone(entry['global_reason'])
                self.assertIsNone(entry['local_reason'])
                self.assertFalse(entry['cusp_already_reached'])
                self.assertLess(entry['input_receipt_age']['local'], 1.5)
        self.assertEqual(anchored['results'][1]['global_reason'], 'excess_curvature')
        timing = self.document('live_anchor_timing.json')
        self.assertEqual(timing['returncode'], 0)
        self.assertGreater(timing['elapsed_sec'], 1.5)

    def test_partial_motion_is_not_relabelled_as_a_pass(self):
        result = self.document('exact_prefix_component/result.json')
        self.assertFalse(result['passed'])
        self.assertEqual(result['errors'], ['Physics: AssertionError()'])
        self.assertEqual(result['action_status'], 4)
        self.assertGreater(result['displacement_m'], .5)
        self.assertLessEqual(result['endpoint_error_m'], .2)
        self.assertLessEqual(result['heading_error_rad'], .35)
        self.assertGreater(result['physics_after']['speed'], .05)
        limit = math.atan(2.501/8.)/.54
        native = [json.loads(line) for line in
                  self.archive.read('exact_prefix_component/native_samples.jsonl').decode().splitlines()]
        moving = [n for n in native if n['sent']['throttle'] > 0.]
        self.assertTrue(moving)
        for sample in moving:
            self.assertLessEqual(abs(sample['sent']['steering']), limit+1e-9)
            self.assertEqual(sample['sent']['gear'], -1)

    def test_independent_stop_recovery_and_owned_cleanup(self):
        stop = self.document('post_stop_settle.json')
        self.assertTrue(stop['passed'])
        self.assertTrue(stop['pause_acknowledged'])
        stable = [s for s in stop['samples'] if s['speed'] < .05]
        self.assertGreaterEqual(stable[-1]['elapsed']-stable[0]['elapsed'], 1.)
        self.assertEqual(self.document('owned_stack_stop.json')['remaining'], [])
        self.assertEqual(self.document('final_process_check.json')['owned_user_core_remaining'], [])
        self.assertGreater(self.document('final_process_check.json')['free_gib'], 1.)
        for snapshot in (stop['physics'], self.document('physical_reconnect.json')['snapshot']):
            self.assertEqual(snapshot['fixedNodeCount'], 0)
            self.assertGreater(snapshot['bodyCollisionNodeCount'], 0)
            self.assertFalse(snapshot['kinematicExtensionLoaded'])
            for wheel in snapshot['wheels']:
                self.assertEqual(wheel['fixedNodeCount'], 0)
                self.assertGreater(wheel['collisionNodeCount'], 0)


if __name__ == '__main__':
    unittest.main()
