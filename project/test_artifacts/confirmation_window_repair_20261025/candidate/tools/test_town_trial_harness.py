"""Shared trial callback contracts using offline doubles; no DDS or vehicle."""
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as S
import json
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'nav2_adapter'), str(ROOT/'nav2_adapter/test')]
from offline_ros_fakes import install, expose, message
install()
expose('rcl_interfaces.msg', Parameter=S,
       ParameterValue=message('ParameterValue', type=int, bool_value=bool),
       ParameterType=S(PARAMETER_BOOL=1))
expose('rcl_interfaces.srv', SetParameters=S(Request=message('Request', parameters=list)))
expose('nav2_msgs.srv', ClearEntireCostmap=S(Request=message('ClearRequest')))
expose('sensor_msgs.msg', Image=message('Image'))
import cv2
import numpy as np
from std_msgs.msg import String
import town_trial_harness as module
import wildos_console as console


class HarnessContracts(unittest.TestCase):
    def setUp(self):
        base = ROOT/'test_artifacts/offline_navigation/harness_tmp'
        base.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=base)
        self.output = Path(self.temp.name)
        assert self.output.resolve().is_relative_to(base.resolve())
        self.nodes = []
        self.now = 100*10**9

    def make(self, model='ackermann', **kwargs):
        node = module.Harness(self.output, 123, carrier_model=model,
                              record_video=False, **kwargs)
        node.get_clock = lambda: S(now=lambda: S(nanoseconds=self.now))
        node.query_started_ns = 90*10**9
        self.nodes.append(node)
        return node

    def tearDown(self):
        for n in self.nodes:
            for log in (n.log, n.native_log, n.sensor_timing_log):
                log.close()
            n.destroy_node()
        self.temp.cleanup()

    def test_import_has_no_live_connection_or_trial_initialization(self):
        self.assertFalse(hasattr(module, 'beamng'))
        self.assertFalse(hasattr(module, 'out'))
        n = self.make()
        self.assertIsNone(n.video)
        self.assertFalse((self.output/'front_camera.mp4').exists())

    def test_default_harness_does_not_initialize_a_video_sink(self):
        n=module.Harness(self.output,123,carrier_model='ackermann')
        self.nodes.append(n)
        self.assertIsNone(n.video)
        self.assertIsNone(n.frame_callback)
        n.record_camera()
        self.assertFalse((self.output/'front_camera.mp4').exists())

    def test_physical_status_and_legacy_profile_are_separate(self):
        for model, planner, radius in [('ackermann', 'CarLike', 8.),
                                       ('ideal_point', 'GridBased', 0.)]:
            n = self.make(model)
            n.status(String(data=json.dumps(dict(schema_version=2,
                carrier_model=model, planner_id=planner, minimum_turning_radius_m=radius))))
            self.assertNotIn('runtime_profile_error', n.state)
            n.status(String(data=json.dumps(dict(schema_version=2,
                carrier_model=model, planner_id=planner, minimum_turning_radius_m=radius+1))))
            self.assertIn('runtime_profile_error', n.state)

    def test_parameter_endpoint_matches_the_actual_motion_backend(self):
        for model, expected in [('ackermann', '/wildos_cmd_vel_to_control/set_parameters'),
                                ('ideal_point', '/wildos_kinematic_vehicle_control/set_parameters')]:
            n = self.make(model)
            future = Future(); future.set_result(S(results=[S(successful=True)]))
            client = Mock(); client.wait_for_service.return_value = True
            client.call_async.return_value = future
            n.create_client = Mock(return_value=client)
            n.destroy_client = Mock()
            with patch.object(module.rclpy, 'spin_until_future_complete', create=True):
                n.set_bool('/wildos_cmd_vel_to_control', 'enable', False)
            self.assertEqual(n.create_client.call_args.args[1], expected)
            self.assertFalse(client.call_async.call_args.args[0].parameters[0].value.bool_value)

    def test_stop_attempts_both_gates_despite_a_service_failure(self):
        n = self.make(); calls=[]
        def reject(owner, name, value):
            calls.append((owner, name, value))
            if len(calls)==1: raise RuntimeError('service rejected')
        n.set_bool=reject; n.spin=Mock(return_value=True)
        with self.assertRaisesRegex(RuntimeError, 'Stop gate failure'):
            n.stop()
        self.assertEqual(calls, [('/wildos_cmd_vel_to_control','enable',False),
                                 ('/wildos_graph_path_bridge','follow_path',False)])

    def test_stale_stationary_sample_cannot_pass_stop(self):
        n = self.make(); n.set_bool = Mock()
        n.state.update(native=dict(speed=0), native_at=0.)
        n.spin = lambda seconds, done: done()
        with self.assertRaisesRegex(RuntimeError, 'fresh stationary'):
            n.stop()
        self.assertEqual(n.set_bool.call_count, 2)

    def test_missing_parameter_result_cannot_be_treated_as_acceptance(self):
        n=self.make(); future=Future();future.set_result(S(results=[]))
        client=Mock();client.wait_for_service.return_value=True;client.call_async.return_value=future
        n.create_client=Mock(return_value=client);n.destroy_client=Mock()
        with patch.object(module.rclpy,'spin_until_future_complete',create=True):
            with self.assertRaisesRegex(RuntimeError,'Parameter update rejected'):
                n.set_bool('/wildos_cmd_vel_to_control','enable',False)

    def camera(self, n, stamp, value):
        im = np.full((4,5,3), value, dtype=np.uint8)
        n.camera(S(header=S(stamp=S(sec=stamp//10**9,nanosec=stamp%10**9),frame_id='camera'),
                   height=4,width=5,step=15,encoding='rgb8',data=im.tobytes()))
        return im

    def mask(self, n, stamp):
        n.mask(S(query=n.query,header=S(stamp=S(sec=stamp//10**9,nanosec=stamp%10**9)),
                 object_mask=S(layout=S(dim=[S(size=i) for i in [1,1,4,5]]), data=[1]*20)))

    def test_arrival_snapshot_keeps_the_exact_observation_when_latest_changes(self):
        n=self.make(); stamp=99*10**9
        original=self.camera(n,stamp,40); self.mask(n,stamp)
        self.camera(n,stamp+100000000,190)
        n.confirmation=dict(query=n.query,observation_stamp_ns=stamp)
        evidence=n.save_target_images(n.query)
        rgb=cv2.imdecode(np.frombuffer(Path(evidence['rgb']).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        np.testing.assert_array_equal(rgb,original)
        self.assertEqual(evidence['observation_stamp_ns'],stamp)

    def test_failure_snapshot_uses_raw_rgb_without_an_old_overlay(self):
        n=self.make(); stamp=99*10**9
        self.camera(n,stamp,20); self.mask(n,stamp)
        original=self.camera(n,stamp+100000000,90)
        evidence=n.save_failure_image()
        rgb=cv2.imdecode(np.frombuffer(Path(evidence['rgb']).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        np.testing.assert_array_equal(rgb,original)
        self.assertEqual(evidence['frame_id'],'camera')
        self.assertAlmostEqual(evidence['source_age_ros_sec'],.9)
        self.assertFalse(evidence['is_arrival_evidence'])
        self.assertFalse(list(self.output.glob('*arrival*.png')))

    def test_console_event_sink_accepts_real_status_callback_fields(self):
        n=self.make()
        n.event=lambda kind,**details:console.record_console_event(n,kind,details)
        data=dict(schema_version=2,query='new target',carrier_model='ackermann',
                  planner_id='CarLike',minimum_turning_radius_m=8.,status='cancelled',stamp_ns=self.now)
        n.status(String(data=json.dumps(data)))
        self.assertEqual(n.last_status,data)
        self.assertEqual(n.events[-1]['query'],'new target')

    def test_user_target_text_cannot_escape_arrival_output_directory(self):
        n=self.make(); stamp=99*10**9
        for query in ('../../outside', 'C:\\outside', 'x'*200, 'gas station'):
            n.query=query
            self.camera(n,stamp,40); self.mask(n,stamp)
            n.confirmation=dict(query=query,observation_stamp_ns=stamp)
            evidence=n.save_target_images(query)
            for kind in ('rgb','mask'):
                self.assertEqual(Path(evidence[kind]).parent, self.output)
                self.assertTrue(Path(evidence[kind]).is_file())

    def test_failure_snapshot_reports_stale_frame_as_stale(self):
        n=self.make(); self.camera(n,90*10**9,30)
        self.assertEqual(n.save_failure_image()['source_age_ros_sec'],10.)

    def test_failure_snapshot_without_camera_has_no_fabricated_image(self):
        n=self.make()
        self.assertEqual(n.save_failure_image(),dict(available=False,reason='no_camera_frame'))
        self.assertFalse(list(self.output.glob('*.png')))

    def test_unpaired_frame_cannot_be_used_for_arrival_or_overlay(self):
        n=self.make(); stamp=99*10**9
        self.camera(n,stamp+1,70); self.mask(n,stamp)
        self.assertIsNone(n.latest_overlay)
        n.confirmation=dict(query=n.query,observation_stamp_ns=stamp)
        with self.assertRaisesRegex(RuntimeError,'Exact arrival RGB/mask'):
            n.save_target_images(n.query)
        self.assertFalse(list(self.output.glob('*arrival*.png')))

    def test_old_overlay_does_not_replace_a_new_camera_frame(self):
        captured=[]; n=self.make(frame_callback=lambda image, stamp:captured.append((image,stamp)))
        stamp=99*10**9
        self.camera(n,stamp,20); self.mask(n,stamp)
        self.camera(n,stamp+100000000,90); n.record_camera()
        self.assertTrue(np.all(captured[-1][0]==90))
        self.assertEqual(captured[-1][1],stamp+100000000)

    def test_ambiguous_confirmation_cannot_enter_arrival_history(self):
        n=self.make()
        data=dict(query=n.query,observation_stamp_ns=99*10**9,localization_valid=True,
                  quality=dict(association_validated=False))
        n.confirmed(String(data=json.dumps(data)))
        self.assertFalse(n.confirmation_history)
        data['quality']['association_validated']=True
        n.confirmed(String(data=json.dumps(data)))
        self.assertEqual(len(n.confirmation_history),1)


if __name__=='__main__':
    unittest.main(verbosity=2)
