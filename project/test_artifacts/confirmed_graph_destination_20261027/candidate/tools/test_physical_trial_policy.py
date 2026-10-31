"""Viewer-free trials must retain navigation, sensor and actuator readiness."""
import unittest
import ast
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'nav2_adapter'))
from server_test_preflight import required_process_roles,dds_process_errors
from check_continuous_start_execution import settle


class TrialReadinessPolicyTests(unittest.TestCase):
    def test_stop_requires_sustained_sdk_samples_after_speed_rebound(self):
        now=[0.]
        def read():return {'speed':.08 if .2<=now[0]<.3 else .04}
        def spin(seconds):now[0]+=seconds
        _,samples=settle(read,spin,clock=lambda:now[0],timeout=2.,stable=.5)
        self.assertGreaterEqual(samples[-1]['elapsed'],.8)
        self.assertTrue(any(row['speed']>.05 for row in samples))
        self.assertTrue(all(row['speed']<.05 for row in samples if row['elapsed']>.3))
    def test_unsettled_or_invalid_speed_is_not_accepted(self):
        now=[0.]
        def spin(seconds):now[0]+=seconds
        with self.assertRaises(RuntimeError):settle(lambda:{'speed':.06},spin,
            clock=lambda:now[0],timeout=1.,stable=.5)
        with self.assertRaises(RuntimeError):settle(lambda:{'speed':float('nan')},spin,
            clock=lambda:now[0],timeout=1.,stable=.5)
    def test_full_trial_uses_canonical_sustained_stop_before_pause(self):
        text=(Path(__file__).parent/'town_physical_single_target_trial.py').read_text()
        tree=ast.parse(text)
        call=next(n for n in ast.walk(tree) if isinstance(n,ast.Call)
            and isinstance(n.func,ast.Name) and n.func.id=='settle')
        self.assertEqual({k.arg:ast.literal_eval(k.value) for k in call.keywords},{'timeout':5.,'stable':.5})
        self.assertLess(text.index('after,samples=settle('),text.index('beamng.control.pause()'))
    def test_all_owners_including_sensor_must_use_frozen_transport(self):
        root=Path(__file__).resolve().parents[1]
        profile=(root/'nav2_adapter/config/fastdds_local_udp.xml').read_bytes()
        good={'dds_environment':{'RMW_IMPLEMENTATION':'rmw_fastrtps_cpp',
            'ROS_LOCALHOST_ONLY':'0','FASTRTPS_DEFAULT_PROFILES_FILE':'frozen.xml'}}
        self.assertEqual(dds_process_errors({'controller':[good],'vision':[good]},[good],
            profile,profile_reader=lambda p:profile),[])
        bad={'dds_environment':dict(good['dds_environment'],ROS_LOCALHOST_ONLY='1')}
        for roles,sensor in (({'controller':[good],'vision':[bad]},[good]),
                             ({'controller':[good],'vision':[good]},[bad]),
                             ({'controller':[good],'vision':[good]},[])):
            with self.subTest(roles=roles,sensor=sensor):
                self.assertTrue(dds_process_errors(roles,sensor,profile,profile_reader=lambda p:profile))
    def test_physical_trial_requires_transport_preflight_before_ros_context_or_command_gate(self):
        text=(Path(__file__).parent/'town_physical_single_target_trial.py').read_text()
        self.assertLess(text.index("'--require-local-dds'"),text.index('rclpy.init('))
        self.assertLess(text.index("'--require-local-dds'"),text.index("node.set_bool('/wildos_cmd_vel_to_control','enable',True)"))
    def test_failure_recording_keeps_traversability_and_controller_pose_without_video_payload(self):
        tree=ast.parse((Path(__file__).parent/'town_physical_single_target_trial.py').read_text())
        topic_assignment=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign)
            and any(isinstance(t,ast.Name) and t.id=='topics' for t in n.targets))
        topics=ast.literal_eval(topic_assignment.value)
        self.assertTrue({'/received_global_plan','/spot1/local_terrain','/spot1/local_map',
                         '/wildos/native_control_status','/spot1/odom','/wildos/cmd_vel_stamped'}<=set(topics))
        self.assertFalse(any('image_raw' in topic for topic in topics))
    def test_no_video_trial_keeps_all_physical_execution_owners(self):
        roles=required_process_roles('ackermann')
        self.assertNotIn('rviz',roles)
        self.assertNotIn('visualization',roles)
        self.assertNotIn('kinematic',roles)
        for name in ('planner','controller','mapper','graphnav_planner','graph_bridge',
                     'native_control','speed_control','vision','triangulation','cloud_filter'):
            self.assertIn(name,roles)

    def test_explicit_display_validation_requires_both_display_owners(self):
        roles=required_process_roles('ackermann',require_visualization=True)
        self.assertIn('rviz',roles)
        self.assertIn('visualization',roles)
        self.assertIn('native_control',roles)

    def test_ideal_diagnostics_cannot_acquire_a_physical_command_owner(self):
        roles=required_process_roles('ideal_point')
        self.assertIn('kinematic',roles)
        self.assertNotIn('native_control',roles)
        self.assertNotIn('speed_control',roles)
        with self.assertRaises(ValueError):required_process_roles('renamed_ideal')


if __name__=='__main__':unittest.main()
