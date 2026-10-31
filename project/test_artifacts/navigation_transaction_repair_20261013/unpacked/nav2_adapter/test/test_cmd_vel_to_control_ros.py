import math
import json
import time
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace as S
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import rclpy
from wildos_nav2_adapter.cmd_vel_to_control import CmdVelToControl


class CmdVelNodeSafetyTests(unittest.TestCase):
    def test_native_sample_and_transmission_ages_are_added(self):
        node = CmdVelToControl()
        try:
            with patch('time.time', return_value=100.):
                node.on_native_motion(String(data=json.dumps(dict(
                    wall_time=99.7, speed=0., actuator_ok=True, telemetry_ok=True,
                    telemetry_age_sec=.3, actual=dict(wheelspeed=0.)))))
            self.assertFalse(node.native_control_ok)
            self.assertIsNone(node.native_motion)
        finally: node.destroy_node()

    def test_native_sample_age_keeps_advancing_after_receipt(self):
        node = CmdVelToControl()
        try:
            clock = [100.]
            node._safety_clock = S(now=lambda: S(nanoseconds=int(clock[0]*1e9)))
            node.enabled = True; node.target_speed = 1.2
            with patch('time.time', return_value=100.):
                node.on_native_motion(String(data=json.dumps(dict(
                    wall_time=99.9, speed=0., actuator_ok=True, telemetry_ok=True,
                    telemetry_age_sec=.25, actual=dict(wheelspeed=0.)))))
            self.assertTrue(node.native_control_ok)
            clock[0] = 100.2
            node.last_state_time = node.last_command_time = 100.2
            node.tick()
            status = json.loads(node.control_status_pub.publish.call_args.args[0].data)
            self.assertEqual(status['reason'], 'native_control_unavailable')
            self.assertEqual(status['brake'], 1.)
            self.assertFalse(status['native_health_ok'])
            self.assertAlmostEqual(status['native_sample_age_steady_sec'], .55)
        finally: node.destroy_node()

    def test_duplicate_native_status_does_not_extend_sample_lifetime_and_new_sample_recovers(self):
        node = CmdVelToControl()
        try:
            clock = [100.]
            node._safety_clock = S(now=lambda: S(nanoseconds=int(clock[0]*1e9)))
            node.enabled = True; node.target_speed = 1.2
            def receipt(wall, source, telemetry_age=.25, **extra):
                with patch('time.time', return_value=wall):
                    node.on_native_motion(String(data=json.dumps(dict(
                        wall_time=source, speed=0., actuator_ok=True, telemetry_ok=True,
                        telemetry_age_sec=telemetry_age, actual=dict(wheelspeed=0.), **extra))))
            receipt(100., 99.9)
            clock[0] = 100.2
            receipt(100.2, 99.9)
            node.last_state_time = node.last_command_time = clock[0]
            node.tick()
            self.assertEqual(node.pub.publish.call_args.args[0].brake, 1.)
            receipt(100.2, 100.2, .01, control_acknowledged=True)
            node.tick()
            self.assertEqual(json.loads(node.control_status_pub.publish.call_args.args[0].data)['reason'], 'tracking')
        finally: node.destroy_node()

    def test_invalid_native_ages_or_unacknowledged_control_cannot_arm(self):
        for source, sample_age, ack in ((100.1, 0., True), (100., -.1, True),
                                         (100., float('nan'), True), (100., 0., False)):
            with self.subTest(source=source, sample_age=sample_age, ack=ack):
                node = CmdVelToControl()
                try:
                    with patch('time.time', return_value=100.):
                        node.on_native_motion(String(data=json.dumps(dict(
                            wall_time=source, speed=0., actuator_ok=True, telemetry_ok=True,
                            telemetry_age_sec=sample_age, control_acknowledged=ack,
                            actual=dict(wheelspeed=0.)))))
                    self.assertFalse(node.native_control_ok)
                finally: node.destroy_node()
    def test_brake_status_exposes_stale_input_without_relaxing_guard(self):
        node=CmdVelToControl()
        try:
            node._safety_clock=S(now=lambda:S(nanoseconds=100_000_000_000))
            node.get_clock=lambda:S(now=lambda:S(nanoseconds=100_000_000_000))
            node.enabled=True;node.target_speed=1.2
            node.last_command_time=99.9;node.last_state_time=99.3
            node.native_control_ok=True;node.native_control_at=99.9
            node.last_source_state_ns=99_300_000_000
            node.last_source_state_frame='vehicles/ego'
            node.tick()
            report=json.loads(node.control_status_pub.publish.call_args.args[0].data)
            self.assertEqual(report['reason'],'stale_state')
            self.assertEqual(report['brake'],1.)
            self.assertFalse(report['actuating'])
            self.assertAlmostEqual(report['command_receipt_age_steady_sec'],.1)
            self.assertAlmostEqual(report['state_receipt_age_steady_sec'],.7)
            self.assertAlmostEqual(report['state_source_age_ros_sec'],.7)
            self.assertEqual(report['state_source_frame'],'vehicles/ego')
            node.last_state_time=99.95
            node.tick()
            recovered=json.loads(node.control_status_pub.publish.call_args.args[0].data)
            self.assertEqual(recovered['reason'],'tracking')
            self.assertEqual(recovered['brake'],0.)
            self.assertAlmostEqual(recovered['state_receipt_age_steady_sec'],.05)
        finally:node.destroy_node()

    def test_missing_input_ages_are_null_and_control_stays_disabled(self):
        node=CmdVelToControl()
        try:
            node.tick()
            report=json.loads(node.control_status_pub.publish.call_args.args[0].data)
            self.assertEqual(report['reason'],'disabled')
            self.assertEqual(report['brake'],1.)
            for key in ('command_receipt_age_steady_sec','state_receipt_age_steady_sec',
                        'native_receipt_age_steady_sec','state_source_age_ros_sec',
                        'native_source_wall_time'):
                self.assertIsNone(report[key])
        finally:node.destroy_node()

    @staticmethod
    def report_healthy_native(node):
        now = node._safety_clock.now().nanoseconds * 1e-9
        node.native_control_ok = True
        node.native_control_at = now

    def test_physically_feasible_correction_is_not_clipped_to_planner_radius(self):
        node = CmdVelToControl()
        try:
            command = Twist()
            command.linear.x, command.angular.z = 1.2, 1.2 / 7.7
            node.on_cmd_vel(command)
            self.assertAlmostEqual(node.target_angular, command.angular.z)
            self.assertGreater(node.target_angular / node.target_speed, 1/8.)
            impossible = Twist()
            impossible.linear.x, impossible.angular.z = -1.2, -.5
            node.allow_reverse = True
            node.on_cmd_vel(impossible)
            self.assertLessEqual(abs(node.target_angular / node.target_speed), 1/node.physical_turning_radius + 1e-9)
        finally:
            node.destroy_node()
    def test_observed_wheel_spin_does_not_accumulate_throttle_boost(self):
        node = CmdVelToControl()
        try:
            node.enabled = True
            msg = Twist()
            msg.linear.x = 1.2
            node.on_cmd_vel(msg)
            node.last_state_time = node.last_command_time
            node.pub = Mock()
            node.throttle_loop.integral = .1
            node.on_native_motion(String(data=json.dumps(dict(wall_time=time.time(), speed=0.,
                actuator_ok=True, telemetry_ok=True, telemetry_age_sec=0., control_error=None,
                actual=dict(wheelspeed=.52)))))
            node.tick()
            self.assertLessEqual(node.pub.publish.call_args.args[0].throttle, .06)
            self.assertEqual(node.throttle_loop.integral, 0.)
            node.enabled = False
            node.tick()
            self.assertEqual(node.throttle_loop.output, 0.)
        finally:
            node.destroy_node()

    def test_stale_native_telemetry_clears_wheel_spin_feedback(self):
        node = CmdVelToControl()
        try:
            node.native_motion = dict(speed=0., wheelspeed=.52)
            node.native_motion_at = node._safety_clock.now().nanoseconds * 1e-9
            node.on_native_motion(String(data=json.dumps(dict(
                wall_time=time.time(), speed=0., actuator_ok=True, telemetry_ok=False,
                telemetry_age_sec=1.0, actual=dict(wheelspeed=.52)))))
            self.assertIsNone(node.native_motion)
            self.assertEqual(node.native_motion_at, 0.)
            self.assertFalse(node.native_control_ok)
        finally:
            node.destroy_node()
    def test_default_has_no_fixed_speed_ceiling_or_two_mps_cutoff(self):
        node = CmdVelToControl()
        try:
            node.enabled = True
            msg = Twist()
            msg.linear.x = 4.
            node.on_cmd_vel(msg)
            self.assertEqual(node.target_speed, 4.)
            node.speed = 3.
            node.last_state_time = node.last_command_time
            node.pub = Mock()
            self.report_healthy_native(node)
            node.tick()
            control = node.pub.publish.call_args.args[0]
            self.assertGreater(control.throttle, 0.)
            self.assertEqual(control.brake, 0.)
        finally:
            node.destroy_node()
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_yaw_only_command_brakes_without_hidden_throttle(self):
        node = CmdVelToControl()
        try:
            node.enabled = True
            node.target_speed = 0.0
            node.target_angular = 0.5
            now = node._safety_clock.now().nanoseconds * 1e-9
            node.last_command_time = now
            node.last_state_time = now
            node.pub = Mock()
            self.report_healthy_native(node)

            node.tick()

            control = node.pub.publish.call_args.args[0]
            self.assertEqual(control.throttle, 0.0)
            self.assertEqual(control.brake, 1.0)
            self.assertEqual(control.steering, 0.0)
        finally:
            node.destroy_node()

    def test_runtime_low_speed_command_preserves_rpp_curvature(self):
        node = CmdVelToControl()
        try:
            node.enabled = True
            node.target_speed = .2
            node.target_angular = .2 / 8.
            node.current_gear = int(node.get_parameter('gear_forward').value)
            now = node._safety_clock.now().nanoseconds * 1e-9
            node.last_command_time = node.last_state_time = now
            node.pub = Mock()
            self.report_healthy_native(node)
            node.tick()
            control = node.pub.publish.call_args.args[0]
            # 8 m rear-axle curve: measured SBR wheelbase and steering scale.
            expected = math.atan(2.501 / 8.) / .54 * node.get_parameter('steer_sign').value
            self.assertAlmostEqual(control.steering, expected)
        finally:
            node.destroy_node()

    def test_missing_or_failed_actuator_health_forces_brakes(self):
        node = CmdVelToControl()
        try:
            node.enabled = True
            msg = Twist()
            msg.linear.x = 1.2
            node.on_cmd_vel(msg)
            now = node._safety_clock.now().nanoseconds * 1e-9
            node.last_state_time = node.last_command_time = now
            node.pub = Mock()
            node.tick()
            control = node.pub.publish.call_args.args[0]
            self.assertEqual(control.throttle, 0.)
            self.assertEqual(control.brake, 1.)

            node.on_native_motion(String(data=json.dumps(dict(
                wall_time=time.time(), speed=0., actuator_ok=False, telemetry_ok=True,
                telemetry_age_sec=0., control_error='TimeoutError: timed out',
                actual=dict(wheelspeed=0.)))))
            node.tick()
            control = node.pub.publish.call_args.args[0]
            self.assertEqual(control.throttle, 0.)
            self.assertEqual(control.brake, 1.)
        finally:
            node.destroy_node()

    def test_disabled_stopped_session_resets_reverse_gear_to_forward(self):
        node = CmdVelToControl()
        try:
            node.enabled = False
            node.current_gear = int(node.get_parameter('gear_reverse').value)
            now = node._safety_clock.now().nanoseconds * 1e-9
            node.last_state_time = now
            node.speed = 0.0
            node.pub = Mock()

            node.tick()

            control = node.pub.publish.call_args.args[0]
            self.assertEqual(control.gear, int(node.get_parameter('gear_forward').value))
            self.assertEqual(control.throttle, 0.0)
            self.assertEqual(control.brake, 1.0)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
