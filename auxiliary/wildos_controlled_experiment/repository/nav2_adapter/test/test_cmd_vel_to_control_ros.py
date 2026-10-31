import math
import json
import time
import unittest
from unittest.mock import Mock
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import rclpy
from wildos_nav2_adapter.cmd_vel_to_control import CmdVelToControl


class CmdVelNodeSafetyTests(unittest.TestCase):
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
