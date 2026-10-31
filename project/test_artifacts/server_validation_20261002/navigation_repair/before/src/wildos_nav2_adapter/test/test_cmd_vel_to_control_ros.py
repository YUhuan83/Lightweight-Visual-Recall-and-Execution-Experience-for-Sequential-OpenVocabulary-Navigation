import math
import unittest
from unittest.mock import Mock

import rclpy
from wildos_nav2_adapter.cmd_vel_to_control import CmdVelToControl


class CmdVelNodeSafetyTests(unittest.TestCase):
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
            node.tick()
            control = node.pub.publish.call_args.args[0]
            # 8 m rear-axle curve: measured SBR wheelbase and steering scale.
            expected = math.atan(2.501 / 8.) / .54 * node.get_parameter('steer_sign').value
            self.assertAlmostEqual(control.steering, expected)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
