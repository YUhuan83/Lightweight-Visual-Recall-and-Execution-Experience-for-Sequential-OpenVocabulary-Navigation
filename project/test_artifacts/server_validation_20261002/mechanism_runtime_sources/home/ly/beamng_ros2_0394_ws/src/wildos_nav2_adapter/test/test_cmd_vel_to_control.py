import math
import unittest

from wildos_nav2_adapter.drive_math import (
    bounded_cmd_velocity,
    drive_outputs,
    steering_from_angular,
    speed_exceeds_limit,
)


class DriveOutputTests(unittest.TestCase):
    def test_disabled_speed_limits_preserve_commands_and_reject_invalid_telemetry(self):
        self.assertEqual(bounded_cmd_velocity(5., .5, 0., .8, True), (5., .5))
        self.assertEqual(bounded_cmd_velocity(-5., -.5, 0., .8, True), (-5., -.5))
        self.assertFalse(speed_exceeds_limit(5., 0.))
        self.assertTrue(speed_exceeds_limit(5., 2.))
        self.assertTrue(speed_exceeds_limit(float('nan'), 0.))
        self.assertTrue(speed_exceeds_limit(float('inf'), 0.))
    def test_town_launch_gain_produces_starting_throttle(self):
        gear, throttle, brake = drive_outputs(
            0.6, 0.0, 0.20, gear_forward=2, gear_reverse=-1,
            allow_reverse=False)
        self.assertEqual(gear, 2)
        self.assertAlmostEqual(throttle, 0.12)
        self.assertEqual(brake, 0.0)

    def test_launch_throttle_is_capped_while_cruise_gain_remains_active(self):
        _, launch_throttle, _ = drive_outputs(
            1.2, 0.0, 0.30, gear_forward=2, gear_reverse=-1,
            allow_reverse=False, max_throttle=0.20, brake_gain=0.5)
        _, cruise_throttle, _ = drive_outputs(
            1.2, 1.0, 0.30, gear_forward=2, gear_reverse=-1,
            allow_reverse=False, max_throttle=0.20, brake_gain=0.5)
        self.assertAlmostEqual(launch_throttle, 0.20)
        self.assertAlmostEqual(cruise_throttle, 0.06)

    def test_launch_cap_and_minimum_cruise_throttle(self):
        _, launch_throttle, _ = drive_outputs(
            1.2, 0.0, 0.20, gear_forward=2, gear_reverse=-1,
            allow_reverse=False, max_throttle=0.12, min_throttle=0.06,
            brake_gain=0.5)
        _, cruise_throttle, _ = drive_outputs(
            1.2, 1.0, 0.20, gear_forward=2, gear_reverse=-1,
            allow_reverse=False, max_throttle=0.12, min_throttle=0.06,
            brake_gain=0.5)
        self.assertAlmostEqual(launch_throttle, 0.12)
        self.assertAlmostEqual(cruise_throttle, 0.06)

    def test_reverse_is_opt_in_and_uses_reverse_gear_with_positive_throttle(self):
        gear, throttle, brake = drive_outputs(
            -0.5, 0.0, 0.35, gear_forward=2, gear_reverse=-1,
            allow_reverse=True)
        self.assertEqual(gear, -1)
        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)

    def test_reverse_remains_braked_when_not_opted_in(self):
        gear, throttle, brake = drive_outputs(
            -0.5, 0.0, 0.35, gear_forward=2, gear_reverse=-1,
            allow_reverse=False)
        self.assertEqual(gear, 2)
        self.assertEqual(throttle, 0.0)
        self.assertEqual(brake, 1.0)


    def test_rejected_reverse_command_also_clears_steering(self):
        speed, angular = bounded_cmd_velocity(-0.5, 0.4, 0.6, 0.8, False)
        self.assertEqual((speed, angular), (0.0, 0.0))

    def test_opted_in_reverse_command_remains_signed_and_bounded(self):
        speed, angular = bounded_cmd_velocity(-2.0, 1.2, 0.6, 0.8, True)
        self.assertEqual((speed, angular), (-0.6, 0.8))

    def test_forward_and_reverse_steering_have_opposite_sign(self):
        forward = steering_from_angular(0.2, 1.0, 3.0, 0.6, 0.5)
        reverse = steering_from_angular(0.2, -1.0, 3.0, 0.6, 0.5)
        self.assertGreater(forward, 0.0)
        self.assertAlmostEqual(forward, -reverse)

    def test_same_curvature_keeps_steering_at_low_approach_speed(self):
        curvature = 1.0 / 8.0
        expected = math.atan(3.0 * curvature) / 0.6
        for speed in (0.2, 0.4, 1.2, -0.2, -0.4, -1.2):
            with self.subTest(speed=speed):
                steering = steering_from_angular(speed * curvature, speed, 3., .6)
                self.assertAlmostEqual(steering, expected)

    def test_stopped_vehicle_does_not_get_a_turn_in_place_steering_command(self):
        self.assertEqual(steering_from_angular(.5, 0., 3., .6), 0.)

    def test_overspeed_brakes_in_reverse_without_negative_throttle(self):
        gear, throttle, brake = drive_outputs(
            -0.3, 0.6, 0.35, gear_forward=2, gear_reverse=-1,
            allow_reverse=True)
        self.assertEqual(gear, -1)
        self.assertEqual(throttle, 0.0)
        self.assertAlmostEqual(brake, 0.15)

    def test_small_overspeed_gets_proportional_braking(self):
        gear, throttle, brake = drive_outputs(
            1.2, 1.3, 0.20, gear_forward=2, gear_reverse=-1,
            allow_reverse=False)
        self.assertEqual(gear, 2)
        self.assertEqual(throttle, 0.0)
        self.assertAlmostEqual(brake, 0.05)

    def test_target_speed_deadband_does_not_apply_throttle_or_brake(self):
        gear, throttle, brake = drive_outputs(
            1.2, 1.18, 0.20, gear_forward=2, gear_reverse=-1,
            allow_reverse=False)
        self.assertEqual(gear, 2)
        self.assertEqual((throttle, brake), (0.0, 0.0))

    def test_zero_command_preserves_current_reverse_gear(self):
        gear, throttle, brake = drive_outputs(
            0.0, 0.0, 0.35, gear_forward=2, gear_reverse=-1,
            allow_reverse=True, current_gear=-1)
        self.assertEqual(gear, -1)
        self.assertEqual(throttle, 0.0)
        self.assertEqual(brake, 1.0)


if __name__ == "__main__":
    unittest.main()
