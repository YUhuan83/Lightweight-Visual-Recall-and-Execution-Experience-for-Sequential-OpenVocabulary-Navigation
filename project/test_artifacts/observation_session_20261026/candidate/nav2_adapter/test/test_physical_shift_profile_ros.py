"""Exercise the deployed physical profile through real controller ticks."""
import unittest
from types import SimpleNamespace as S
import rclpy
import test_cmd_vel_to_control_ros as fixtures

class PhysicalShiftProfileTests(unittest.TestCase):
    ready = fixtures.SignedGroundFeedbackTests.ready
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def physical_shift_profile(self, target, velocity=(0., 0.)):
        import ast
        from pathlib import Path
        n, state = self.ready(target, velocity)
        launch = Path(__file__).resolve().parents[1]/'launch/physical_vehicle_control.launch.py'
        tree = ast.parse(launch.read_text(encoding='utf8'))
        confirmation = n.get_parameter('gear_change_confirmation_time').value
        for item in ast.walk(tree):
            if isinstance(item, ast.Dict):
                for key, value in zip(item.keys, item.values):
                    if isinstance(key, ast.Constant) and key.value == 'gear_change_confirmation_time':
                        confirmation = ast.literal_eval(value)
        original = n.get_parameter
        n.get_parameter = lambda name: S(value=confirmation) if name == 'gear_change_confirmation_time' else original(name)
        n.current_gear = int(n.get_parameter('gear_forward' if target < 0 else 'gear_reverse').value)
        return n, state

    def test_physical_profile_stable_reverse_does_not_wait_two_seconds(self):
        n, _ = self.physical_shift_profile(-1.2)
        n.tick(); self.clock += .35; n.tick()
        c = n.pub.publish.call_args.args[0]
        self.assertEqual(c.gear, int(n.get_parameter('gear_reverse').value))
        self.assertGreater(c.throttle, 0.)

    def test_physical_profile_stable_forward_does_not_wait_two_seconds(self):
        n, _ = self.physical_shift_profile(1.2)
        n.tick(); self.clock += .35; n.tick()
        c = n.pub.publish.call_args.args[0]
        self.assertEqual(c.gear, int(n.get_parameter('gear_forward').value))
        self.assertGreater(c.throttle, 0.)

    def test_physical_profile_keeps_flicker_and_moving_shift_guards(self):
        n, _ = self.physical_shift_profile(-1.2)
        original = n.current_gear
        n.tick(); self.clock += .2; n.target_speed = 1.2; n.tick()
        self.assertEqual(n.current_gear, original)
        self.assertIsNone(n.pending_gear)
        n, _ = self.physical_shift_profile(-1.2, (.25, 0.))
        original = n.current_gear
        n.tick(); self.clock += .35; n.tick()
        c = n.pub.publish.call_args.args[0]
        self.assertEqual(n.current_gear, original)
        self.assertEqual(c.throttle, 0.); self.assertEqual(c.brake, 1.)

    def test_physical_profile_expired_command_still_stops_during_shift(self):
        n, _ = self.physical_shift_profile(-1.2)
        original = n.current_gear
        n.tick(); self.clock += .6; n.tick()
        c = n.pub.publish.call_args.args[0]
        self.assertEqual(n.current_gear, original)
        self.assertEqual(c.throttle, 0.); self.assertEqual(c.brake, 1.)


    def test_expired_command_is_the_stop_cause_with_other_inputs_fresh(self):
        n, _ = self.physical_shift_profile(-1.2)
        n.tick(); self.clock += .6
        n.last_state_time = n.native_control_at = self.clock
        n.native_sample_age_at_receipt = 0.
        n.tick()
        c=n.pub.publish.call_args.args[0]
        self.assertEqual(c.brake,1.)
        self.assertEqual(c.throttle,0.)
        self.assertIsNone(n.pending_gear)
        import json
        self.assertEqual(json.loads(n.control_status_pub.publish.call_args.args[0].data)['reason'],'stale_command')

    def test_stop_interruption_requires_new_direction_confirmation(self):
        n,_=self.physical_shift_profile(-1.2)
        original=n.current_gear
        n.tick(); self.clock += .2
        n.enabled=False; n.tick()
        self.assertIsNone(n.pending_gear)
        n.enabled=True; n.last_command_time=n.last_state_time=n.native_control_at=self.clock
        n.tick(); self.clock += .15; n.tick()
        self.assertEqual(n.current_gear,original)
        self.assertEqual(n.pub.publish.call_args.args[0].throttle,0.)
        self.clock += .2
        n.last_command_time=n.last_state_time=n.native_control_at=self.clock
        n.tick()
        self.assertEqual(n.current_gear,-1)
        self.assertGreater(n.pub.publish.call_args.args[0].throttle,0.)
