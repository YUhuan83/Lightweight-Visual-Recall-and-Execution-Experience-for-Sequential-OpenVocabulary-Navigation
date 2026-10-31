from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\continuous_exploration_20261024')
text='''"""Exercise the deployed physical profile through real controller ticks."""
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

'''+(root/'pending_shift_profile_tests.txt').read_text(encoding='utf8')+'''
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
'''
path=root/'candidate/nav2_adapter/test/test_physical_shift_profile_ros.py'
path.write_text(text,encoding='utf8')
print(path)
