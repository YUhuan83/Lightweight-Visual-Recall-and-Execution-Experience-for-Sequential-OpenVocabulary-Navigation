"""Expose remaining contracts using actual production callbacks and ROS doubles.

Passing here means the gap was reproduced, not that navigation was repaired.
"""
from pathlib import Path
import json
import sys
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'nav2_adapter'), str(ROOT/'nav2_adapter/test')]
from offline_ros_fakes import install
install()
from geometry_msgs.msg import Twist
from wildos_nav2_adapter.cmd_vel_to_control import CmdVelToControl
import test_execution_progress_ros as progress


class ContractGaps(unittest.TestCase):
    def test_delayed_twist_has_no_source_identity_or_age(self):
        node = CmdVelToControl()
        try:
            node._safety_clock = S(now=lambda: S(nanoseconds=100_000_000_000))
            message = Twist()
            message.linear.x = 1.2
            node.on_cmd_vel(message)
            self.assertEqual(node.last_command_time, 100.)
            self.assertEqual(node.target_speed, 1.2)
            self.assertFalse(hasattr(message, 'header'))
            # The same old payload arriving five seconds later is assigned the
            # new receive time. Its actual publication age is unknowable here.
            node._safety_clock = S(now=lambda: S(nanoseconds=105_000_000_000))
            node.on_cmd_vel(message)
            self.assertEqual(node.last_command_time, 105.)
            self.assertEqual(node.target_speed, 1.2)
        finally:
            node.destroy_node()

    def test_successful_sections_can_repeat_without_global_progress_exit(self):
        fixture = progress.ExecutionProgressTests()
        fixture.setUp()
        try:
            node = fixture.node
            node.reject_exploration_route = Mock()
            fixture.succeed(0., 1.)
            credited = node.recovery_progress['credited_distance']
            for _ in range(20):
                fixture.succeed(1., 0.)
                fixture.succeed(0., 1.)
            self.assertEqual(node.recovery_progress['credited_distance'], credited)
            self.assertEqual(node.recovery_attempts, 0)
            node.reject_exploration_route.assert_not_called()
            self.assertFalse(node.object_found)
        finally:
            fixture.tearDown()


if __name__ == '__main__':
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ContractGaps))
    report = dict(tests=result.testsRun, failures=len(result.failures), errors=len(result.errors),
                  defects_reproduced=result.wasSuccessful(), production_changed=False,
                  scope='Actual callback methods with ROS doubles; not evidence that these timings occurred in the field')
    Path(__file__).with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    raise SystemExit(not result.wasSuccessful())
