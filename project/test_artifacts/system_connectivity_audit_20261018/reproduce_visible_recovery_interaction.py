"""Counterexamples: visible-route timeout erases physical recovery state. No fix."""
from pathlib import Path
import io
import json
import sys
import unittest
from types import SimpleNamespace as S

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT/'nav2_adapter'), str(ROOT/'nav2_adapter/test'),
               str(ROOT/'official_adapter'),
               str(ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
from test_navigation_transactions_ros import NavigationTransactionTests

observations = {}


class VisibleRecoveryInteraction(unittest.TestCase):
    def setUp(self):
        self.f = NavigationTransactionTests(); self.f.setUp()
        self.n = self.f.node

    def tearDown(self): self.f.tearDown()

    def visible(self):
        responses = self.f.service()
        responses[-1].set_result(S(success=True))
        self.n.query = 'green residential house'
        self.n.expected = (6., 0.)
        self.n.visible_approach_active = True
        self.n.visible_approach_started_at = self.f.wall-11.

    def first_stall(self):
        self.n.send_motion_segment()
        handle, result = self.f.accept()
        self.n.escape_trace.extend((i*.3, 0., 0., self.f.wall) for i in range(-10, 1))
        self.n.follow_failed('execution_stalled', stall_class='actuated_no_path_progress')
        return handle, result

    def test_overdue_visible_timer_erases_first_stall_and_driven_corridor(self):
        self.visible(); self.first_stall()
        self.assertEqual(self.n.last_status['status'], 'follow_failed')
        self.assertEqual(len(self.n.escape_failures), 1)
        self.assertEqual(len(self.n.escape_trace), 11)
        self.assertEqual(self.n.blocked_until-self.f.wall, 2.)
        self.assertTrue(self.n.follow_quarantine)
        self.n.activate_visible_approach()
        self.assertEqual(self.n.last_status['status'], 'visible_target_route_timeout')
        self.assertFalse(self.n.escape_failures)
        self.assertFalse(self.n.escape_trace)
        self.assertIsNone(self.n.blocked_endpoint)
        self.assertTrue(self.n.follow_quarantine)  # Safety barrier correctly survives.
        observations['first_stall'] = dict(
            failure_count_before=1, corridor_samples_before=11,
            failure_count_after=len(self.n.escape_failures),
            corridor_samples_after=len(self.n.escape_trace),
            retry_backoff_sec=2., timeout_timer_period_sec=.25,
            old_action_quarantine_preserved=True,
            event='visible timer removes recovery evidence before retry can run')

    def test_overdue_visible_timer_discards_escape_awaiting_measured_stop(self):
        self.visible(); self.f.prepare_quarantined_escape()
        self.assertIsNotNone(self.n.deferred_execution_escape)
        self.assertEqual(self.n.last_status['status'], 'recovery_waiting_for_stop')
        self.assertTrue(self.n.follow_quarantine)
        self.n.activate_visible_approach()
        self.assertIsNone(self.n.deferred_execution_escape)
        self.assertFalse(self.n.escape_trace)
        self.assertTrue(self.n.follow_quarantine)
        self.assertEqual(self.n.last_status['status'], 'visible_target_route_timeout')
        observations['second_stall'] = dict(
            pending_escape_before=True, pending_escape_after=False,
            old_action_quarantine_preserved=True,
            event='Observation-route deadline revokes physical recovery while stop proof is pending')

    def test_ordinary_failure_retains_corridor_and_first_failure(self):
        self.first_stall()
        self.n.activate_visible_approach()
        self.assertEqual(len(self.n.escape_failures), 1)
        self.assertEqual(len(self.n.escape_trace), 11)
        self.assertEqual(self.n.last_status['status'], 'follow_failed')
        observations['ordinary_reference'] = dict(failure_count=1, corridor_samples=11,
                                                   recovery_state_retained=True)


if __name__ == '__main__':
    stream = io.StringIO()
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(VisibleRecoveryInteraction))
    (OUT/'visible_recovery_interaction.log').write_text(stream.getvalue(), encoding='utf8')
    report = dict(scope=__doc__, tests=result.testsRun, failures=len(result.failures),
                  errors=len(result.errors), expected_gap_reproduced=result.wasSuccessful(),
                  observations=observations, production_changed=False,
                  repair_verified=False, real_dds=False, physical_simulation=False)
    (OUT/'visible_recovery_interaction.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(not result.wasSuccessful())
