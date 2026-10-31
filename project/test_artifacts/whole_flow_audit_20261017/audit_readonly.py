"""Read-only implementation audit. Passing probes reproduce gaps, not repairs."""
from pathlib import Path
import hashlib
import json
import sys
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'nav2_adapter'), str(ROOT/'nav2_adapter/test'),
               str(ROOT/'official_adapter'),
               str(ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
from beamng_msgs.msg import StateSensor
from geometry_msgs.msg import Twist
from wildos_nav2_adapter.cmd_vel_to_control import CmdVelToControl
from wildos_nav2_adapter.continuous_path import trim_moving_path
from wildos_nav2_adapter.graph_route import validate_local_prefix
from beamng_wildos_adapter.measurement_clock import MeasurementClock
from visual_navigation.wildos.graph_observation_store import GraphObservationStore
from visual_navigation.wildos.execution_experience_store import ExecutionExperienceStore
import numpy as np
import test_motion_intent_handoff_ros as handoff
import test_execution_progress_ros as progress

OBSERVATIONS = {}

def inventory():
    manifest = json.loads((ROOT/'deploy/20261017_full_vehicle_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
    paths = set(manifest['members'])
    for folder in ('nav2_adapter', 'official_adapter', 'ackermann_smac', 'ackermann_rpp',
                   'wildos_port/repro_check/src/nebula2-wildos'):
        for p in (ROOT/folder).rglob('*'):
            if p.is_file() and p.suffix in ('.py', '.cpp', '.hpp', '.h', '.yaml', '.xml', '.lua') and not any(
                    part in ('__pycache__', 'build', 'install', '.git') for part in p.relative_to(ROOT).parts):
                paths.add(p.relative_to(ROOT).as_posix())
    hashes = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sorted(paths) if (ROOT/name).is_file()}
    return dict(field_candidate=manifest['sha256'], source_hashes=hashes,
                differences_from_field=[name for name, want in manifest['members'].items() if hashes.get(name) != want])

class WholeFlowGaps(unittest.TestCase):
    def test_heading_negative_evidence_is_shared_maximum(self):
        store = GraphObservationStore(':memory:', 'audit')
        try:
            store.register_map('audit-map')
            store.record_search_coverage('audit-map', 'target', 'node', [0], 1., timestamp=1.)
            store.record_search_coverage('audit-map', 'target', 'node', [8], .1, timestamp=2.)
            actual = store.get_search_coverage('audit-map', 'target', 'node')
            self.assertEqual(actual['negative_evidence'], 1.)
            self.assertEqual(actual['heading_mask'], 257)
            OBSERVATIONS['heading_coverage'] = dict(actual, intended_second_heading_strength=.1)
        finally:
            store.close()

    def test_success_only_execution_samples_always_retrieve_neutral_prior(self):
        store = ExecutionExperienceStore(':memory:', 'audit')
        try:
            # append_scene_sample sets heading_scores=None. The production
            # feedback publisher emits only SUCCEEDED, whose reward is 1.
            for heading in (0., 1., 2., 3.):
                self.assertTrue(store.record_feedback(np.array([1., 0.]), None,
                    heading, 1., 1, heading_count=16))
            prior, similarity = store.retrieve(np.array([1., 0.]), 16)
            np.testing.assert_array_equal(prior, np.ones(16))
            OBSERVATIONS['experience_neutral_prior'] = dict(prior=prior.tolist(),
                similarity=similarity, records=4,
                scope='Fresh same-process store under current successful-only producer; other hypothetical feedback producers excluded')
        finally:
            store.close()

    def test_late_twist_receipt_refreshes_unidentifiable_source(self):
        node = CmdVelToControl()
        try:
            message = Twist(); message.linear.x = 1.2
            node._safety_clock = S(now=lambda: S(nanoseconds=100_000_000_000))
            node.on_cmd_vel(message)
            node._safety_clock = S(now=lambda: S(nanoseconds=105_000_000_000))
            node.on_cmd_vel(message)
            self.assertEqual(node.last_command_time, 105.)
            OBSERVATIONS['twist_source_gap'] = dict(source_header=hasattr(message, 'header'), refreshed_receipt=node.last_command_time)
        finally:
            node.destroy_node()

    def test_sensor_restart_clock_and_control_epoch_disagree(self):
        clock = MeasurementClock()
        node = CmdVelToControl()
        try:
            msg = StateSensor(); msg.dir.x = 1.; msg.header.stamp.sec = 100
            node.on_state(msg)
            clock.update(100_000_000_000, 1_000_000_000_000)
            mapped = clock.update(1_000_000_000, 1_001_000_000_000)
            msg.header.stamp.sec = 1; msg.velocity.x = 1.2
            node.on_state(msg)
            self.assertTrue(clock.last_update_reset)
            self.assertEqual(node.last_source_state_ns, 100_000_000_000)
            self.assertEqual(node.speed, 0.)
            OBSERVATIONS['sensor_restart_epoch'] = dict(adapter_accepts_ns=mapped,
                control_last_source_ns=node.last_source_state_ns, control_speed=node.speed,
                scope='Simulator clock restart while ROS/control processes survive; not shown in last single-load field run')
        finally:
            node.destroy_node()

    def test_late_large_clock_rollback_is_indistinguishable_from_restart(self):
        clock = MeasurementClock()
        clock.update(100_000_000_000, 1_000_000_000_000)
        mapped = clock.update(99_000_000_000, 1_001_000_000_000)
        self.assertTrue(clock.last_update_reset)
        self.assertEqual(mapped, 1_001_000_000_000)
        OBSERVATIONS['clock_rollback_ambiguity'] = dict(source_ns=99_000_000_000,
            mapped_ns=mapped, reset=True, scope='Conditional source rollback; no field occurrence claimed')

    def test_successful_short_sections_have_no_overall_progress_exit(self):
        f = progress.ExecutionProgressTests(); f.setUp()
        try:
            n = f.node; n.reject_exploration_route = Mock()
            f.succeed(0., 1.)
            credited = n.recovery_progress['credited_distance']
            for _ in range(20):
                f.succeed(1., 0.); f.succeed(0., 1.)
            self.assertEqual(n.recovery_progress['credited_distance'], credited)
            self.assertEqual(n.recovery_attempts, 0)
            n.reject_exploration_route.assert_not_called()
            OBSERVATIONS['local_success_loop'] = dict(cycles=20, recovery_attempts=n.recovery_attempts, rejected=False)
        finally:
            f.tearDown()

    def test_unaccepted_new_intention_is_blocked_by_old_action_abort(self):
        f = handoff.MotionIntentHandoffTests(); f.setUp()
        try:
            n = f.node; f.resolve(); old_identity = n.retired_motion['identity']
            n.start_segment = Mock()
            route = f.new_route(); n.maybe_extend_path()
            self.assertIs(n.route, route)
            n.try_execution_escape = Mock(return_value=False)
            f.fixture.complete_section(status=6)
            self.assertEqual(n.blocked_endpoint, (18., 0.))
            self.assertEqual(n.last_status['exploration_id'], 'frontier-b:2')
            OBSERVATIONS['candidate_failure_owner'] = dict(executed_owner=old_identity,
                failure_status_owner=n.last_status['exploration_id'], blocked_endpoint=n.blocked_endpoint,
                scope='Unreleased local handoff candidate; new intention never accepted by FollowPath')
        finally:
            f.tearDown()

    def test_old_tracking_failure_can_reject_unexecuted_new_intention(self):
        f = handoff.MotionIntentHandoffTests(); f.setUp()
        try:
            n = f.node; f.resolve(); n.start_segment = Mock()
            new_route = f.new_route(); n.maybe_extend_path()
            n.try_execution_escape = Mock(return_value=False)
            n.recovery_attempts = 2
            n.follow_failed('tracking_deviation', cross_track_error=3.)
            n.route_rejection_pub.publish.assert_called_once()
            message = n.route_rejection_pub.publish.call_args.args[0]
            self.assertEqual(message.header.stamp, new_route.header.stamp)
            self.assertEqual(message.pose.position.x, 18.)
            OBSERVATIONS['candidate_rejection_owner'] = dict(rejected_endpoint=18.,
                accepted_motion_owner='frontier-a:1', rejected_intention='frontier-b:2',
                recovery_attempts=2, scope='Conditional retry state; no field occurrence claimed')
        finally:
            f.tearDown()

    def test_candidate_rechecks_original_prefix_and_misses_current_forward_obstacle(self):
        f = handoff.MotionIntentHandoffTests(); f.setUp()
        try:
            n = f.node; f.resolve()
            n.odom.pose.pose.position.x = 5.
            # Current body ends at about x=8.4; a newly observed cell at x=9
            # lies in the next two metres of swept footprint.
            n.local_grid.data[10*40+19] = 100
            trimmed = trim_moving_path(n.motion_queue[0], (5., 0.), 0.)
            points = [(p.pose.position.x, p.pose.position.y) for p in trimmed.poses]
            correct = validate_local_prefix(points, [0.]*len(points), n.local_grid, (5., 0.), 0.)
            self.assertEqual(correct, 'local_footprint_collision')
            n.maybe_extend_path()
            f.handle.cancel_goal_async.assert_not_called()
            self.assertTrue(n.following)
            OBSERVATIONS['candidate_prefix_missed_obstacle'] = dict(current_x=5., obstacle_cell_x=9.,
                trimmed_check=correct, candidate_cancelled=False,
                scope='Bridge guard only; RPP live collision checking remains enabled')
        finally:
            f.tearDown()

    def test_candidate_rechecks_old_prefix_outside_rolling_map_and_stops(self):
        f = handoff.MotionIntentHandoffTests(); f.setUp()
        try:
            n = f.node; f.resolve(); n.odom.pose.pose.position.x = 5.
            n.local_grid.info.origin.position.x = 1.
            trimmed = trim_moving_path(n.motion_queue[0], (5., 0.), 0.)
            points = [(p.pose.position.x, p.pose.position.y) for p in trimmed.poses]
            correct = validate_local_prefix(points, [0.]*len(points), n.local_grid, (5., 0.), 0.)
            self.assertIsNone(correct)
            n.maybe_extend_path()
            f.handle.cancel_goal_async.assert_called_once()
            OBSERVATIONS['candidate_prefix_false_stop'] = dict(current_x=5., map_origin_x=1.,
                trimmed_check=correct, candidate_cancelled=True,
                reason=n.last_status.get('reason'))
        finally:
            f.tearDown()

if __name__ == '__main__':
    before = inventory()
    (OUT/'source_inventory.json').write_text(json.dumps(before, indent=2), encoding='utf8')
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(WholeFlowGaps))
    after = inventory()
    report = dict(scope=__doc__, tests=result.testsRun, failures=len(result.failures), errors=len(result.errors),
        observations=OBSERVATIONS, production_sources_unchanged=before==after,
        source_files_checked=len(before['source_hashes']), field_simulation=False,
        callback_transport='ROS interface doubles; real production Python methods')
    (OUT/'diagnostics.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps(report, indent=2))
    raise SystemExit(not (result.wasSuccessful() and before==after))
