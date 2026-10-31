"""Offline diagnosis: active execution ownership when extension planning fails.

Uses production Python callbacks with ROS doubles; no ROS/DDS, simulator or
compiled Nav2 plugins. Historical defect evidence remains in the original
audit directory; the current regression report is written separately.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'nav2_adapter'),
               str(ROOT/'nav2_adapter/test'), str(ROOT/'official_adapter')]
from offline_execution_contract_audit import endpoint_replay
from wildos_nav2_adapter.motion_path import diagnose_maneuver_join


def short_join_sweep():
    case = json.loads((ROOT/'nav2_adapter/test/fixtures/physical_short_cusp_join_20261012.json').read_text())
    points, headings = case['points'], case['headings']
    counts = {key: 0 for key in ('accepted_reached', 'accepted_failed',
                                'rejected_reached', 'rejected_failed')}
    examples = {}
    for radius in (0., .05, .1, .15, .19):
        for angle in range(12 if radius else 1):
            theta = angle*math.tau/12
            position = (points[0][0]+radius*math.cos(theta),
                        points[0][1]+radius*math.sin(theta))
            for yaw_offset in (-.3, -.2, -.1, 0., .1, .2, .3):
                heading = headings[0]+yaw_offset
                join, reason = diagnose_maneuver_join(points, headings, position, heading,
                                                     8., endpoint_position_tolerance=.2)
                replay = endpoint_replay(position, heading, points[-1], headings[-1],
                                         direction=-1, step=.005)
                key = ('accepted_' if join else 'rejected_') + ('reached' if replay['reached'] else 'failed')
                counts[key] += 1
                if key not in examples:
                    examples[key] = dict(position=position, heading=heading, reason=reason,
                        prediction_gap_m=math.dist(join[-1][:2], points[-1]) if join else None,
                        replay=replay)
    return dict(scope='343 synthetic poses around one recorded short path; not 343 trials',
                counts=counts, examples=examples)


def extension_probe(*, timeout=True, action_status=4, newest_route=False, foreground=False,
                    carrier_model='ackermann'):
    if 'rclpy' not in sys.modules:
        from offline_ros_fakes import install
        install()
    from test_execution_mechanism_ros import ExecutionMechanismTests
    from test_graph_path_bridge_routing import route
    fixture = ExecutionMechanismTests('test_live_target_arrival_waits_then_releases_unconfirmed_goal')
    fixture.setUp()
    node = fixture.node
    try:
        node.carrier_model = carrier_model
        node.planner_id = 'GridBased' if carrier_model == 'ideal_point' else 'CarLike'
        node.minimum_turning_radius = 0. if carrier_model == 'ideal_point' else 8.
        node.expected = node.query = None
        node.following = not foreground
        node.motion_queue = [route((0., 0.), (6., 0.))] if not foreground else []
        old_handle = Mock()
        node.follow_handle = old_handle if not foreground else None
        node.active_follow_serial = node.follow_request_serial = 7
        node.extending_path = not foreground
        node.pending = True
        node.candidates = [(12., 0., 0., 12.)]
        node.candidate_index = 0
        node.plan_handle = Mock()
        old_plan_handle = node.plan_handle
        node.plan_request_started_at = time.monotonic()-node.compute_path_timeout-.1
        if newest_route:
            node.latest_route = route((0., 0.), (80., 0.))
        # Count actual starts but retain production guards and state mutation.
        node.start_segment = Mock(wraps=node.start_segment)
        node.try_execution_escape = Mock(return_value=False)
        old_epoch = node.epoch
        old_serial = node.active_plan_serial = node.plan_request_serial = 1

        def snapshot():
            return dict(epoch=node.epoch, following=node.following, pending=node.pending,
                extending_path=node.extending_path, queued_sections=len(node.motion_queue),
                original_follow_handle_retained=node.follow_handle is old_handle,
                status=deepcopy(node.last_status), start_segment_calls=node.start_segment.call_count,
                follow_goals=node.follow_client.send_goal_async.call_count)

        if timeout:
            node.recover_timed_out_plan()
        else:
            node.candidate_failed('empty_path')
        after_failure = snapshot()
        if foreground:
            response = Mock()
            response.result.return_value = old_plan_handle
            old_plan_handle.accepted = True
            node.on_plan_accepted(response, old_epoch, 0, old_serial)
            delivered = dict(old_planner_cancel_calls=old_plan_handle.cancel_goal_async.call_count)
        else:
            node.odom.pose.pose.position.x = 6.
            future = Mock()
            future.result.return_value.status = action_status
            node.on_follow_result(future, old_epoch, (6., 0.), 7)
            delivered = snapshot()
        return dict(carrier_model=carrier_model, timeout=timeout, action_status=action_status, newest_route=newest_route,
                    foreground=foreground, old_epoch=old_epoch,
                    after_failure=after_failure, after_original_result=delivered)
    finally:
        fixture.tearDown()


def audit():
    sources = ('nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
               'nav2_adapter/wildos_nav2_adapter/continuous_path.py',
               'nav2_adapter/wildos_nav2_adapter/motion_path.py',
               'nav2_adapter/wildos_nav2_adapter/execution_feedback.py')
    return dict(scope='Offline production callback diagnosis; no new field acceptance',
        short_join_sweep=short_join_sweep(), extension_timeout_success=extension_probe(),
        extension_timeout_abort=extension_probe(action_status=6),
        extension_timeout_new_route=extension_probe(newest_route=True),
        ideal_point_timeout_control=extension_probe(carrier_model='ideal_point'),
        extension_non_timeout_control=extension_probe(timeout=False),
        foreground_timeout_control=extension_probe(foreground=True),
        sources={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources})


if __name__ == '__main__':
    out = ROOT/'test_artifacts/planner_ownership_repair_20261013/extension_after.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    report = audit()
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
