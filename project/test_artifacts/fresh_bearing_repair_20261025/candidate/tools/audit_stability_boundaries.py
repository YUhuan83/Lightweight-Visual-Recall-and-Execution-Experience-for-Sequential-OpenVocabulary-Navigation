"""Reproduce boundary defects in an isolated process/domain; never drive BeamNG."""
from pathlib import Path
import argparse
import json
import sys
import time
from copy import deepcopy
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'nav2_adapter'), str(ROOT/'nav2_adapter/test'),
               str(ROOT/'official_adapter'),
               str(ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--real-ros', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.real_ros:
        from offline_ros_fakes import install
        install()
    import rclpy
    from std_msgs.msg import String
    from beamng_wildos_adapter.goal_standoff import GoalStandoff
    from test_execution_mechanism_ros import ExecutionMechanismTests
    if args.real_ros:
        rclpy.init()
        from rclpy.executors import SingleThreadedExecutor
    results = []

    def fixture():
        f = ExecutionMechanismTests()
        f.setUp()
        f.node.carrier_model = 'ideal_point'
        f.node.input_freshness_issue = Mock(return_value=None)
        return f

    f = fixture()
    goal = GoalStandoff()
    goal.get_clock = f.node.get_clock
    goal.on_query(String(data=f.node.query))
    goal.confirmed_instance_anchor = dict(query=f.node.query,
        instance_id='producer-instance-a', observation_stamp_ns=f.now,
        object_position=[20., 0., 2.], radius=.5)
    # Producer selected an instance, but its confirmation was dropped or
    # ignored during memory candidate comparison. The consumer has no ID.
    f.node.active_object_confirmation = None
    f.node.upstream_instance_id = ''
    ack_count = []
    if args.real_ros:
        f.node.create_subscription(String, '/spot1/object_search/instance_release_ack',
                                   lambda m: ack_count.append(m.data), 10)
        executor = SingleThreadedExecutor()
        executor.add_node(f.node); executor.add_node(goal)
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
    else:
        goal.instance_ack_pub = Mock()
        f.node.instance_release_pub = Mock(side_effect=None)
        f.node.instance_release_pub.publish.side_effect = goal.on_instance_release
    f.node.observation_attempts = {(i, i) for i in range(4)}
    f.node.observation_wait_until = time.monotonic()-1
    f.node.monitor_observation()
    for _ in range(5):
        f.node.retry_graph_reset()
        if args.real_ros:
            deadline = time.monotonic()+.2
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
    if not args.real_ros:
        ack_count = list(goal.instance_ack_pub.publish.call_args_list)
    results.append(dict(case='release_without_observed_instance_identity',
        bug_reproduced=f.node.instance_release_pending is not None and len(ack_count)==0,
        producer_instance_before='producer-instance-a',
        producer_instance_after=(goal.confirmed_instance_anchor or {}).get('instance_id'),
        consumer_requested_instance=(f.node.instance_release_pending or {}).get('instance_id'),
        ack_count=len(ack_count), graph_reset_pending=f.node.query_reset_pending,
        real_dds_transport=args.real_ros))
    if args.real_ros:
        executor.shutdown()
    goal.destroy_node(); f.tearDown()

    f = fixture()
    node = f.node
    node.active_object_confirmation = dict(query=node.query, instance_id='producer-instance-a',
        observation_stamp_ns=f.now, object_position=[20., 0.])
    node.confirmed_route_object = (20., 0.)
    node.start_live_observation()
    observation_goal = tuple(node.expected)
    observation_deadline = node.live_observation_deadline
    # Reach the observation position, then receive a closer confirmed route
    # before the ordinary route_done/wait transition clears the old deadline.
    node.odom.pose.pose.position.x, node.odom.pose.pose.position.y = observation_goal
    node.route = None; node.following = node.pending = False
    node.confirmed_route_started_at = time.monotonic()-11.
    data = dict(query=node.query, object_position=[20., 0., 2.], radius=.5,
                standoff=[15.648004908, 0.], observation_stamp_ns=f.now)
    node.route_confirmed_target(data, observation_goal)
    confirmed_goal = tuple(node.expected)
    epoch_before = node.epoch
    node.following = True
    retained = node.live_observation_deadline == observation_deadline
    if retained:
        node.live_observation_deadline = time.monotonic()-1.
    node.monitor_observation()
    results.append(dict(case='old_observation_timeout_cancels_confirmed_route',
        bug_reproduced=retained and confirmed_goal != observation_goal and node.epoch > epoch_before
            and node.expected != confirmed_goal,
        observation_goal=observation_goal, confirmed_goal=confirmed_goal,
        deadline_retained_after_confirmation=retained,
        epoch_before=epoch_before, epoch_after=node.epoch,
        resulting_goal=node.expected, resulting_status=node.last_status['status'],
        real_dds_transport=False))
    f.tearDown()
    if args.real_ros:
        rclpy.shutdown()
    report = dict(scope='Deterministic synthetic boundary probes on current candidate source; '
                        'not a BeamNG movement or full-search test', findings=results,
                  reproduced=sum(r['bug_reproduced'] for r in results))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
