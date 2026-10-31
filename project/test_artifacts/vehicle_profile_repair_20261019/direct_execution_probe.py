"""ComputePath -> validated single-gear FollowPath -> original SBR motion, with source ACK."""
import argparse
import copy
import json
import math
from pathlib import Path
import signal
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.srv import GetParameters, SetParameters
    from nav2_msgs.action import ComputePathToPose, FollowPath
    from nav_msgs.msg import Odometry, OccupancyGrid
    from std_msgs.msg import String
    from beamngpy import BeamNGpy
    from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
    import yaml
    from wildos_nav2_adapter.runtime_contract import navigation_runtime_contract, runtime_parameter_value
    from wildos_nav2_adapter.graph_path_bridge import xy, yaw
    from wildos_nav2_adapter.graph_route import validate_vehicle_path, validate_local_prefix
    from wildos_nav2_adapter.motion_path import motion_sections, diagnose_maneuver_join
    from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors
    from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
    import sys
    root = Path(__file__).resolve().parent
    sys.path.insert(0, str(root / 'candidate/tools'))
    from check_continuous_start_execution import settle
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('bounded_physical_execution_probe')
    state = {}; halt = False; bng = None; action_handle = None; result = None; start = None
    report = dict(scope=__doc__, passed=False, errors=[], plans=[], motion_goals=0,
                  graph_bridge_started=False, visual_search_started=False,
                  complete_navigation_passed=False, video=False)
    events = (args.output / 'events.jsonl').open('x')

    def interrupt(*_):
        nonlocal halt
        halt = True

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)

    def event(kind, **data):
        events.write(json.dumps(dict(kind=kind, monotonic_ns=time.monotonic_ns(), wall_ns=time.time_ns(), **data)) + '\n')
        events.flush()

    def receive(key, message):
        if key in ('native', 'control'):
            message = json.loads(message.data)
            event(key, value=message)
        state[key] = message; state[key + '_at'] = time.monotonic()

    for key, cls, topic in (('odom', Odometry, '/spot1/odom'),
                           ('grid', OccupancyGrid, '/global_costmap/costmap'),
                           ('local', OccupancyGrid, '/local_costmap/costmap'),
                           ('native', String, '/wildos/native_control_status'),
                           ('control', String, '/wildos/control_status')):
        node.create_subscription(cls, topic, lambda message, key=key: receive(key, message), 10)

    def spin(seconds, predicate=lambda: False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if halt:
                raise InterruptedError('Execution probe interrupted')
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return True
        return False

    def wait(future, seconds=6.):
        assert spin(seconds, future.done), 'Service/action response deadline'
        assert not future.exception()
        return future.result()

    def enabled(value):
        client = node.create_client(SetParameters, '/wildos_cmd_vel_to_control/set_parameters')
        try:
            assert client.wait_for_service(timeout_sec=4.)
            response = wait(client.call_async(SetParameters.Request(parameters=[Parameter('enable', value=value).to_parameter_msg()])))
            assert len(response.results) == 1 and response.results[0].successful
        finally:
            node.destroy_client(client)

    def source_ready(requested):
        control = state.get('control', {}); native = state.get('native', {})
        owner = control.get('source_control') or {}; receipt = native.get('source_receipt') or {}
        return (control.get('enabled') is True and control.get('source_ack_ready') is True
                and isinstance(owner.get('sender_id'), str) and owner['sender_id']
                and type(owner.get('generation')) is int and owner['generation'] >= 0
                and owner.get('sender_id') == receipt.get('sender_id')
                and owner.get('generation') == receipt.get('generation')
                and isinstance(owner.get('issued_wall_ns'), int) and owner['issued_wall_ns'] >= requested
                and receipt.get('applied') is True
                and native.get('telemetry_ok') is True and native.get('actuator_ok') is True
                and time.monotonic() - state.get('control_at', 0.) < .5
                and time.monotonic() - state.get('native_at', 0.) < .5)

    try:
        required = navigation_runtime_contract(yaml.safe_load((root / 'candidate/nav2_adapter/config/nav2_ackermann_control.yaml').read_text()), 'ackermann')
        required.pop('/wildos_graph_path_bridge')
        required['/wildos_cmd_vel_to_control']['enable'] = False
        actual = {}
        for owner, fields in required.items():
            client = node.create_client(GetParameters, owner + '/get_parameters')
            try:
                assert client.wait_for_service(timeout_sec=8.), owner
                response = wait(client.call_async(GetParameters.Request(names=list(fields))))
                actual[owner] = {name: runtime_parameter_value(v, fields[name])
                                 for name, v in zip(fields, response.values)}
                assert actual[owner] == fields, (owner, actual[owner], fields)
            finally:
                node.destroy_client(client)
        report['actual_parameters'] = actual
        assert all(name != 'wildos_graph_path_bridge' for name, _ in node.get_node_names_and_namespaces())
        assert spin(8., lambda: all(k in state for k in ('odom', 'grid', 'local', 'native', 'control')))
        install_fail_closed_reads(PrefixedLengthSocket)
        bng = BeamNGpy('127.0.0.1', 25252, quit_on_close=False, socket_timeout=5).open(launch=False)
        before = read_physics_snapshot(bng)
        assert not physical_snapshot_errors(before) and before['speed'] < .05
        report['physics_before'] = before
        pose = state['odom'].pose.pose; start = xy(pose); heading = yaw(pose)
        report['initial'] = [*start, heading]
        planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        follower = ActionClient(node, FollowPath, '/follow_path')
        assert planner.wait_for_server(timeout_sec=5.) and follower.wait_for_server(timeout_sec=5.)
        selected = None
        for length in (-3., 3.):
            goal = ComputePathToPose.Goal(); goal.planner_id = 'CarLike'; goal.use_start = False
            goal.goal.header.frame_id = 'spot1/odom'; goal.goal.header.stamp = node.get_clock().now().to_msg()
            # The navigation pose is planar, even though physical odometry has pitch/Z.
            goal.goal.pose.position.x = start[0] + length * math.cos(heading)
            goal.goal.pose.position.y = start[1] + length * math.sin(heading)
            goal.goal.pose.orientation.z = math.sin(heading / 2.)
            goal.goal.pose.orientation.w = math.cos(heading / 2.)
            handle = wait(planner.send_goal_async(goal)); assert handle.accepted
            response = wait(handle.get_result_async()); path = response.result.path
            points = [xy(p.pose) for p in path.poses]; headings = [yaw(p.pose) for p in path.poses]
            item = dict(relative_goal=length, status=response.status, points=points, headings=headings)
            report['plans'].append(item)
            if response.status != 4 or len(points) < 2:
                continue
            reason = validate_vehicle_path(points, headings, [start, points[-1]], state['grid'], points[-1],
                                           start=start, minimum_turning_radius=8., heading_quantization_bins=72)
            reason = reason or validate_local_prefix(points, headings, state['local'], start, heading)
            join, join_reason = diagnose_maneuver_join(points, headings, start, heading, 8.)
            sections = motion_sections(points)
            path_length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
            item.update(validation_reason=reason, join_reason=join_reason, sections=sections, length_m=path_length)
            if (reason is None and join is not None and len(sections) == 1
                    and .8 <= path_length <= 8. and max(math.dist(p, start) for p in points) <= 4.):
                selected = path; break
        assert selected is not None, 'No admitted bounded single-gear plan; controls remain closed'
        requested = time.time_ns(); enabled(True)
        assert spin(10., lambda: source_ready(requested)), 'No post-enable matching source ACK'
        report['post_enable_source'] = copy.deepcopy(state['control'].get('source_control'))
        goal = FollowPath.Goal(path=selected, controller_id='FollowPath', goal_checker_id='goal_checker')
        action_handle = wait(follower.send_goal_async(goal)); assert action_handle.accepted
        report['motion_goals'] = 1
        result = action_handle.get_result_async()
        end = time.monotonic() + 40.
        while not result.done():
            assert time.monotonic() < end, 'Physical action deadline'
            spin(.1)
            native = state.get('native', {})
            assert native.get('telemetry_ok') and native.get('actuator_ok'), 'Native health failure'
            assert time.monotonic() - state.get('native_at', 0.) < 1., 'Native telemetry stale'
            event('motion_pose', point=list(xy(state['odom'].pose.pose)))
        response = wait(result)
        report['action_status'] = response.status
        assert response.status == 4, 'FollowPath did not succeed'
        report['selected_endpoint'] = list(xy(selected.poses[-1].pose))
    except BaseException as exc:
        report['errors'].append(repr(exc))
    finally:
        gate_closed = False
        # Interrupt prevents future motion but must not prevent the stop gate.
        halt = False
        try:
            enabled(False); gate_closed = True
            if action_handle is not None and result is not None and not result.done():
                wait(action_handle.cancel_goal_async(), 5.)
            if bng:
                after, samples = settle(lambda: read_physics_snapshot(bng), spin, timeout=12.)
                report['physics_after'] = after; report['stop_samples'] = samples
                assert not physical_snapshot_errors(after)
                spin(.2)
                report['final'] = list(xy(state['odom'].pose.pose))
                report['displacement_m'] = math.dist(start, report['final']) if start is not None else 0.
                report['sdk_translation_xy_m'] = (math.dist(report['physics_before']['position'][:2], after['position'][:2])
                    if 'physics_before' in report else 0.)
                if 'selected_endpoint' in report:
                    report['endpoint_distance_m'] = math.dist(report['final'], report['selected_endpoint'])
                    report['passed'] = (report['action_status'] == 4 and report['displacement_m'] > .5
                                        and report['sdk_translation_xy_m'] > .5
                                        and report['endpoint_distance_m'] <= 1.)
        except Exception as exc:
            report['errors'].append('Stop: ' + repr(exc))
        if bng:
            try:
                bng.control.pause(); bng.disconnect()
            except Exception as exc:
                report['errors'].append('Pause/disconnect: ' + repr(exc))
        report['stop_gate_closed'] = gate_closed
        report['passed'] = report['passed'] and gate_closed and not report['errors']
        events.close(); node.destroy_node(); rclpy.shutdown()
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k not in ('actual_parameters', 'plans')}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
