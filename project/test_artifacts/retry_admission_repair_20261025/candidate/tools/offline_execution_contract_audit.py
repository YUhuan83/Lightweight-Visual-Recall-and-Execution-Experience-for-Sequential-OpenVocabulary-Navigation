"""Offline diagnosis only: no sockets, ROS daemon, simulator, or production edits.

The short-section replay uses the endpoint-carrot equation in the local RPP
source and an exact bicycle plant stepped by distance. It excludes steering
dynamics, speed control, DDS, collision checking, and the compiled RPP plugin.
Recorded native samples bracket dispatch; they are not exact callback poses.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'nav2_adapter'), str(ROOT / 'nav2_adapter/test'),
               str(ROOT / 'official_adapter')]
from wildos_nav2_adapter.motion_path import diagnose_maneuver_join, maneuver_reached


def endpoint_replay(position, heading, endpoint, endpoint_heading, *, direction,
                    radius=8., tolerance=.2, step=.002, max_distance=12.):
    """Exact bicycle increments, instantaneous steering, zero transport delay."""
    position = tuple(position)
    minimum = math.inf
    for index in range(math.ceil(max_distance / step) + 1):
        dx, dy = endpoint[0] - position[0], endpoint[1] - position[1]
        x = dx * math.cos(heading) + dy * math.sin(heading)
        y = -dx * math.sin(heading) + dy * math.cos(heading)
        distance = math.hypot(dx, dy)
        minimum = min(minimum, distance)
        reached = maneuver_reached(position, heading, endpoint, endpoint_heading,
                                   tolerance, .35)
        # Match the local C++ direction guard after its stopped-endpoint branch.
        reversed_carrot = direction * (1 if x >= 0 else -1) < 0
        if reached or reversed_carrot or index * step >= max_distance:
            return dict(reached=reached, direction_guard=(not reached and reversed_carrot),
                        minimum_distance_m=minimum, endpoint_distance_m=distance,
                        traveled_m=index * step, final_position=position,
                        final_heading=heading, heading_error=math.remainder(
                            heading - endpoint_heading, 2 * math.pi))
        requested = 2 * y / (distance * distance) if distance * distance > .001 else 0.
        curvature = max(-1 / radius, min(1 / radius, requested))
        signed = direction * step
        turn = signed * curvature
        lx = math.sin(turn) / curvature if abs(curvature) > 1e-10 else signed
        ly = (1 - math.cos(turn)) / curvature if abs(curvature) > 1e-10 else 0.
        position = (position[0] + lx * math.cos(heading) - ly * math.sin(heading),
                    position[1] + lx * math.sin(heading) + ly * math.cos(heading))
        heading += turn


def recorded_geometry():
    from beamng_wildos_adapter.geometry import body_rotation, shifted_body_position
    from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
    file = ROOT / 'nav2_adapter/test/fixtures/physical_short_cusp_join_20261012.json'
    case = json.loads(file.read_text())
    points, headings = case['points'], case['headings']
    rows = []
    for sample in case['observations']:
        native = sample['native']
        vector = lambda v: SimpleNamespace(x=v[0], y=v[1], z=v[2])
        rotation = body_rotation(vector(native['direction']), vector(native['up']))
        corrected = shifted_body_position(native['position'], rotation, SBR_BASE_OFFSET)
        actual_heading = math.atan2(rotation[1, 0], rotation[0, 0])
        old, _ = diagnose_maneuver_join(points, headings, sample['position'], sample['heading'], 8.)
        current, reason = diagnose_maneuver_join(points, headings, sample['position'], sample['heading'],
                                                8., endpoint_position_tolerance=.2)
        rows.append(dict(wall_time=sample['wall_time'],
            previous_cusp_position_error_m=math.dist(sample['position'], points[0]),
            previous_cusp_heading_error_rad=math.remainder(sample['heading']-headings[0], 2*math.pi),
            previous_cusp_disk_accepts=maneuver_reached(sample['position'], sample['heading'],
                                                       points[0], headings[0], .2, .35),
            legacy_join_accepts=old is not None,
            legacy_predicted_gap_m=math.dist(old[-1][:2], points[-1]) if old else None,
            current_join_accepts=current is not None, current_join_reason=reason,
            zero_delay_bicycle=endpoint_replay(sample['position'], sample['heading'],
                                               points[-1], headings[-1], direction=-1),
            full_3d_offset_difference_m=math.dist(sample['position'], corrected[:2]),
            full_3d_offset_replay=endpoint_replay(corrected[:2], actual_heading,
                                                 points[-1], headings[-1], direction=-1)))
    return dict(fixture=str(file.relative_to(ROOT)), fixture_sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                scope=case['kind'], samples=rows,
                nominal_start_replay=endpoint_replay(points[0], headings[0], points[-1], headings[-1], direction=-1))


def callback_rejections():
    # Imported only within this isolated command; never install doubles in a ROS process.
    from offline_ros_fakes import install
    install()
    import test_short_endpoint_handoff_ros as fixture
    from test_graph_path_bridge_routing import route, pose
    obj = fixture.ShortEndpointHandoffTests('test_valid_short_cusp_reports_the_same_endpoint_contract_it_dispatches')
    obj.setUp()
    try:
        node, case = obj.node, obj.case
        sample = case['observations'][0]
        end = case['points'][-1]
        final = (end[0]-5., end[1])
        node.odom.pose.pose = pose(*sample['position']).pose
        node.odom.pose.pose.orientation.z = math.sin(sample['heading']/2)
        node.odom.pose.pose.orientation.w = math.cos(sample['heading']/2)
        node.route = route(sample['position'], final)
        node.route_endpoint = final
        path = route(*case['points'])
        for item, h in zip(path.poses, case['headings']):
            item.pose.orientation.z = math.sin(h/2)
            item.pose.orientation.w = math.cos(h/2)

        def ready():
            now = time.monotonic()
            node.native_status_at = node.odom_received_at = now
            node.grid_received_at = node.grid_arrived_at = node.local_grid_received_at = now
            node.motion_queue = [deepcopy(path), route(end, final)]
            node.following = node.pending = False

        # Inject repeated occurrences of the same rejected handoff. This probes
        # the real callback branch's budget, not an autonomous full-vehicle loop.
        records = []
        for _ in range(8):
            ready()
            node.send_motion_segment(after_maneuver=True)
            records.append(dict(status=deepcopy(node.last_status),
                recovery_attempts=node.recovery_attempts, blocked_endpoint=node.blocked_endpoint,
                start_segment_calls=node.start_segment.call_count))
        post_cusp = dict(injected_rejections=8, observations=records,
                        follow_goals=node.follow_client.send_goal_async.call_count)

        # The same actual-pose failure before the first action is assigned to
        # execution recovery rather than advancing another planning candidate.
        ready()
        node.candidates = [(end[0], end[1], case['headings'][-1], 2.), (*final, 0., 6.)]
        node.candidate_index = 0
        node.try_execution_escape = Mock(return_value=False)
        node.try_candidate = Mock()
        node.send_motion_segment()
        initial = dict(status=deepcopy(node.last_status), candidate_index=node.candidate_index,
            candidate_count=len(node.candidates), next_candidate_calls=node.try_candidate.call_count,
            follow_goals=node.follow_client.send_goal_async.call_count,
            blocked_endpoint=node.blocked_endpoint)
        return dict(scope='Production callbacks with ROS doubles, maps free; no DDS/physics/planner execution',
                    repeated_handoff=post_cusp, initial_infeasible_candidate=initial)
    finally:
        obj.tearDown()


def returned_path_contract():
    from test_graph_route import grid
    from wildos_nav2_adapter.graph_route import validate_vehicle_path
    archive = ROOT / 'test_artifacts/server_validation_20261012/direct_steering_evidence.zip'
    member = 'direct_steering_curve1/execution/result.json'
    with zipfile.ZipFile(archive) as z:
        result = json.loads(z.read(member))
    plan = next(p for p in result['plans'] if p.get('global_reason') == 'excess_curvature')
    points, headings = plan['points'], plan['headings']
    # Translate the recorded shape into a free test grid; do not change heading,
    # spacing, section boundaries, radius, or the curvature validator.
    start = points[0]
    translated = [(p[0]-start[0], p[1]-start[1]) for p in points]
    occupancy = grid(width=320, height=320, resolution=.25)
    occupancy.info.origin.position.x = occupancy.info.origin.position.y = -40.
    reason = validate_vehicle_path(translated, headings, [translated[0], translated[-1]],
        occupancy, translated[-1], start=translated[0], minimum_turning_radius=8., heading_quantization_bins=72)
    return dict(archive=str(archive.relative_to(ROOT)), archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                member=member, planner_action_status=plan['status'], poses=len(points),
                recorded_rejection=plan['global_reason'], offline_free_map_rejection=reason,
                scope='Recorded returned shape with current production validator; raw pre-smoothing path unavailable')


def graph_corridor_contract():
    """Show what a clear obstacle map proves, without inventing terrain labels."""
    from test_graph_route import grid, set_cell
    from wildos_nav2_adapter.graph_route import validate_vehicle_path, distance_to_route
    occupancy = grid(width=320, height=320, resolution=.25)
    occupancy.info.origin.position.x = occupancy.info.origin.position.y = -40.
    headings = [math.pi*i/200 for i in range(201)]
    points = [(8*math.sin(h), 8*(1-math.cos(h))) for h in headings]
    route = [points[0], points[-1]]
    def check():
        return validate_vehicle_path(points, headings, route, occupancy, points[-1],
            start=points[0], minimum_turning_radius=8., heading_quantization_bins=72)
    clear_reason = check()
    set_cell(occupancy, *points[100], 100)
    return dict(scope='Synthetic free occupancy map; no real terrain or plant',
                corridor_limit_m=19.4, maximum_distance_from_graph_edge_m=max(
                    distance_to_route(p, route) for p in points),
                clear_obstacle_map_rejection=clear_reason, marked_obstacle_rejection=check())


def main():
    out = ROOT / 'test_artifacts/offline_execution_audit_20261013'
    out.mkdir(exist_ok=True)
    report = dict(scope='Offline diagnosis, no new server connection or production changes',
                  date='2026-10-13', short_cusp=recorded_geometry(),
                  callback_contracts=callback_rejections(), returned_path=returned_path_contract(),
                  graph_corridor=graph_corridor_contract())
    (out/'audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
