"""Bounded fault probes of production state callbacks; no DDS or simulator."""
from concurrent.futures import Future
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
import subprocess
import time
from types import SimpleNamespace as S
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'test_artifacts/navigation_state_audit_20261013'
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'nav2_adapter'), str(ROOT/'nav2_adapter/test'),
               str(ROOT/'official_adapter')]
from offline_ros_fakes import install
install()
from std_msgs.msg import String
import test_execution_mechanism_ros as fixtures
from test_graph_path_bridge_routing import route
from wildos_nav2_adapter.graph_route import cell_cost
from wildos_nav2_adapter.input_quality import valid_grid
from wildos_nav2_adapter.motion_path import diagnose_maneuver_join
from offline_execution_contract_audit import callback_rejections, returned_path_contract, graph_corridor_contract


class Session:
    def __init__(self, carrier='ackermann'):
        self.wall = 1000.
        self.clock_patch = patch('time.monotonic', side_effect=lambda: self.wall)
        self.clock_patch.start()
        self.utc_patch = patch('time.time', side_effect=lambda: 1_790_000_000.+self.wall)
        self.utc_patch.start()
        self.fixture = fixtures.ExecutionMechanismTests()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.node.carrier_model = carrier
        self.node.minimum_turning_radius = 0. if carrier == 'ideal_point' else 8.
        self.node.planner_id = 'GridBased' if carrier == 'ideal_point' else 'CarLike'
        self.node.query = self.node.expected = None
        self.node.motion_queue = [route((0., 0.), (6., 0.))]
        self.responses = []
        def dispatch(goal, **kwargs):
            future = Future()
            self.responses.append(future)
            return future
        self.node.follow_client.send_goal_async.side_effect = dispatch
        self.refresh()

    def refresh(self):
        n = self.node
        stamp = n.get_clock().now().to_msg()
        for message, receive in ((n.odom,n.on_odom), (n.grid,n.on_map),
                                 (n.local_grid,n.on_local_map)):
            fresh = deepcopy(message)
            fresh.header.stamp = stamp
            receive(fresh)
        # Supply fresh messages through the actual input callbacks, not by
        # renewing private receipt ages. No drive input is supplied in a gap.
        n.on_native_status(String(data=json.dumps(dict(wall_time=time.time(), speed=0.,
            telemetry_age_sec=0., actuator_ok=True, telemetry_ok=True,
            control_acknowledged=True, actual={'brake':1.}, sent={'brake':1.}))))
        n.on_control_status(String(data=json.dumps(dict(stamp_ns=self.fixture.now,
            actuating=False, target_speed=0., target_angular=0.))))

    def tick(self, seconds=60):
        trace = []
        for elapsed in range(1, seconds+1):
            self.wall = 1000.+elapsed
            self.fixture.now = int((100.+elapsed)*1e9)
            self.refresh()
            self.node.monitor_inputs()
            if elapsed % 2 == 0:
                self.node.retry_when_planner_starts()
            self.node.retry_graph_reset()
            self.node.monitor_memory_selection()
            if elapsed in (1, 10, 30, 60):
                trace.append(self.snapshot(elapsed))
        return trace

    def accept(self, response=None):
        handle = Mock(accepted=True)
        result = Future()
        handle.get_result_async.return_value = result
        (response or self.responses[-1]).set_result(handle)
        return handle, result

    def snapshot(self, elapsed=0):
        n = self.node
        return dict(elapsed_sec=elapsed, epoch=n.epoch, following=n.following,
                    pending=n.pending, extending=n.extending_path,
                    replacement_pending=n.replacement_pending,
                    deferred_result=n.deferred_follow_result is not None,
                    follow_handle=n.follow_handle is not None,
                    queued_sections=len(n.motion_queue), request_serial=n.follow_request_serial,
                    active_serial=n.active_follow_serial, status=deepcopy(n.last_status),
                    query_reset_pending=n.query_reset_pending, query_reset_inflight=n.query_reset_inflight,
                    input_issue=n.input_freshness_issue(), local_issue=n.local_freshness_issue(),
                    transport_issue=n.execution_transport_issue(),
                    odom_stamp_ns=n.odom.header.stamp.sec*10**9+n.odom.header.stamp.nanosec,
                    map_stamp_ns=n.grid.header.stamp.sec*10**9+n.grid.header.stamp.nanosec,
                    follow_requests=len(self.responses))

    def close(self):
        self.fixture.tearDown()
        self.utc_patch.stop()
        self.clock_patch.stop()


def follow_acceptance_probe(carrier):
    s = Session(carrier)
    try:
        s.node.send_motion_segment()
        assert len(s.responses) == 1
        trace = s.tick()
        blocked = s.snapshot(60)
        s.accept()
        return dict(carrier=carrier, before_late_acceptance=blocked,
                    after_late_acceptance=s.snapshot(60), trace=trace,
                    reproduced=blocked['following'] and not blocked['follow_handle']
                               and blocked['follow_requests'] == 1)
    finally:
        s.close()


def replacement_probe():
    s = Session()
    try:
        n = s.node
        n.send_motion_segment()
        old_handle, old_result = s.accept()
        n.send_motion_segment(replacement=[route((0., 0.), (12., 0.))])
        n.odom.pose.pose.position.x = 6.
        old_result.set_result(S(status=4))
        trace = s.tick()
        blocked = s.snapshot(60)
        s.accept()
        return dict(before_late_acceptance=blocked, after_late_acceptance=s.snapshot(60),
                    trace=trace, reproduced=blocked['replacement_pending'] and blocked['deferred_result']
                    and blocked['following'] and blocked['follow_requests'] == 2)
    finally:
        s.close()


def result_response_probe():
    s = Session()
    try:
        n = s.node
        n.send_motion_segment()
        handle, result = s.accept()
        # Goal geometry is reached, fresh native feedback reports braking;
        # withhold the action result instead of inventing an arrival callback.
        n.odom.pose.pose.position.x = 6.
        trace = s.tick()
        blocked = s.snapshot(60)
        n.start_segment = Mock()
        result.set_result(S(status=4))
        return dict(before_late_result=blocked, after_late_result=s.snapshot(60),
                    trace=trace, reproduced=blocked['following'] and blocked['follow_handle']
                    and blocked['queued_sections'] == 1 and blocked['follow_requests'] == 1)
    finally:
        s.close()


def duplicate_result_probe():
    s = Session()
    try:
        n = s.node
        n.send_motion_segment()
        handle, result = s.accept()
        old_epoch, old_serial = n.epoch, n.active_follow_serial
        n.start_segment = Mock()
        n.odom.pose.pose.position.x = 6.
        result.set_result(S(status=4))
        n.motion_queue = [route((6., 0.), (12., 0.))]
        n.send_motion_segment()
        before = s.snapshot()
        # Inject a repeated callback; no assertion that DDS actually duplicates it.
        n.on_follow_result(result, old_epoch, (6., 0.), old_serial)
        after = s.snapshot()
        return dict(before_duplicate=before, after_duplicate=after,
                    new_acceptance_still_pending=not s.responses[-1].done(),
                    scope='Repeated callback injection, not a recorded DDS duplication',
                    reproduced=before['following'] and before['queued_sections'] == 1
                               and not after['following'] and after['queued_sections'] == 0)
    finally:
        s.close()


def reset_service_probe():
    s = Session()
    try:
        n = s.node
        n.query = 'green residential house'
        n.query_reset_pending = True
        n.graph_reset_client.service_is_ready.return_value = True
        responses = []
        def dispatch(_):
            response = Future(); responses.append(response); return response
        n.graph_reset_client.call_async.side_effect = dispatch
        n.retry_graph_reset()
        trace = s.tick()
        blocked = s.snapshot(60)
        calls = n.graph_reset_client.call_async.call_count
        responses[0].set_result(S(success=True))
        late_first = s.snapshot(60)
        for response in responses[1:]:
            response.set_result(S(success=True))
        return dict(before_late_response=blocked, after_late_response=s.snapshot(60),
                    after_late_first_response=late_first,
                    service_calls=calls, trace=trace,
                    reproduced=blocked['query_reset_pending'] and blocked['query_reset_inflight']
                               and calls == 1)
    finally:
        s.close()


def normal_action_control():
    s = Session()
    try:
        n = s.node
        n.send_motion_segment()
        handle, result = s.accept()
        n.start_segment = Mock()
        n.odom.pose.pose.position.x = 6.
        result.set_result(S(status=4))
        return dict(final=s.snapshot(), handoff_calls=n.start_segment.call_count,
                    passed=not n.following and not n.motion_queue and n.start_segment.call_count == 1)
    finally:
        s.close()


def rotated_map_control():
    s = Session()
    try:
        grid = deepcopy(s.node.grid)
        grid.info.origin.position.x = grid.info.origin.position.y = 10.
        grid.info.origin.orientation.z = math.sin(math.pi/4)
        grid.info.origin.orientation.w = math.cos(math.pi/4)
        grid.data[1] = 100
        # Local cell (1,0) centre transformed through a 90 degree origin.
        return dict(valid=valid_grid(grid, 'spot1/odom'),
                    obstacle_cost=cell_cost(grid, 9.5, 11.5),
                    free_cost=cell_cost(grid, 8.5, 11.5),
                    passed=cell_cost(grid, 9.5, 11.5) == 100 and cell_cost(grid, 8.5, 11.5) == 0)
    finally:
        s.close()


def lookahead_contract():
    examples = []
    total = differing = 0
    headings = [i/80 for i in range(81)]
    points = [(8*math.sin(h), 8*(1-math.cos(h))) for h in headings]
    for x in (-.3, 0., .3):
        for y in (-.3, 0., .3):
            for heading in (-.3, -.15, 0., .15, .3):
                three, a = diagnose_maneuver_join(points, headings, (x, y), heading, 8., lookahead=3.)
                six, b = diagnose_maneuver_join(points, headings, (x, y), heading, 8., lookahead=6.)
                total += 1
                if (three is None) != (six is None):
                    differing += 1
                    if len(examples) < 4:
                        examples.append(dict(position=[x,y], heading=heading,
                                             three_m_accepted=three is not None, three_m_reason=a,
                                             six_m_accepted=six is not None, six_m_reason=b))
    return dict(samples=total, differing_classifications=differing, examples=examples,
                scope='Same Python predictor at 3/6m; not C++ controller or full trajectory replay')


def candidate_fallback_probe():
    import test_short_endpoint_handoff_ros as handoff
    fixture = handoff.ShortEndpointHandoffTests()
    fixture.setUp()
    n, case = fixture.node, fixture.case
    try:
        sample = case['observations'][0]
        start, heading = sample['position'], sample['heading']
        end = (start[0]+6.*math.cos(heading), start[1]+6.*math.sin(heading))
        n.odom.pose.pose.position.x, n.odom.pose.pose.position.y = start
        n.odom.pose.pose.orientation.z = math.sin(heading/2)
        n.odom.pose.pose.orientation.w = math.cos(heading/2)
        n.route = n.latest_route = route(start, end)
        n.route_endpoint = end
        n.candidates = [(*end, case['headings'][-1], 6.), (*end, heading, 6.)]
        n.candidate_index = 0
        first = route(*case['points'])
        for p, h in zip(first.poses, case['headings']):
            p.pose.orientation.z, p.pose.orientation.w = math.sin(h/2), math.cos(h/2)
        second = route(case['points'][-1], end)
        for p in second.poses:
            p.pose.orientation.z, p.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        n.motion_queue = [first, second]
        n.try_candidate = Mock()
        n.send_motion_segment()
        rejected = dict(status=deepcopy(n.last_status), candidate_index=n.candidate_index,
                        next_candidate_calls=n.try_candidate.call_count,
                        actions=n.follow_client.send_goal_async.call_count)
        # Independently restore the same physical pose/maps and test a supplied
        # straight alternative. This does not claim Smac generated it.
        n.following = n.pending = False
        n.blocked_endpoint = None
        alternative = route(start, end)
        for p in alternative.poses:
            p.pose.orientation.z, p.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        n.motion_queue = [alternative]
        n.send_motion_segment()
        return dict(first_rejection=rejected,
                    alternative_actions=n.follow_client.send_goal_async.call_count,
                    alternative_status=deepcopy(n.last_status),
                    scope='Supplied second path, same pose/free maps, not an actual Smac alternative',
                    reproduced=rejected['actions'] == 0 and rejected['next_candidate_calls'] == 0
                    and n.follow_client.send_goal_async.call_count == 1)
    finally:
        fixture.tearDown()


def sources():
    paths = []
    for directory in ('nav2_adapter/wildos_nav2_adapter', 'nav2_adapter/config',
                      'nav2_adapter/launch', 'official_adapter/beamng_wildos_adapter',
                      'official_adapter/config', 'official_adapter/launch',
                      'ackermann_rpp', 'ackermann_smac',
                      'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation',
                      'wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper/src',
                      'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/src'):
        paths.extend(p for p in (ROOT/directory).rglob('*') if p.is_file()
                     and (p.suffix in ('.py','.cpp','.hpp','.yaml','.xml','.lua')
                          or p.name == 'CMakeLists.txt')
                     and '__pycache__' not in p.parts and 'test' not in p.parts)
    paths.extend(ROOT/p for p in ('tools/beamng_extensions/wildosKinematic.lua',
                                  'tools/attach_current_vehicle_sensors.py',
                                  'config/sbr_physical_sensor_rig.json',
                                  'nav2_adapter/setup.py', 'official_adapter/setup.py'))
    return {str(p.relative_to(ROOT)).replace('\\','/'): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(set(paths))}


def known_callback_probe():
    # The historical helper installs its own ROS doubles; isolate that install
    # rather than bypassing its protection against a real ROS process.
    code = ("import sys,json; sys.path.insert(0," + repr(str(ROOT/'tools')) + "); "
            "from offline_execution_contract_audit import callback_rejections; "
            "print(json.dumps(callback_rejections()))")
    result = subprocess.run([sys.executable, '-c', code], cwd=ROOT,
                            capture_output=True, text=True, encoding='utf8', errors='replace', timeout=30)
    if result.returncode:
        raise RuntimeError(result.stderr)
    return json.loads(result.stdout)


def audit():
    before = sources()
    report = dict(scope='Frozen production source, real Python callbacks with ROS doubles; no server/physics',
        follow_acceptance=[follow_acceptance_probe(c) for c in ('ackermann','ideal_point')],
        replacement_acceptance=replacement_probe(), duplicate_follow_result=duplicate_result_probe(),
        follow_result_response=result_response_probe(),
        graph_reset_response=reset_service_probe(), normal_action=normal_action_control(),
        rotated_map=rotated_map_control(), lookahead=lookahead_contract(),
        candidate_fallback=candidate_fallback_probe(),
        known_callback_contracts=known_callback_probe(), returned_path=returned_path_contract(),
        terrain_corridor=graph_corridor_contract(), sources=before)
    after = sources()
    report['production_unchanged'] = before == after
    if before != after:
        raise RuntimeError('Production changed during audit')
    return report


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    report = audit()
    (OUT/'audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in report.items() if k != 'sources'},ensure_ascii=False,indent=2))
