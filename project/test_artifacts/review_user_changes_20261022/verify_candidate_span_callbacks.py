"""Replay recorded routing inputs through production callbacks with ROS doubles."""
from pathlib import Path
import json
import math
import sys
from unittest.mock import Mock

root = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(root/'nav2_adapter'), str(root/'nav2_adapter/test'), str(root/'official_adapter')]
from offline_ros_fakes import install
install()
from test_graph_path_bridge_routing import GraphBridgeRoutingTests, route
from wildos_nav2_adapter.graph_path_bridge import yaw
from wildos_nav2_adapter.motion_path import motion_sections
from wildos_nav2_adapter.input_quality import stamp_ns

base = Path(__file__).parent
audit = base/'v7_trial_20261022_223039/route_recovery_audit'
snapshot = next(e for e in json.loads((audit/'snapshots.json').read_text()) if e['elapsed']==88.35)
recorded = json.loads((audit/'public_grid_88.35.json').read_text())
fixture = GraphBridgeRoutingTests()
fixture.setUp()
node = fixture.node
try:
    node.odom.pose.pose.position.x, node.odom.pose.pose.position.y, angle = snapshot['rear_axle']
    node.odom.pose.pose.orientation.z, node.odom.pose.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
    grid = node.grid
    grid.info.width, grid.info.height = recorded['width'], recorded['height']
    grid.info.resolution = recorded['resolution']
    grid.info.origin.position.x, grid.info.origin.position.y = recorded['origin'][:2]
    grid.data = recorded['data']
    node.route = route(*[p[:2] for p in snapshot['route']])
    node.route_complete = False
    node.route_endpoint = snapshot['route'][-1][:2]
    node.route_metadata[stamp_ns(node.route)] = snapshot['metadata']
    node.input_freshness_issue = Mock(return_value=None)
    node.execution_transport_issue = Mock(return_value=None)
    node.maybe_side_lookback = Mock(return_value=False)
    node.try_candidate = Mock()
    node.start_segment()
    actual = [tuple(map(float,s.split())) for s in (base/'candidate_span.txt.path_27').read_text().splitlines()]
    points, headings = [p[:2] for p in actual], [p[2] for p in actual]
    path = route(*points)
    for message, heading in zip(path.poses,headings):
        message.pose.orientation.z, message.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
    result = dict(scope='Production callback doubles + recorded public map; no DDS/vehicle/private-map replay',
        first_four_candidates=node.candidates[:4],
        tested_short_target=points[-1],
        sampled_path_length_m=sum(math.dist(a,b) for a,b in zip(points,points[1:])),
        motion_sections=len(motion_sections(points)),
        whole_path_issue=node.validate_path(points,headings,[snapshot['rear_axle'][:2]]+node.current_route_points(),grid,points[-1],start=snapshot['rear_axle'][:2]),
        section_limit_issue=node.exploration_motion_issue(points),
        tracking_issue=node.initial_tracking_issue(points,headings))
    (base/'candidate_span_callback_replay.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
finally:
    node.destroy_node()
