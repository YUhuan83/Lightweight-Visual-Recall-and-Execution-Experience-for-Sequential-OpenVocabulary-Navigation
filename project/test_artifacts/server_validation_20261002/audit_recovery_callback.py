"""Exercise the actual recovery callback with ROS interface test doubles."""
from pathlib import Path
import json
import sys
import time
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'nav2_adapter'),str(ROOT/'nav2_adapter/test')]
from offline_ros_fakes import install
install()
from test_graph_path_bridge_routing import GraphBridgeRoutingTests,route

GraphBridgeRoutingTests.setUpClass()
case=GraphBridgeRoutingTests();case.setUp();node=case.node
try:
    node.query='gas station';node.memory_goal=node.expected=(15.,0.)
    node.route=route((0.,0.),(15.,0.));node.route_endpoint=(15.,0.)
    node.follow_enabled=True;node.follow_client=Mock();node.plan_client=Mock()
    node.follow_failed('motion_section_invalid',detail='excess_curvature')
    node.blocked_until=time.monotonic()-1;node.grid.data[-1]=100
    node.start_segment=Mock()
    node.retry_when_planner_starts()
    timer_retries=node.start_segment.call_count
    node.on_route(route((0.,0.),(-28.,0.)))
    wrong_destination_retries=node.start_segment.call_count
    node.on_route(route((0.,0.),(15.,0.)))
    valid_destination_retries=node.start_segment.call_count
    assert (timer_retries,wrong_destination_retries,valid_destination_retries)==(1,1,2)
    result={'scope':'Actual production callbacks with ROS test doubles; no DDS or vehicle',
        'backoff_expired':True,'map_changed':True,'timer_retries':timer_retries,
        'wrong_destination_retries':wrong_destination_retries,'valid_destination_retries':valid_destination_retries,
        'memory_goal_retained':node.memory_goal== (15.,0.)}
    out=Path(__file__).resolve().parent/'navigation_repair/recovery_callback.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
finally:
    case.tearDown();GraphBridgeRoutingTests.tearDownClass()
