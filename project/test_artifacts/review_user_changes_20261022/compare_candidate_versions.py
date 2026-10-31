"""Run frozen v7/v8 candidate callbacks twice on identical recorded inputs."""
from pathlib import Path
from types import MethodType
from unittest.mock import Mock
import ast
import json
import math
import sys

root=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(root/'nav2_adapter'),str(root/'nav2_adapter/test'),str(root/'official_adapter')]
from offline_ros_fakes import install
install()
from test_graph_path_bridge_routing import GraphBridgeRoutingTests,route
from wildos_nav2_adapter import graph_path_bridge as bridge
from wildos_nav2_adapter.input_quality import stamp_ns
base=Path(__file__).parent
tree=ast.parse((base/'candidate_span_before/nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py').read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='GraphPathBridge')
method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='start_segment')
namespace=dict(bridge.__dict__)
exec(compile(ast.Module(body=[method],type_ignores=[]),'<frozen-v7-start-segment>','exec'),namespace)
before=namespace['start_segment']
report=[]
directory=base/'v8_trial_20261023_085412/route_recovery_audit'
for snapshot in json.loads((directory/'snapshots.json').read_text()):
    grid=json.loads((directory/('public_grid_%s.json'%snapshot['elapsed'])).read_text())
    for repeat in (1,2):
        result=dict(elapsed=snapshot['elapsed'],repeat=repeat)
        for name,method in [('v7',before),('v8',bridge.GraphPathBridge.start_segment)]:
            fixture=GraphBridgeRoutingTests();fixture.setUp();node=fixture.node
            try:
                node.odom.pose.pose.position.x,node.odom.pose.pose.position.y,heading=snapshot['rear_axle']
                node.odom.pose.pose.orientation.z,node.odom.pose.pose.orientation.w=math.sin(heading/2),math.cos(heading/2)
                node.grid.info.width,node.grid.info.height=grid['width'],grid['height']
                node.grid.info.resolution=grid['resolution']
                node.grid.info.origin.position.x,node.grid.info.origin.position.y=grid['origin'][:2]
                node.grid.data=grid['data']
                node.route=route(*[p[:2] for p in snapshot['route']])
                node.route_endpoint=snapshot['route'][-1][:2]
                node.route_complete=snapshot['metadata']['complete']
                node.route_metadata[stamp_ns(node.route)]=snapshot['metadata']
                node.input_freshness_issue=Mock(return_value=None)
                node.execution_transport_issue=Mock(return_value=None)
                node.maybe_side_lookback=Mock(return_value=False)
                node.begin_exploration_comparison=Mock()  # Inspect its retained normal candidate set.
                node.try_candidate=Mock()
                MethodType(method,node)()
                result[name]=node.candidates[:6]
            finally:node.destroy_node()
        result['first_three_same']=result['v7'][:3]==result['v8'][:3]
        report.append(result)
(base/'v7_v8_recorded_candidate_ab.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
