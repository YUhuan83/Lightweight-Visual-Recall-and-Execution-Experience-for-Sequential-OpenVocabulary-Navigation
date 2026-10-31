"""Reproduce production state handoff only; no ROS, map or physics acceptance."""
import ast,copy,json,math
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
root=Path(__file__).resolve().parents[2]
source=root/'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
tree=ast.parse(source.read_text(encoding='utf-8'))
method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='execute_checked_path')
ns=dict(xy=lambda p:(p.position.x,p.position.y),yaw=lambda p:0.,motion_sections=lambda _:[(0,1),(1,2)],deepcopy=copy.deepcopy,math=math)
exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)
results=[]
for active in (True,False):
 path=S(poses=[S(pose=S(position=S(x=x,y=0.))) for x in (0.,2.,0.)])
 n=S(pending=True,extending_path=True,following=active,follow_enabled=True,prepared_extension=None,carrier_model='ackermann',grid=None,odom=S(pose=S(pose=path.poses[0].pose)),input_freshness_issue=Mock(return_value=None),validate_path=Mock(return_value=None),initial_tracking_issue=Mock(return_value=None),current_route_points=Mock(return_value=[(0.,0.),(10.,0.)]),send_motion_segment=Mock(),extension_failed=Mock())
 ns['execute_checked_path'](n,path,(0.,0.))
 results.append(dict(old_action_active=active,pending=n.pending,following=n.following,extending_path=n.extending_path,prepared_extension=n.prepared_extension is not None,send_calls=n.send_motion_segment.call_count))
assert results[0]['following'] and results[0]['prepared_extension']
assert not results[1]['following'] and not results[1]['pending'] and results[1]['prepared_extension'] and results[1]['send_calls']==0
out=dict(scope='Exact production method, synthetic validated two-section path; validation mocked to isolate ownership. Current faulty behavior reproduced, not a passing repair regression.',cases=results,reproduced_idle_prepared_extension=True)
Path(__file__).with_suffix('.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2))
