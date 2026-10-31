"""State-speed direction loss counterexample; no vehicle execution."""
from pathlib import Path
from types import SimpleNamespace as S
import ast,math,json,sys
r=Path(__file__).resolve().parents[2];sys.path.insert(0,str(r/'nav2_adapter'))
from wildos_nav2_adapter.drive_math import drive_outputs
source=r/'nav2_adapter/wildos_nav2_adapter/cmd_vel_to_control.py'
tree=ast.parse(source.read_text(encoding='utf-8'));method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='on_state')
ns=dict(math=math,StateSensor=object);exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)
rows=[]
for velocity in (1.2,-1.2):
 node=S(last_source_state_ns=0,_safety_clock=S(now=lambda:S(nanoseconds=10**9)))
 msg=S(header=S(stamp=S(sec=1,nanosec=0),frame_id='world'),velocity=S(x=velocity,y=0.,z=0.))
 ns['on_state'](node,msg)
 gear,throttle,brake=drive_outputs(-1.2,node.speed,.2,2,-1,True,current_gear=-1)
 rows.append(dict(world_velocity_x=velocity,body_heading_assumed_positive_world_x=True,measured_speed_used_by_loop=node.speed,requested_speed=-1.2,gear=gear,throttle=throttle,brake=brake))
assert rows[0]['measured_speed_used_by_loop']==rows[1]['measured_speed_used_by_loop']
assert rows[0]['throttle']==rows[0]['brake']==0.
out=dict(scope='Exact on_state callback AST and production drive_outputs, synthetic opposite velocity samples; direction-loss reproduction only, not all field failures explained',cases=rows,direction_discarded=True)
Path(__file__).with_suffix('.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps(out,indent=2))
