import ast,collections,hashlib,json,math,sys,time
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
base=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\depth_geometry_20261027')
root=base/'candidate';field=base/'field'
sys.path.insert(0,str(root/'official_adapter'))
from beamng_wildos_adapter.target_selection import required_cluster_support,supported_position_clusters,select_target_cluster
source=root/'official_adapter/beamng_wildos_adapter/goal_standoff.py'
t=ast.parse(source.read_text(encoding='utf8'));c=next(n for n in t.body if isinstance(n,ast.ClassDef) and n.name=='GoalStandoff')
c.bases=[];c.body=[n for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='on_localization']
def pose():return S(header=S(stamp=S()),pose=S(position=S()))
e=dict(np=np,json=json,math=math,time=time,deque=collections.deque,PoseStamped=pose,
 required_cluster_support=required_cluster_support,supported_position_clusters=supported_position_clusters,select_target_cluster=select_target_cluster)
exec(compile(ast.fix_missing_locations(ast.Module(body=[c],type_ignores=[])),str(source),'exec'),e)
r=json.loads((field/'field_depth_replay_1.json').read_text(encoding='utf8'))['rounds'][0]
actual=[p['data'] for p in r['recorded_points']]
def replay(points):
 n=e['GoalStandoff']();n.active_query='green residential house';n.query_started_ns=n.last_localization_ns=0
 n.confirmed_instance_anchor=None;n.navigation_region_backoff=collections.deque(maxlen=16)
 n.observations=[];n.replacement_observations=collections.deque(maxlen=10);n.query_origin=None
 n.confirmation_window_sec=60.;n.get_parameter=lambda name:S(value={'confirmation_frames':5,'confirmation_radius':1.5}[name])
 confirmations=[];n.retry_confirmation=lambda **k:confirmations.append(n.pending_confirmation[1])
 history=[]
 for p in points:
  stamp=p['observation_stamp_ns'];n.get_clock=lambda stamp=stamp:S(now=lambda:S(nanoseconds=stamp))
  n.on_localization(S(data=json.dumps(p)))
  values=np.asarray([o['position'] for o in n.observations]);counts=[0]
  if len(values):
   for seed in values:
    counts.append(sum(np.linalg.norm(values[:,:2]-seed[:2],axis=1)<=1.5))
  history.append(dict(stamp_ns=stamp,observations=len(n.observations),max_radius_support=int(max(counts))))
 return dict(history=history,confirmations=confirmations)
a=replay(actual);assert not a['confirmations'];assert max(h['max_radius_support'] for h in a['history'])==3
# Positive control: four distinct fresh observations of the same supported point.
import copy
control=[copy.deepcopy(actual[-1]) for _ in range(4)]
for i,p in enumerate(control):p['observation_stamp_ns']+=i*100_000_000
b=replay(control);assert len(b['confirmations'])==1,b
old=json.loads((field/'field_depth_old_geometry_1.json').read_text(encoding='utf8'))['rounds'][0]
oldrows={p['stamp_ns']:p for p in old['rows']};newrows=r['rows']
restored=[dict(stamp_ns=p['stamp_ns'],position=p['position'],surface=[t for t in p['trace'] if t['function']=='_surface_quality']) for p in newrows if p['result']=='lidar' and oldrows[p['stamp_ns']]['result']!='lidar']
assert len(restored)==2
for f in ('field_depth_replay','field_depth_old_geometry','displacement_audit'):
 assert (field/(f+'_1.json')).read_bytes()==(field/(f+'_2.json')).read_bytes(),f
out=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),scope='Actual production on_localization method, recorded valid messages; zero transport age gives maximum possible support. No semantic truth asserted.',
 query='green residential house',confirmation_window_sec=60,required_support=4,radius_m=1.5,actual=a,positive_control_confirmed=len(b['confirmations']),
 same_recording_before=old['results'],same_recording_after=r['results'],newly_restored=restored)
encoded=json.dumps(out,indent=2,sort_keys=True).encode();(field/sys.argv[1]).write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),max_support=3,required=4,old=old['results'],new=r['results'],positive_control=1)))

