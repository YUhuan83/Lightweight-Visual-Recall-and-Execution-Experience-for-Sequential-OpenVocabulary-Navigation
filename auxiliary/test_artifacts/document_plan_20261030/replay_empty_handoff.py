"""Replay the recorded v73 empty-path instant using the completed step-2 candidate entry."""
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022/v73_document_plan')
root=base.with_name('v74_document_handoff')
s=(base/'probe_v67_forward.py').read_text()
s=s.replace(f"root=Path('{base}')",f"root=Path('{root}')")
s=s.replace("trial=parent/'v67_target_road_120s/full/trial_20261029_212751'",f"trial=Path('{base}/full/trial_20261030_184621')")
s=s.replace("early=json.loads((root/'v67_local_ahead_replay.json').read_text())['runs'][0]['first_early'];assert early",'early={}')
s=s.replace("stamp=origin+round(early['q']*1e9)",'''import re
from decimal import Decimal
stack=next(trial.glob('sessions/*/stack.log')).read_text()
failure=re.search(r'\\[(\\d{10}\\.\\d+)\\].*ComputePath candidate failed: epoch=9 candidate=0 reason=empty_path',stack)
assert failure
stamp=int(Decimal(failure.group(1))*10**9);early['q']=(stamp-origin)*1e-9''')
start=s.index('confirmation=next(');end=s.index('n=S(',start)
s=s[:start]+'''_,raw=last('/spot1/raw_object_waypoint')
pos=raw.pose.position;target=[pos.x,pos.y,pos.z]
pos=odom.pose.pose.position;q=odom.pose.pose.orientation
heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
start=[pos.x,pos.y,heading]
''' +s[end:]
# The connection was closed after loading maps in the original script; the
# raw lookup must use the same already-closed recording before closing it.
s=s.replace("ot,odom=last('/spot1/odom');db.close()", "ot,odom=last('/spot1/odom')")
s=s.replace("pos=raw.pose.position;target=", "db.close()\npos=raw.pose.position;target=")
s=s.replace('n=S(upstream_target_approach=True',"n=S(carrier_model='ackermann',upstream_target_approach=True")
start=s.index("cpp=(parent/'probe_vehicle_observation.cpp')");end=s.index('env=os.environ.copy()',start)
s=s[:start]+f"out=root/'empty_path_smac_paths';out.mkdir(exist_ok=True);exe=Path('{base}/v67_smac_paths/probe')\n"+s[end:]
s=s.replace('v67_forward_probe.txt','empty_forward_probe.txt')
s=s.replace('v67_smac_forward_replay.json','empty_path_forward_replay.json')
exec(compile(s,'recorded_empty_path_forward_probe','exec'))
