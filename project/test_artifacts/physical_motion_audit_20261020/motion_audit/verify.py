"""Check closed-trial identity, native/bag wall-clock alignment and numerical consistency."""
import pathlib,json,sqlite3,hashlib,math
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
root=pathlib.Path(__file__).resolve().parents[1]
p=root/'motion_audit'
r=json.loads((p/'physical_motion_review.json').read_text());s=json.loads((p/'source_timing.json').read_text())
trial=pathlib.Path(r['trial']);driver=json.loads((trial/'result.json').read_text());assert driver['closed']
bag=pathlib.Path(driver['session'])/'goal_001/interfaces/interfaces_0.db3'
manifest=json.loads((root/'recording_delivery_manifest.json').read_text())
h=hashlib.sha256()
with bag.open('rb') as f:
    while b:=f.read(4*1024*1024):h.update(b)
assert h.hexdigest()==manifest['source_bag']['sha256']
weights=r['overall']['duration_weights']
assert math.isclose(weights['moving_sec']+weights['speed_below_0.1_sec'],weights['observed_sec'],abs_tol=1e-6)
assert math.isclose(sum(m['duration_weights']['observed_sec'] for m in r['minutes']),weights['observed_sec'],abs_tol=1e-6)
assert weights['stopped_actual_brake_sec']<=weights['speed_below_0.1_sec']<=weights['observed_sec']
longest=max(r['stationary_intervals'],key=lambda v:v['range_sec'][1]-v['range_sec'][0])
assert longest['range_sec'][1]-longest['range_sec'][0]>51 and longest['net_m']<.006
assert s['native_actuator_bad']==0 and s['native_lease_tripped']==0
c=sqlite3.connect('file:'+str(bag)+'?mode=ro',uri=True)
ids=dict(c.execute('select name,id from topics'))
align=[]
for t,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(ids['/wildos/native_control_status'],)):
    v=json.loads(deserialize_message(b,String).data);align.append(t/1e9-v['wall_time'])
assert min(align)>=0 and max(align)<.2
issuers=set();lifetimes=[]
for t,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(ids['/wildos/native_control_command'],)):
    v=json.loads(deserialize_message(b,String).data);issuers.add(v['sender_id']);lifetimes.append(v['expires_wall_ns']-v['issued_wall_ns'])
assert len(issuers)==1 and all(0<x<=500000000 for x in lifetimes)
s['scope']='Separate recorded packet latency, receipt lifetime and SDK call timing; no guard changes.'
s.pop('clock_alignment_sample',None)
s['native_wall_vs_bag_receipt_sec']={'count':len(align),'min':min(align),'max':max(align)}
s['lifetime_ns_exact']={'min':min(lifetimes),'max':max(lifetimes)}
s['interpretation_limit']='First native receipt appears after application, not an exact receive timestamp. Callback delay is inferred from paired Native command_receipt_age; it does not distinguish DDS delay from callback scheduling. Unobserved command sequence is not proof of command loss.'
(p/'source_timing.json').write_text(json.dumps(s,indent=2)+'\n')
report=dict(scope=__doc__,passed=True,bag_sha256=h.hexdigest(),native_samples=r['native_samples'],source_sender_count=len(issuers),
 near_stationary_percent=100*weights['speed_below_0.1_sec']/weights['observed_sec'],
 bag_receive_after_native_wall_sec=s['native_wall_vs_bag_receipt_sec'],
 source_lifetime_ns=s['lifetime_ns_exact'],complete_navigation_passed=False,production_changes=False,
 files={f.name:dict(bytes=f.stat().st_size,sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in p.iterdir() if f.is_file() and f.name!='verification.json'})
(p/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='files'},indent=2))
