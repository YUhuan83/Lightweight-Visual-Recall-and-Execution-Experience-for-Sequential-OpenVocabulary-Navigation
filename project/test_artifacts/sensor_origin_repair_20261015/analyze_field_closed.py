import json,sqlite3,hashlib,zipfile,math,collections,re
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=Path('/home/ly/wildos-console');sessions=[r/'audit_results/session_20261015_203918']
def analyze_observer(path,events):
 latest_odom=None;ages=[];stale_fresh=0;counts=collections.Counter();last={};failures=[]
 def stamp(h):return h['stamp']['sec']+h['stamp']['nanosec']/1e9
 targets=[e for e in events if e.get('kind')=='plan_status' and e.get('status')=='follow_failed']
 for line in path.open():
  z=json.loads(line);t=z['topic'];msg=z['message'];now=z['received_ros_ns']/1e9
  counts[t]+=1;last[t]=z
  if t=='/spot1/odom':latest_odom=stamp(msg['header'])
  if t=='/received_global_plan':
   age=now-stamp(msg['header']);ages.append(age)
   if age>.5 and latest_odom and now-latest_odom<.3:stale_fresh+=1
  while targets and now>=targets[0]['stamp_ns']/1e9:
   event=targets.pop(0);snap={}
   for topic in ('/spot1/odom','/received_global_plan','/lookahead_point','/wildos/planned_path','/wildos/cmd_vel'):
    v=last.get(topic)
    if v:snap[topic]={'received_age_sec':now-v['received_ros_ns']/1e9,'message':v['message']}
   failures.append({'event':event,'snapshot':snap})
 stats=lambda a:dict(count=len(a),median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(max(a))) if a else None
 return {'topic_counts':dict(counts),'controller_pose_source_age_sec':stats(ages),'stale_with_fresh_external_odom':stale_fresh,'failure_snapshots':failures,'unmatched_failure_count':len(targets)}

reports=[];files=set()
for s in sessions:
 assert (s/'session_result.json').exists() and (s/'goal_001/result.json').exists(),s
 a=[json.loads(l) for l in (s/'vision_participation.jsonl').read_text().splitlines()];x=[z for z in a if z['kind']=='scores'];final=next(z for z in reversed(a) if z['kind']=='final')
 g=s/'goal_001';events=[json.loads(l) for l in (g/'events.jsonl').read_text().splitlines()]
 native=[json.loads(l) for l in (g/'native_samples.jsonl').read_text().splitlines()];points=[v['position'] for v in native if isinstance(v.get('position'),list)]
 timing=[json.loads(l) for l in (g/'sensor_timing.jsonl').read_text().splitlines()]
 ages={}
 def stats(v):return dict(count=len(v),median=float(np.median(v)),p95=float(np.percentile(v,95)),max=float(max(v)))
 for t in ('/spot1/odom','/spot1/realsense/front/color/image_raw'):
  v=[v['ros_header_age_ns']/1e9 for v in timing if v['topic']==t and v.get('ros_header_age_ns') is not None];ages[t]=stats(v) if v else None
 c=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
 topic_counts={n:cnt for n,cnt in c.execute('select topics.name,count(messages.id) from topics left join messages on topics.id=messages.topic_id group by topics.id')}
 tid,typ=c.execute("select id,type from topics where name='/spot1/graphnav_planner/route_status'").fetchone();kind=get_message(typ)
 choices=collections.Counter()
 for ts,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
  z=json.loads(deserialize_message(b,kind).data);goal=z.get('goal');recent=[v for v in x if 0<=ts/1e9-v['wall']<3.]
  hits=[n for v in reversed(recent) for n in v['candidates'] if goal and math.dist(n['xy'],goal[:2])<1e-5]
  choices['visual' if hits and hits[0].get('default')==[0.] else 'fallback' if hits else 'unmatched']+=1
 raw_visual=[n['max_score'] for v in x for n in v['candidates'] if n.get('default')==[0.]]
 observer=analyze_observer(s/'execution_observer/messages.jsonl',events)
 connectors=[tuple(map(int,v)) for v in re.findall(r'continuous_start: evaluated=(\d+) candidates=(\d+) timeout=(\d+)',(s/'stack.log').read_text())]
 report=dict(connector_search={'calls':len(connectors),'zero_candidate_calls':sum(v[1]==0 for v in connectors),'timeouts':sum(v[2] for v in connectors)},controller_observer=observer,session=str(s),final=final,observed_score_snapshots=len(x),max_nondefault_nodes=max(v['nondefault_nodes'] for v in x),max_memory_utility_delta=max(v['max_memory_utility_delta'] for v in x),
   visual_score_maxima=stats(raw_visual) if raw_visual else None,selected_goal_source_counts=dict(choices),source_age_sec=ages,
   model_snapshot_age_sec=stats([v['wall']-v['stamp_ns']/1e9 for v in x]),
   native_distance_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:])),status_counts=dict(collections.Counter(v.get('status') for v in events if v['kind']=='plan_status')),
   topic_counts=topic_counts,result=json.loads((g/'result.json').read_text()),session_result=json.loads((s/'session_result.json').read_text()))
 (s/'repair_review.json').write_text(json.dumps(report,indent=2));reports.append(report);c.close()
 files.update(p for p in s.rglob('*') if p.is_file())
for p in r.glob('field_sensor_full_search_*'):
 if (p/'result.json').exists() and json.loads((p/'result.json').read_text()).get('session') in {str(s) for s in sessions}:files.update(x for x in p.rglob('*') if x.is_file())
for name in ('source_identity.json','source_identity_042ee7af.json','source_identity_bf3ee70d.json','score_memory_build.log','score_memory_build_final.log','score_memory_build_clock.log','score_memory_gate_passed.json','audit_console.json'):
 if (r/name).is_file():files.add(r/name)
files.update(p for p in Path('/home/ly/full_score_search_20261015').glob('*') if p.is_file())
files.update(p for p in Path('/home/ly/sensor_origin_repair_20261015').glob('*') if p.is_file())
out=r/'sensor_origin_field_20261015_203918.zip';m={}
with zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED) as z:
 for p in sorted(files):
  b=p.read_bytes();n=str(p.relative_to(r)) if p.is_relative_to(r) else 'diagnostics/'+p.parent.name+'/'+p.name;m[n]=hashlib.sha256(b).hexdigest();z.writestr(n,b)
 z.writestr('MANIFEST.json',json.dumps(m,indent=2))
print(json.dumps(dict(reports=reports,archive=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),files=len(m)),indent=2))
