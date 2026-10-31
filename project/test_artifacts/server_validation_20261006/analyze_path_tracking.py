import json, math, statistics
from collections import defaultdict
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root='/home/ly/town_memory_validation_20261003/green_house_station_arrival_20261006_idealpoint_syncfix1'
bag=root+'/rosbag'; event_path=root+'/events.jsonl'
events=[json.loads(x) for x in open(event_path,encoding='utf-8')]
status_events=[x for x in events if x.get('kind')=='plan_status' and isinstance(x.get('stamp_ns'),int)]
paths=[x for x in events if x.get('kind')=='planned_path' and x.get('phase')=='green_house_search']
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
types={x.name:x.type for x in r.get_all_topics_and_types()}
rows=[]; bag_status={}; bag_status_records=[]
while r.has_next():
 topic,data,recv=r.read_next()
 if topic=='/spot1/odom':
  m=deserialize_message(data,get_message(types[topic])); p=m.pose.pose.position; q=m.pose.pose.orientation
  yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  rows.append((recv,p.x,p.y,yaw))
 elif topic=='/wildos/plan_status':
  m=deserialize_message(data,get_message(types[topic]))
  try: v=json.loads(m.data)
  except: continue
  sn=v.get('stamp_ns')
  if isinstance(sn,int):
   if sn not in bag_status: bag_status[sn]=recv
   bag_status_records.append({'stamp_ns':sn,'recv':recv,'data':v})
if not rows: raise RuntimeError('No odometry in bag')
# Associate paths with nearest planned transition that has the same endpoint.
path_records=[]
for p in paths:
 pts=p.get('points') or []
 if len(pts)<2: continue
 end=pts[-1]
 candidates=[]
 for s in status_events:
  if s.get('status')!='planned' or not isinstance(s.get('goal'),list) or len(s['goal'])<2: continue
  if math.dist(end,s['goal'][:2])<.25:
   candidates.append(s)
 if not candidates: continue
 st=min(candidates,key=lambda s:abs(float(s['elapsed'])-float(p['elapsed'])))
 if abs(float(st['elapsed'])-float(p['elapsed']))>1.0: continue
 followings=[s for s in status_events if s.get('status')=='following'
             and isinstance(s.get('goal'),list) and math.dist(end,s['goal'][:2])<.25
             and float(s['elapsed'])>=float(st['elapsed'])
             and float(s['elapsed'])-float(p['elapsed'])<1.0]
 following=min(followings,key=lambda s:abs(float(s['elapsed'])-float(p['elapsed']))) if followings else st
 recv=bag_status.get(following['stamp_ns'])
 if recv is None: continue
 path_records.append({'elapsed':float(p['elapsed']),'recv':recv,'points':pts,'length':float(p.get('length',0)),
                      'goal':end,'status_stamp_ns':following['stamp_ns'],'terminal':None})
path_records.sort(key=lambda x:x['recv'])
for i,pr in enumerate(path_records):
 t1=path_records[i+1]['recv'] if i+1<len(path_records) else rows[-1][0]
 terminal=[s for s in bag_status_records if pr['recv']<s['recv']<t1
           and s['data'].get('status') in ('follow_failed','segment_done','recovery_failed')]
 if terminal:
  first=min(terminal,key=lambda s:s['recv'])
  pr['end_recv']=first['recv']; pr['terminal']=first['data'].get('reason') or first['data'].get('status')
 else:
  pr['end_recv']=t1
def project(point,pts):
 best=(float('inf'),0.0)
 dist0=0.0
 for a,b in zip(pts,pts[1:]):
  dx=b[0]-a[0];dy=b[1]-a[1];l2=dx*dx+dy*dy;seg=math.sqrt(l2)
  t=0 if l2<1e-9 else max(0,min(1,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/l2))
  ex=a[0]+t*dx;ey=a[1]+t*dy;err=math.hypot(point[0]-ex,point[1]-ey)
  if err<best[0]: best=(err,dist0+t*seg)
  dist0+=seg
 return best
def pct(vals,p):
 z=sorted(vals); return z[min(len(z)-1,int(p*(len(z)-1)))] if z else None
metrics=[]
for i,pr in enumerate(path_records):
 t0=pr['recv']; t1=pr['end_recv']
 active=[x for x in rows if t0<=x[0]<t1]
 if len(active)<2: continue
 pts=pr['points']; projections=[project((x[1],x[2]),pts) for x in active]
 travel=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(active,active[1:]))
 net=math.hypot(active[-1][1]-active[0][1],active[-1][2]-active[0][2])
 progress=projections[-1][1]-projections[0][1]
 turn=sum(abs(math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3]))) for a,b in zip(active,active[1:]))*180/math.pi
 metrics.append({'elapsed':round(pr['elapsed'],2),'duration':round((t1-t0)/1e9,2),'goal':[round(x,2) for x in pr['goal']],
  'path_len':round(pr['length'],2),'odom_samples':len(active),'travel':round(travel,2),'net':round(net,2),'path_progress':round(progress,2),
  'cross_track_p50':round(pct([x[0] for x in projections],.5),3),'cross_track_p95':round(pct([x[0] for x in projections],.95),3),
  'cross_track_max':round(max(x[0] for x in projections),3),'turn_deg':round(turn,1),'start_xy':[round(active[0][1],2),round(active[0][2],2)],
  'end_xy':[round(active[-1][1],2),round(active[-1][2],2)],'terminal':pr['terminal']})
print(json.dumps({'event_paths':len(paths),'matched_to_bag_status':len(path_records),'metrics_n':len(metrics),
 'first_path_matches':path_records[0]['status_stamp_ns'] if path_records else None,
 'metrics':metrics},indent=2))
