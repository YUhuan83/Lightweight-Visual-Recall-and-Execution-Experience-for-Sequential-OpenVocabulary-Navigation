import json, math, statistics
from collections import defaultdict
import bisect
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag='/home/ly/town_memory_validation_20261003/green_house_station_arrival_20261006_idealpoint_syncfix1/rosbag'
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr','cdr'))
types={x.name:x.type for x in r.get_all_topics_and_types()}
msgs=defaultdict(list)
keep={'/spot1/odom','/vehicles/ego/sensors/state','/wildos/control_status','/wildos/plan_status','/wildos/execution_feedback','/spot1/graphnav_planner/path','/spot1/graphnav_planner/route_status'}
while r.has_next():
    topic,data,recv=r.read_next()
    if topic not in keep: continue
    m=deserialize_message(data,get_message(types[topic]))
    msgs[topic].append((recv,m))
def stamp(t): return t.sec*1e9+t.nanosec
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def val(m):
    try:return json.loads(m.data)
    except:return {}
out={}
# Odom samples and pose progression
od=msgs['/spot1/odom']; rows=[]
for recv,m in od:
    p=m.pose.pose.position; q=m.pose.pose.orientation; tw=m.twist.twist
    rows.append({'recv':recv,'stamp':stamp(m.header.stamp),'frame':m.header.frame_id,'x':p.x,'y':p.y,'yaw':yaw(q),'vx':tw.linear.x,'wz':tw.angular.z})
stepdist=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(rows,rows[1:]))
out['odom']={'n':len(rows),'frame_counts':dict((f,sum(x['frame']==f for x in rows)) for f in set(x['frame'] for x in rows)),'first':rows[0] if rows else None,'last':rows[-1] if rows else None,'travel':stepdist,'net':math.hypot(rows[-1]['x']-rows[0]['x'],rows[-1]['y']-rows[0]['y']) if rows else None,'heading_abs_deg':sum(abs(math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw']))) for a,b in zip(rows,rows[1:]))*180/math.pi,'max_stamp_skew_ms':max(abs(x['stamp']-x['recv'])/1e6 for x in rows),'p95_stamp_skew_ms':sorted(abs(x['stamp']-x['recv'])/1e6 for x in rows)[int(.95*(len(rows)-1))]}
# State sensor and timestamp compare
states=msgs['/vehicles/ego/sensors/state']; sr=[]
for recv,m in states:
    p=m.position; d=m.dir; u=m.up
    f=np.array([d.x,d.y,d.z],dtype=float); f/=max(float(np.linalg.norm(f)),1e-9)
    up=np.array([u.x,u.y,u.z],dtype=float); up/=max(float(np.linalg.norm(up)),1e-9)
    right=np.cross(f,up); right/=max(float(np.linalg.norm(right)),1e-9)
    left=-right
    base=np.array([p.x,p.y,p.z])+np.column_stack((f,left,np.cross(f,left)))@np.array([-2.172,.330,0.0])
    sr.append({'recv':recv,'stamp':stamp(m.header.stamp),'x':p.x,'y':p.y,'dx':d.x,'dy':d.y,
               'odom_x_expected':float(base[0]),'odom_y_expected':float(base[1]),
               'yaw_expected':math.atan2(float(f[1]),float(f[0]))})
state_stamp_skew=[abs(x['stamp']-x['recv'])/1e6 for x in sr]
out['state']={'n':len(sr),'first':sr[0] if sr else None,'last':sr[-1] if sr else None,
              'source_stamp_vs_ros_receipt_offset_ms':{'median':statistics.median(state_stamp_skew) if sr else None,
                 'p95':sorted(state_stamp_skew)[int(.95*(len(state_stamp_skew)-1))] if sr else None,
                 'max':max(state_stamp_skew) if sr else None}}
if sr and rows:
    odom_recv=[x['recv'] for x in rows]
    pos_errors=[]; yaw_errors=[]; header_offsets=[]; receipt_gaps=[]
    for s in sr:
        idx=bisect.bisect_left(odom_recv,s['recv'])
        choices=[rows[i] for i in (idx-1,idx) if 0<=i<len(rows)]
        o=min(choices,key=lambda x:abs(x['recv']-s['recv']))
        receipt_gaps.append((o['recv']-s['recv'])/1e6)
        pos_errors.append(math.hypot(s['odom_x_expected']-o['x'],s['odom_y_expected']-o['y']))
        yaw_errors.append(abs(math.atan2(math.sin(s['yaw_expected']-o['yaw']),math.cos(s['yaw_expected']-o['yaw']))))
        header_offsets.append((o['stamp']-s['stamp'])/1e6)
    def stats(v):
        z=sorted(v)
        return {'median':statistics.median(z),'p95':z[int(.95*(len(z)-1))],'max':max(z)}
    out['state_odom_contract']={'matched_pairs':len(sr),'body_offset_m':[-2.172,.330,0.0],
       'arrival_delta_ms_signed':stats(receipt_gaps),'transformed_xy_error_m':stats(pos_errors),
       'yaw_error_deg':stats([math.degrees(x) for x in yaw_errors]),
       'odom_header_minus_beamng_state_source_stamp_ms':stats(header_offsets)}
# Route transitions grouped by endpoint/mode; Path msgs unique transitions only
routes=[]
for recv,m in msgs['/spot1/graphnav_planner/route_status']:
    v=val(m)
    if v and (not routes or json.dumps(v,sort_keys=True)!=json.dumps(routes[-1]['data'],sort_keys=True)):
        routes.append({'recv':recv,'data':v})
out['route_transitions']=[{'t':round((x['recv']-rows[0]['recv'])/1e9,2) if rows else None,'data':x['data']} for x in routes[:100]]
paths=[]; prev=None
for recv,m in msgs['/spot1/graphnav_planner/path']:
    h=m.header; ps=m.poses
    sig=(stamp(h.stamp),h.frame_id,len(ps),round(ps[-1].pose.position.x,2) if ps else None,round(ps[-1].pose.position.y,2) if ps else None)
    if sig!=prev:
        paths.append({'recv':recv,'stamp':stamp(h.stamp),'frame':h.frame_id,'n':len(ps),'start':[ps[0].pose.position.x,ps[0].pose.position.y] if ps else None,'end':[ps[-1].pose.position.x,ps[-1].pose.position.y] if ps else None})
        prev=sig
out['path_changes']=[{'t':round((x['recv']-rows[0]['recv'])/1e9,2) if rows else None,**x} for x in paths[:150]]
# Feedback failure reasons, transitions in status
out['execution_feedback']=[{'t':round((recv-rows[0]['recv'])/1e9,2) if rows else None,'data':val(m)} for recv,m in msgs['/wildos/execution_feedback']]
plans=[]; last=None
for recv,m in msgs['/wildos/plan_status']:
    v=val(m); state=v.get('state') or v.get('status')
    if state!=last:
        plans.append({'t':round((recv-rows[0]['recv'])/1e9,2) if rows else None,'state':state,'data':v})
        last=state
out['plan_status_changes']=plans[:100]
# movement/change in heading per coarse 5s window using ODOM controller frame
wins=[]
if rows:
    t0=rows[0]['recv']; i=0
    for t in range(0,int((rows[-1]['recv']-t0)/1e9),5):
        a=[r for r in rows if t <= (r['recv']-t0)/1e9 < t+5]
        if len(a)>1:
            dist=sum(math.hypot(b['x']-c['x'],b['y']-c['y']) for c,b in zip(a,a[1:]))
            net=math.hypot(a[-1]['x']-a[0]['x'],a[-1]['y']-a[0]['y'])
            dh=sum(abs(math.atan2(math.sin(b['yaw']-c['yaw']),math.cos(b['yaw']-c['yaw']))) for c,b in zip(a,a[1:]))*180/math.pi
            wins.append({'t':t,'n':len(a),'travel':round(dist,2),'net':round(net,2),'turn_deg':round(dh,1),'v_mean':round(statistics.mean(x['vx'] for x in a),3)})
out['windows_5s']=wins
print(json.dumps(out,indent=2))
