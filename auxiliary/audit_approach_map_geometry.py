"""Public-map snapshot checks, not reconstruction of locked controller internals."""
import bisect,hashlib,json,math,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import footprint_blockers,cell_cost
root=Path('/home/ly/execution_continuity_review_20261022/v25_fast_shift')
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def pose(p):
    q=p.orientation
    return p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
reports=[]
for trial in sorted((root/'full').glob('trial_*')):
    assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'))
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
    def load(name):
        tid,typ=topics[name];cls=get_message(typ)
        return [(t,deserialize_message(b,cls)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
    grids={k:load(k) for k in ('/global_costmap/costmap','/local_costmap/costmap')}
    odom=load('/spot1/odom');received=load('/received_global_plan')
    plans={d['stamp_ns']:d for _,m in load('/wildos/plan_status') for d in [json.loads(m.data)]}
    db.close()
    rows=[]
    for t,event in sorted(plans.items()):
        if not (event.get('status')=='replanning_after_maneuver' and event.get('reason')=='footprint_collision'
            or event.get('status') in ('follow_failed','continuous_motion_failed') and event.get('reason')=='follow_aborted'):continue
        oi=bisect.bisect_right([bt for bt,m in odom],t)-1
        bt,o=odom[oi];x,y,h=pose(o.pose.pose)
        pi=bisect.bisect_right([bt for bt,m in received],t)-1
        last_path=received[pi] if pi>=0 else None
        row=dict(event=event,odom_age_s=(t-ns(o))/1e9,pose=[x,y,h],maps={})
        if last_path:row['received_path']=dict(record_age_s=(t-last_path[0])/1e9,frame=last_path[1].header.frame_id,poses=len(last_path[1].poses))
        for topic,series in grids.items():
            samples=[]
            for offset in (-2.,-1.,-.5,0.):
                gi=bisect.bisect_right([bt for bt,m in series],t+int(offset*1e9))-1
                if gi<0:continue
                gt,g=series[gi];assert g.header.frame_id==o.header.frame_id
                blockers=footprint_blockers(g,(x,y),h,limit=30)
                s=dict(snapshot_offset_s=offset,source_age_at_failure_s=(t-ns(g))/1e9,
                    center_cost=cell_cost(g,x,y),body_blockers_relative=blockers,
                    blockers_world=[[x+math.cos(h)*a-math.sin(h)*b,y+math.sin(h)*a+math.cos(h)*b] for a,b in blockers])
                if offset==0 and last_path:
                    points=[]
                    for stamped in last_path[1].poses:
                        px,py,ph=pose(stamped.pose)
                        if last_path[1].header.frame_id==o.header.frame_id:points.append((px,py,ph))
                        elif last_path[1].header.frame_id=='spot1/nav_base_link':points.append((x+math.cos(h)*px-math.sin(h)*py,y+math.sin(h)*px+math.cos(h)*py,h+ph))
                    if points:
                        start=min(range(len(points)),key=lambda i:math.hypot(points[i][0]-x,points[i][1]-y));length=0.
                        for i in range(start,len(points)):
                            if i>start:length+=math.dist(points[i-1][:2],points[i][:2])
                            if length>2.:break
                            blocked=footprint_blockers(g,points[i][:2],points[i][2],limit=5)
                            if blocked:
                                s['first_path_prefix_blocker']=dict(along_path_m=length,pose=points[i],body_relative=blocked);break
                samples.append(s)
            row['maps'][topic]=samples
        rows.append(row)
    reports.append(dict(trial=str(trial),rows=rows,caveat='Public OccupancyGrid snapshots and prior odom/path, not exact locked private costmap or rejected command arc. Bridge checker includes interior cells, while moving RPP primarily checks raster boundary; path prefix is not command arc. No raw LiDAR provenance recorded.'))
encoded=json.dumps(reports,sort_keys=True).encode();(root/'approach_map_geometry.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),rounds=reports)))
