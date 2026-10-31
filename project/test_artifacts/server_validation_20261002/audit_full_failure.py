"""Read saved ROS bags only; never publish, resume or control the simulation."""
from pathlib import Path
import bisect
import collections
import hashlib
import json
import math
import sqlite3
import subprocess
import sys

sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'), __file__, *sys.argv[1:], '--runtime-child'],
                                   env=runtime_env()).returncode)

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_path_bridge import xy, yaw
from wildos_nav2_adapter.graph_route import cell_cost, footprint_blockers, distance_to_route
from beamng_wildos_adapter.pointcloud import decode_xyz

folder = ROOT/sys.argv[1]
out = ROOT/'failure_audit'/sys.argv[1]
out.mkdir(parents=True, exist_ok=True)
db = sqlite3.connect('file:'+str(next((folder/'rosbag').glob('*.db3')))+'?mode=ro', uri=True)
topics = {name:(ident, get_message(kind)) for ident,name,kind in db.execute('select id,name,type from topics')}
cache = {}

def messages(topic):
    if topic not in cache:
        if topic not in topics: cache[topic] = []
        else:
            ident, kind = topics[topic]
            cache[topic] = [(t, deserialize_message(data, kind)) for t,data in db.execute(
                'select timestamp,data from messages where topic_id=? order by timestamp', (ident,))]
    return cache[topic]

def before(topic, stamp):
    if topic not in topics:return None,None
    ident, kind = topics[topic]
    row = db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',
                     (ident,stamp)).fetchone()
    return (row[0],deserialize_message(row[1],kind)) if row else (None,None)

def after(topic, stamp):
    if topic not in topics:return None,None
    ident,kind=topics[topic]
    row=db.execute('select timestamp,data from messages where topic_id=? and timestamp>=? order by timestamp limit 1',
                   (ident,stamp)).fetchone()
    return (row[0],deserialize_message(row[1],kind)) if row else (None,None)

def stampof(m):
    return m.header.stamp.sec+m.header.stamp.nanosec/1e9

queries=[(t,m.data) for t,m in messages('/spot1/object_search/query')]
case=json.loads((folder/'full_test.json').read_text())
house_t=next((t for t,q in queries if q=='house'), None)
house_e=next(e['elapsed'] for e in case['events'] if e['kind']=='query_sent' and e['query']=='house')
if house_t is not None:
    base=house_t/1e9-house_e
    clock_anchor='house query recorded in bag'
else:
    # Recorder discovery can miss an early query. The later goal switch is
    # logged by both the harness and recorder, so use that clock anchor.
    return_t=next(t for t,q in queries if q=='gas station')
    return_e=next(e['elapsed'] for e in case['events']
                  if e['kind']=='query_sent' and e['phase']=='station_return')
    base=return_t/1e9-return_e
    clock_anchor='station_return query; early house query absent from bag'
def elapsed(t):return round(t/1e9-base,4)

def near_cells(grid, position, heading, radius=6):
    cells=np.asarray(grid.data).reshape(grid.info.height,grid.info.width)
    yy,xx=np.where(cells>=99)
    world=np.c_[grid.info.origin.position.x+(xx+.5)*grid.info.resolution,
                grid.info.origin.position.y+(yy+.5)*grid.info.resolution]
    offset=world-np.array(position)
    f=offset[:,0]*math.cos(heading)+offset[:,1]*math.sin(heading)
    l=-offset[:,0]*math.sin(heading)+offset[:,1]*math.cos(heading)
    d=np.hypot(f,l);ids=np.argsort(d)
    return [[float(f[i]),float(l[i]),float(world[i,0]),float(world[i,1]),int(cells[yy[i],xx[i]])]
            for i in ids if d[i]<radius][:40]

report={'attempt':sys.argv[1], 'trial_base_wall_time':base, 'clock_anchor':clock_anchor, 'topics':{}, 'queries':[],
        'masks':[], 'waypoints':[], 'failures':[], 'paths':[], 'visual_logs':[], 'motion':{}}
for name,(ident,kind) in topics.items():
    count=db.execute('select count(*) from messages where topic_id=?',(ident,)).fetchone()[0]
    report['topics'][name]=count
report['queries']=[{'elapsed':elapsed(t),'query':q} for t,q in queries]
for t,m in messages('/spot1/object_mask'):
    report['masks'].append({'elapsed':elapsed(t),'query':m.query,'image_age_sec':t/1e9-stampof(m),
                            'positive_pixels':int(np.count_nonzero(m.object_mask.data))})
for t,m in messages('/spot1/imgnav_waypoint'):
    report['waypoints'].append({'elapsed':elapsed(t),'point':list(xy(m.pose))})

logs=messages('/rosout')
for t,m in logs:
    if m.name=='wildos' and not ('Received callback' in m.msg or 'Message buffer is empty' in m.msg):
        report['visual_logs'].append({'elapsed':elapsed(t),'level':m.level,'message':m.msg[:500]})
statuses=[(t,json.loads(m.data)) for t,m in messages('/wildos/plan_status')]
seen=set()
for t,s in statuses:
    if s.get('status') not in ('follow_failed','stale_costmap','no_free_graph_candidate','no_safe_vehicle_path'):continue
    key=json.dumps(s,sort_keys=True)
    if key in seen:continue
    seen.add(key)
    item={'elapsed':elapsed(t),'status':s}
    ot,odom=before('/spot1/odom',t)
    if odom is None:continue
    point,heading=xy(odom.pose.pose),yaw(odom.pose.pose)
    q=odom.pose.pose.orientation
    item.update(rear=list(point),heading=heading,odom_receive_age=(t-ot)/1e9,
                orientation=[q.x,q.y,q.z,q.w],height=odom.pose.pose.position.z)
    for topic,label in [('/global_costmap/costmap','global'),('/local_costmap/costmap','local')]:
        gt,grid=before(topic,t)
        if grid is None:continue
        item[label]={'grid_age':(t-gt)/1e9,'center_cost':int(cell_cost(grid,*point)),
                     'current_body_blockers':footprint_blockers(grid,point,heading),
                     'near_occupied_cells':near_cells(grid,point,heading)}
        np.savez_compressed(out/(str(round(elapsed(t),2))+'_'+label+'.npz'),
                            data=np.asarray(grid.data),origin=[grid.info.origin.position.x,grid.info.origin.position.y],
                            shape=[grid.info.height,grid.info.width],resolution=grid.info.resolution,
                            rear=point,heading=heading)
    for topic,label in [('/global_costmap/costmap_raw','global_raw'),('/local_costmap/costmap_raw','local_raw')]:
        gt,grid=before(topic,t)
        if grid is None:continue
        meta=grid.metadata
        item[label]={'age':(t-gt)/1e9,'stamp':stampof(grid),'origin':[meta.origin.position.x,meta.origin.position.y],
                     'shape':[meta.size_y,meta.size_x],'resolution':meta.resolution}
        np.savez_compressed(out/(str(round(elapsed(t),2))+'_'+label+'.npz'),
                            data=np.asarray(grid.data),origin=[meta.origin.position.x,meta.origin.position.y],
                            shape=[meta.size_y,meta.size_x],resolution=meta.resolution,
                            rear=point,heading=heading)
        at,ag=after(topic,t)
        am=ag.metadata
        item[label]['next_publish_delay']=(at-t)/1e9
        np.savez_compressed(out/(str(round(elapsed(t),2))+'_'+label+'_after.npz'),
                            data=np.asarray(ag.data),origin=[am.origin.position.x,am.origin.position.y],
                            shape=[am.size_y,am.size_x],resolution=am.resolution,rear=point,heading=heading)
    for topic,label in [('/wildos/points','marking'),('/wildos/clearing_points','clearing'),
                        ('/spot1/ouster/front/points_filtered','raw_cloud')]:
        ct,cloud=before(topic,t)
        if cloud is not None:
            pts=decode_xyz(cloud)
            item[label]={'age':(t-ct)/1e9,'frame':cloud.header.frame_id,'count':len(pts)}
            cot,co=before('/spot1/odom',ct)
            cq=co.pose.pose.orientation;cp=co.pose.pose.position
            item[label]['odom']={'position':[cp.x,cp.y,cp.z],'orientation':[cq.x,cq.y,cq.z,cq.w]}
            np.savez_compressed(out/(str(round(elapsed(t),2))+'_'+label+'.npz'),points=pts,
                                frame=cloud.header.frame_id,stamp=stampof(cloud))
    pt,path=before('/wildos/planned_path',t)
    if path is not None:
        points=[xy(ps.pose) for ps in path.poses]
        item['last_published_section']={'elapsed':elapsed(pt),'points':points,
                                       'headings':[yaw(ps.pose) for ps in path.poses],
                                       'cross_track':distance_to_route(point,points)}
    at,arc=before('/lookahead_collision_arc',t)
    if arc is not None and arc.poses:
        pts=[xy(ps.pose) for ps in arc.poses]
        # End orientations are unset in the RPP debug arc. Record this limitation.
        tangent=math.atan2(pts[-1][1]-(pts[-2][1] if len(pts)>1 else point[1]),
                           pts[-1][0]-(pts[-2][0] if len(pts)>1 else point[0]))
        if math.cos(tangent-heading)<0:tangent+=math.pi
        item['last_arc']={'age':(t-at)/1e9,'points':pts,'end_heading_approx':tangent}
        for topic,label in [('/global_costmap/costmap','global'),('/local_costmap/costmap','local')]:
            gt,grid=before(topic,t)
            item[label]['arc_end_blockers_approx']=footprint_blockers(grid,pts[-1],tangent)
            item[label]['arc_end_center_cost']=int(cell_cost(grid,*pts[-1]))
    nt,native=before('/wildos/native_control_status',t)
    if native is not None:item['native']=json.loads(native.data)
    report['failures'].append(item)

unique_paths=[];seen=set()
for t,path in messages('/wildos/planned_path'):
    points=[xy(p.pose) for p in path.poses]
    key=str(points)
    if key in seen:continue
    seen.add(key);unique_paths.append((t,path))
    headings=[yaw(p.pose) for p in path.poses]
    p={'elapsed':elapsed(t),'points':points,'headings':headings,'length':sum(math.dist(a,b) for a,b in zip(points,points[1:]))}
    ot,odom=before('/spot1/odom',t)
    p['start_actual']=list(xy(odom.pose.pose));p['start_yaw']=yaw(odom.pose.pose)
    for topic,label in [('/global_costmap/costmap','global'),('/local_costmap/costmap','local')]:
        gt,grid=before(topic,t)
        p[label]={'age':(t-gt)/1e9,'center_costs':[int(cell_cost(grid,*point)) for point in points],
                  'body_blocked_poses':sum(bool(footprint_blockers(grid,point,h,limit=1)) for point,h in zip(points,headings))}
    report['paths'].append(p)

odom=messages('/spot1/odom');native=[(t,json.loads(m.data)) for t,m in messages('/wildos/native_control_status')]
search_end=next(e['elapsed'] for e in case['events'] if e['kind']=='house_search_finished')
search=[(t,m) for t,m in native if house_e<=elapsed(t)<=search_end]
travel=sum(math.dist(a[1]['position'][:2],b[1]['position'][:2]) for a,b in zip(search,search[1:]))
report['motion']={'travelled_native_xy_m':travel,'max_departure_m':case['maximum_departure'],
                  'native_watchdog_count':sum(n['watchdog_stop'] for t,n in search),
                  'maximum_speed':max(n['speed'] for t,n in search),'odom_count':len(odom)}
report['odom']=[{'elapsed':elapsed(t),'position':list(xy(m.pose.pose)),'heading':yaw(m.pose.pose),
                 'linear':m.twist.twist.linear.x,'angular':m.twist.twist.angular.z} for t,m in odom]
report['native']=[{'elapsed':elapsed(t),'position':n['position'],'speed':n['speed'],
                  'sent':n['sent'],'actual':n['actual']} for t,n in search]
for topic in ['/spot1/odom','/global_costmap/costmap','/local_costmap/costmap','/wildos/points','/spot1/scored_nav_graph']:
    rows=db.execute('select timestamp from messages where topic_id=? order by timestamp',(topics[topic][0],)).fetchall()
    gaps=np.diff([r[0]/1e9 for r in rows])
    report.setdefault('rates',{})[topic]={'median_gap':float(np.median(gaps)),'max_gap':float(gaps.max())}
report['controller_errors']=[{'elapsed':elapsed(t),'message':m.msg} for t,m in logs if m.name=='controller_server' and m.level>=30]
report['triangulation_logs']=[{'elapsed':elapsed(t),'node':m.name,'message':m.msg} for t,m in logs
    if ('triangulat' in m.name or 'standoff' in m.name) and
    not ('Message buffer is empty' in m.msg or 'Received callback' in m.msg)]
report['map_after_return_failure']={}
return_fail=next(t for t,s in statuses if s.get('source')=='memory_via_graphnav' and s.get('status')=='follow_failed')
for topic in ['/global_costmap/costmap','/global_costmap/costmap_raw']:
    rows=[(t,g) for t,g in messages(topic) if t>=return_fail]
    hashes=[hashlib.sha256(bytes(np.asarray(g.data,dtype=np.uint8))).hexdigest() for t,g in rows]
    report['map_after_return_failure'][topic]={'publications':len(rows),'distinct_data':len(set(hashes)),
        'first_map_difference_after_s':next(((t-return_fail)/1e9 for (t,g),h in zip(rows,hashes) if h!=hashes[0]),None)}
(out/'audit.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'attempt':sys.argv[1], 'motion':report['motion'],'mask_counts':dict(collections.Counter(m['query'] for m in report['masks'])),
                  'failure_count':len(report['failures']),'rates':report['rates'],'controller_errors':report['controller_errors'],
                  'output':str(out)},indent=2),flush=True)
