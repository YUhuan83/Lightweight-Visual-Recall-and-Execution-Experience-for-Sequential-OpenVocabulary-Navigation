"""Closed-run independent collision coordinates, bridge epoch, and cloud-stage audit."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import math
import sqlite3
import sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from beamng_wildos_adapter.geometry import body_rotation,xyz
from wildos_nav2_adapter.pointcloud import decode_xyz,split_costmap_points,filter_robot_footprint,filter_height,local_ground_plane
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
from wildos_nav2_adapter.source_control import IDENTITY,receipt_issue

run=Path(sys.argv[1]);probe=json.loads((run/'contact_result.json').read_text())
result=json.loads((run/'result.json').read_text());sensor=json.loads((run/'sensor.json').read_text())
assert result['component_closed'] and result['errors']==[] and probe['protocol_closed']
assert probe['unload']['unloaded'] is True and (run/'interfaces/metadata.yaml').is_file()
anchor=sensor['bridge_start_time_ros_ns'];assert type(anchor) is int
db=next((run/'interfaces').glob('*.db3'))
con=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
topics={n:(i,t) for i,n,t in con.execute('SELECT id,name,type FROM topics')}
counts={n:con.execute('SELECT count(*) FROM messages WHERE topic_id=?',(i,)).fetchone()[0] for n,(i,_) in topics.items()}
def load(name):
    i,typ=topics[name];cls=get_message(typ)
    return [(t,deserialize_message(data,cls)) for t,data in con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(i,))]
def ns(message):return message.header.stamp.sec*10**9+message.header.stamp.nanosec
def stats(values):
    a=np.asarray(values,float)
    return dict(n=len(a),minimum=float(a.min()),median=float(np.median(a)),p95=float(np.percentile(a,95)),maximum=float(a.max())) if len(a) else None
def vec(value):return np.array([value[a] for a in ('x','y','z')],float)

states=load('/vehicles/ego/sensors/state');raw=load('/vehicles/ego/sensors/front_lidar')
adapted=load('/spot1/ouster/front/points_filtered');mark=load('/wildos/points');clear=load('/wildos/clearing_points')
timing=[json.loads(m.data) for t,m in load('/spot1/measurement_timing')]
native=[json.loads(m.data) for t,m in load('/wildos/native_control_status')]
commands=[json.loads(m.data) for t,m in load('/wildos/native_control_command')]
con.close()
state_by_stamp={ns(m):m for t,m in states};raw_by_stamp={ns(m):m for t,m in raw}
mark_by_stamp={ns(m):m for t,m in mark};clear_by_stamp={ns(m):m for t,m in clear}
lidar_timing={v['mapped_stamp_ns']:v for v in timing if v['sensor']=='lidar' and v.get('status')=='published'}
all_nodes=[n for s in probe['contacts'] for n in (s['contacts']['nodes'] or [])]
body=[n for n in all_nodes if n['class']=='body']
reference_errors=[n.get('peak_reference_error') for n in all_nodes if n.get('peak_reference_error')]
assert not reference_errors and all(n.get('peak_reference') for n in all_nodes)
def error(n):return float(np.linalg.norm(vec(n['peak_position'])-vec(n['peak_reference']['node_world'])))
node_rows=[dict(node_id=n['node_id'],node_name=n.get('node_name'),part=n.get('part_origin'),
    force=n['max_normal_force'],world_error_m=error(n),
    relative_hypothesis_error_m=float(np.linalg.norm(vec(n['peak_position'])-vec(n['peak_reference']['node_relative']))),
    peak_sim_time=n['peak_sim_time'],reference_sim_time=n['peak_reference']['engine_sim_time']) for n in body]
peak=max(body,key=lambda n:n['max_normal_force']) if body else None
samples=[]
if peak:
    contact=vec(peak['peak_reference']['node_world'])
    valid=[(t,m) for t,m in adapted if ns(m) in lidar_timing and lidar_timing[ns(m)]['state_source_stamp_ns'] in state_by_stamp]
    earliest=min((lidar_timing[ns(m)]['state_source_stamp_ns']-anchor)*1e-9 for t,m in valid)
    for target in (earliest,earliest+1.,peak['peak_sim_time']-.25,peak['peak_sim_time'],peak['peak_sim_time']+3.):
        t,m=min(valid,key=lambda row:abs((lidar_timing[ns(row[1])]['state_source_stamp_ns']-anchor)*1e-9-target))
        tm=lidar_timing[ns(m)];sm=state_by_stamp[tm['state_source_stamp_ns']]
        actual_sim=(tm['state_source_stamp_ns']-anchor)*1e-9
        rotation=body_rotation(sm.dir,sm.up);origin=xyz(sm.position)
        p=decode_xyz(m)-np.asarray(SBR_BASE_OFFSET);p=p[np.linalg.norm(p,axis=1)<=40.]
        outside=filter_robot_footprint(p,3.4,1.05,1.05);height=filter_height(outside,.1,1.5)
        plane=local_ground_plane(outside,8.)
        replay_mark,replay_clear=split_costmap_points(p,min_height=.1,max_height=1.5,filter_footprint=True,front_extent=3.4,rear_extent=1.05,half_width=1.05,voxel=.2,max_points=20000,ground_plane_radius=8.,ground_clearance=.25)
        def to_world(points):return (points+np.asarray(SBR_BASE_OFFSET))@rotation.T+origin
        def roi(points):return np.linalg.norm(points[:,:2]-contact[:2],axis=1)<=.4
        def region(points):
            choose=roi(points)
            return dict(count=int(choose.sum()),nearest_3d_m=float(np.linalg.norm(points-contact,axis=1).min()) if len(points) else None)
        world=to_world(p);roi_p=p[roi(world)]
        residual=roi_p[:,2]-(roi_p[:,0]*plane[0]+roi_p[:,1]*plane[1]+plane[2]) if plane is not None else []
        row=dict(target_engine_sim=target,matched_state_engine_sim=actual_sim,sim_difference_sec=actual_sim-target,
            source_header_ns=tm['source_stamp_ns'],state_header_ns=tm['state_source_stamp_ns'],mapped_header_ns=ns(m),
            source_skew_ns=tm['skew_ns'],raw=region(decode_xyz(raw_by_stamp[tm['source_stamp_ns']])),
            adapted=region(world),outside_footprint=region(to_world(outside)),after_height=region(to_world(height)),
            roi_body_z=stats(roi_p[:,2]),roi_plane_residual=stats(residual),contact_world=contact.tolist(),
            fitted_plane=None if plane is None else plane.tolist(),
            body_rotation=rotation.tolist(),vehicle_origin=origin.tolist())
        for label,by_stamp,replayed in [('marking',mark_by_stamp,replay_mark),('clearing',clear_by_stamp,replay_clear)]:
            observed=decode_xyz(by_stamp[ns(m)])
            row[label]=region(to_world(observed))
            row[label+'_replay']=dict(recorded=len(observed),replayed=len(replayed),
                shape_equal=observed.shape==replayed.shape,
                max_error=float(np.abs(observed-replayed).max()) if observed.shape==replayed.shape and len(observed) else None)
        samples.append(row)

repeats=[];previous=None
for t,m in raw:
    digest=hashlib.sha256(bytes(m.data)).hexdigest()
    source=state_by_stamp.get(ns(m))
    if previous and previous['digest']==digest and previous['stamp']!=ns(m) and source is not None and previous['state'] is not None:
        repeats.append(dict(previous_source_ns=previous['stamp'],current_source_ns=ns(m),
            sim_header_advance_sec=(ns(m)-previous['stamp'])*1e-9,
            pose_displacement_m=math.dist(xyz(source.position),xyz(previous['state'].position)),
            state_speed_mps=float(np.linalg.norm(xyz(source.velocity))),payload_sha256=digest))
    previous=dict(digest=digest,stamp=ns(m),state=source)
events=[json.loads(s) for s in (run/'contact_result.events.jsonl').read_text().splitlines()]
stalls=[e for e in events if e['kind']=='execution_stalled'];window=[];source_match=[]
if stalls:
    end=stalls[0]['wall_ns'];begin=end-round(stalls[0]['no_progress_sec']*1e9)
    window=[n for n in native if begin<=round(n['wall_time']*1e9)<=end]
    source_map={(v['sender_id'],v['generation'],v['sequence']):v for v in commands}
    for n in window:
        receipt=n.get('source_receipt') or {};source=source_map.get(tuple(receipt.get(k) for k in ('sender_id','generation','sequence')))
        source_match.append(source is not None and all(receipt.get(k)==source[k] for k in (*IDENTITY,'control')) and receipt['control']==n['sent'] and receipt_issue(receipt,round(n['wall_time']*1e9)) is None)
report=dict(scope=__doc__,run=str(run),outcome=probe['outcome'],complete_navigation_passed=False,
    database=dict(path=str(db),bytes=db.stat().st_size,sha256=hashlib.sha256(db.read_bytes()).hexdigest()),counts=counts,
    bridge_clock=dict(anchor_ros_ns=anchor,delay_offset_sec=sensor['bridge_delay_offset_sec'],
        state_engine_seconds_range=[(ns(states[0][1])-anchor)*1e-9,(ns(states[-1][1])-anchor)*1e-9],
        contract='source header = bridge anchor + SDK engine elapsed time; mapped header additionally uses adapter offset',
        lidar_true_acquisition_time_verified=False),
    contact_coordinates=dict(nodes=len(all_nodes),body_nodes=len(body),world_errors_m=stats([error(n) for n in all_nodes]),
        body_world_errors_m=stats([error(n) for n in body]),body_records=node_rows,
        reference_errors=reference_errors,body_peak=peak,
        body_normal_dot_same_callback_direction=stats([float(vec(n['peak_normal'])@vec(n['peak_reference']['direction'])) for n in body if n['node_id']==161]),
        event_reference_time_errors_sec=stats([n['peak_sim_time']-n['peak_reference']['engine_sim_time'] for n in all_nodes]),
        collision_object_identity_available=False),
    cloud_snapshots=samples,repeated_raw_payload_new_header=dict(pairs=len(repeats),
        moving_pairs=sum(r['pose_displacement_m']>.01 for r in repeats),records=repeats),
    stall_window=dict(n=len(window),source_matches=sum(source_match),brake=stats([n['sent']['brake'] for n in window]),
        throttle=stats([n['sent']['throttle'] for n in window]),gears=dict(Counter(n['actual']['gear'] for n in window)),
        watchdog=sum(n['watchdog_stop'] for n in window)),
    enable_barrier=probe.get('enable_barrier'),stop=probe['stop'],unload=probe['unload'])
(run/'frame_review.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('cloud_snapshots','stop','contact_coordinates','repeated_raw_payload_new_header')},indent=2))
print(json.dumps(dict(body_world_error=report['contact_coordinates']['body_world_errors_m'],
    repeated_payload_pairs=len(repeats),moving_pairs=report['repeated_raw_payload_new_header']['moving_pairs'],
    cloud_snapshots=samples),indent=2))
