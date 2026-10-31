"""Read only a closed recorded-section run; separate force, sensor and public-map evidence."""
from pathlib import Path
from collections import Counter
import bisect
import hashlib
import json
import math
import sqlite3
import sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from beamng_wildos_adapter.geometry import body_rotation,xyz
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
from wildos_nav2_adapter.pointcloud import decode_xyz,split_costmap_points,local_ground_plane,filter_robot_footprint,filter_height
from wildos_nav2_adapter.graph_path_bridge import xy,yaw
from wildos_nav2_adapter.graph_route import footprint_blockers,validate_local_prefix,cell_cost
from wildos_nav2_adapter.continuous_path import trim_moving_path
from wildos_nav2_adapter.source_control import IDENTITY,receipt_issue


run=Path(sys.argv[1]);result=json.loads((run/'result.json').read_text())
probe=json.loads((run/'contact_result.json').read_text())
assert result['errors']==[] and result['component_closed'] and probe['protocol_closed']
assert probe['errors']==[] and probe['unload']['unloaded'] is True
assert (run/'interfaces/metadata.yaml').is_file()
db=next((run/'interfaces').glob('*.db3'))
con=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
topics={n:(i,t) for i,n,t in con.execute('SELECT id,name,type FROM topics')}
def load(topic):
    if topic not in topics:return []
    ident,typ=topics[topic];cls=get_message(typ)
    return [(t,deserialize_message(data,cls)) for t,data in con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(ident,))]
def ns(message):return message.header.stamp.sec*10**9+message.header.stamp.nanosec
def latest(rows,wall_ns):
    index=bisect.bisect_right([t for t,m in rows],wall_ns)-1
    return rows[max(index,0)] if rows else None
def nearest(rows,stamp):return min(rows,key=lambda row:abs(ns(row[1])-stamp))
def summary(values):
    a=np.asarray(values,float)
    return dict(n=len(a),minimum=float(a.min()),median=float(np.median(a)),p95=float(np.percentile(a,95)),maximum=float(a.max())) if len(a) else None
def cloud_roi(points,contact):
    delta=points-contact
    inside=np.linalg.norm(delta[:,:2],axis=1)<=.4
    near=points[inside]
    return dict(points=len(points),within_xy_40cm=int(inside.sum()),
        within_xy_40cm_z_25cm=int((inside&(np.abs(delta[:,2])<=.25)).sum()),
        nearest_3d_m=float(np.linalg.norm(delta,axis=1).min()) if len(points) else None,
        z_range_in_xy_40cm=[float(near[:,2].min()),float(near[:,2].max())] if len(near) else None)

counts={n:con.execute('SELECT count(*) FROM messages WHERE topic_id=?',(i,)).fetchone()[0] for n,(i,t) in topics.items()}
timings=[(t,json.loads(m.data)) for t,m in load('/spot1/measurement_timing')]
state=load('/vehicles/ego/sensors/state');odom=load('/spot1/odom')
raw=load('/vehicles/ego/sensors/front_lidar');adapted=load('/spot1/ouster/front/points_filtered')
mark=load('/wildos/points');clear=load('/wildos/clearing_points')
global_maps=load('/global_costmap/costmap');local_maps=load('/local_costmap/costmap')
source=[json.loads(m.data) for t,m in load('/wildos/native_control_command')]
controls=[(t,json.loads(m.data)) for t,m in load('/wildos/control_status')]
native=[(t,json.loads(m.data)) for t,m in load('/wildos/native_control_status')]
native_sim=[v for t,v in native if type(v.get('sim_time')) in (int,float) and math.isfinite(v['sim_time'])]
con.close()
required=('/vehicles/ego/sensors/state','/spot1/measurement_timing','/vehicles/ego/sensors/front_lidar',
          '/spot1/ouster/front/points_filtered','/wildos/points','/wildos/clearing_points','/wildos/diagnostic/contact_snapshot')
for topic in required:assert counts.get(topic,0)>0,topic
by_state={ns(m):(t,m) for t,m in state};by_odom={ns(m):(t,m) for t,m in odom}
clock_rows=[v for t,v in timings if v['sensor']=='state' and v.get('status')=='published']
mapped_checks=[(v['source_stamp_ns'] in by_state,v['mapped_stamp_ns'] in by_odom,
               v['mapped_stamp_ns']-v['source_stamp_ns']==v['clock_offset_ns']) for v in clock_rows]
sdk_anchors=[]
for st,sm in state:
    if np.linalg.norm(xyz(sm.velocity))<=.2:continue
    candidates=[v for v in native_sim if math.dist(v['position'],xyz(sm.position))<1e-7
                and math.dist(v['velocity'],xyz(sm.velocity))<1e-7]
    if len(candidates)==1:
        sdk_anchors.append(ns(sm)-round(candidates[0]['sim_time']*1e9))
lidar_rows=[v for t,v in timings if v['sensor']=='lidar' and v.get('status')=='published']
timing_for_stamp={v['mapped_stamp_ns']:v for v in lidar_rows}
raw_by_stamp={ns(m):(t,m) for t,m in raw};mark_by_stamp={ns(m):(t,m) for t,m in mark};clear_by_stamp={ns(m):(t,m) for t,m in clear}

events=[json.loads(line) for line in (run/'contact_result.events.jsonl').read_text().splitlines()]
stall=next(e for e in events if e['kind']=='execution_stalled')
end=stall['wall_ns'];begin=end-round(stall['no_progress_sec']*1e9)
window=[v for t,v in native if begin<=round(v['wall_time']*1e9)<=end]
commands={(v['sender_id'],v['generation'],v['sequence']):v for v in source}
matched=[];invalid=[];reasons=Counter()
for v in window:
    receipt=v.get('source_receipt') or {};key=tuple(receipt.get(k) for k in ('sender_id','generation','sequence'))
    command=commands.get(key)
    exact=command is not None and all(receipt.get(k)==command[k] for k in (*IDENTITY,'control')) and receipt.get('control')==v['sent']
    issue=receipt_issue(receipt,round(v['wall_time']*1e9))
    matched.append(exact and issue is None)
    if not exact or issue:invalid.append(dict(wall_time=v['wall_time'],exact=exact,issue=issue))
    nearest_control=latest(controls,round(v['wall_time']*1e9))
    if nearest_control:reasons[nearest_control[1]['reason']]+=1

body=[(s,n) for s in probe['contacts'] for n in (s['contacts']['nodes'] or []) if n['class']=='body']
sustained=[(s,n) for s,n in body if n['node_id']==161]
peak_s,peak=max(body,key=lambda item:item[1]['max_normal_force'])
contact=np.array([peak['peak_position'][a] for a in ('x','y','z')])
opposition=[]
for sample,node in sustained:
    measured=min(native_sim,key=lambda v:abs(v['sim_time']-node['peak_sim_time']))
    opposition.append(float(np.array([node['peak_normal'][a] for a in ('x','y','z')])@np.asarray(measured['direction'])))

fixture=json.loads((run.parent/'recorded_section_fixture.json').read_text())
from nav_msgs.msg import Path as Route
from rosidl_runtime_py.set_message import set_message_fields
route=Route();set_message_fields(route,fixture['path'])
points=[xy(p.pose) for p in route.poses];headings=[yaw(p.pose) for p in route.poses]
snapshots=[]
for target_sim in (4.,5.,peak['peak_sim_time']-.3,peak['peak_sim_time'],10.5,13.5):
    witness=min(native_sim,key=lambda v:abs(v['sim_time']-target_sim))
    wall_ns=round(witness['wall_time']*1e9)
    at,am=latest(adapted,wall_ns)
    timing=timing_for_stamp.get(ns(am))
    assert timing is not None,'Missing explicit adapted/source mapping'
    st,sm=by_state[timing['state_source_stamp_ns']]
    rotation=body_rotation(sm.dir,sm.up);origin=xyz(sm.position)
    raw_tuple=raw_by_stamp.get(timing['source_stamp_ns'])
    assert raw_tuple is not None,'Missing raw LiDAR selected by adapter'
    raw_points=decode_xyz(raw_tuple[1]);local_points=decode_xyz(am)
    body_points=local_points-np.asarray(SBR_BASE_OFFSET)
    body_points=body_points[np.linalg.norm(body_points,axis=1)<=40.]
    replay_mark,replay_clear=split_costmap_points(body_points,min_height=.1,max_height=1.5,filter_footprint=True,
        front_extent=3.4,rear_extent=1.05,half_width=1.05,voxel=.2,max_points=20000,ground_plane_radius=8.,ground_clearance=.25)
    outside=filter_robot_footprint(body_points,3.4,1.05,1.05)
    height=filter_height(outside,.1,1.5)
    plane=local_ground_plane(outside,8.)
    # Use the full 3D rotation for XY; pitch/roll contribute to world XY too.
    body_world=(body_points+np.asarray(SBR_BASE_OFFSET))@rotation.T+origin
    world_roi=np.linalg.norm(body_world[:,:2]-contact[:2],axis=1)<=.4
    roi_body=body_points[world_roi]
    after_foot=(outside+np.asarray(SBR_BASE_OFFSET))@rotation.T+origin
    after_height=(height+np.asarray(SBR_BASE_OFFSET))@rotation.T+origin
    residual=roi_body[:,2]-(roi_body[:,0]*plane[0]+roi_body[:,1]*plane[1]+plane[2]) if plane is not None else []
    item=dict(target_sim_time=target_sim,state_header_stamp_ns=ns(sm),
        state_header_clock='BeamNG ROS bridge anchored epoch; not obj:getSimTime seconds',
        contact_to_sensor_pairing='nearest Native sim/wall sample then latest recorded cloud; not an exact acquisition join',
        source_skew_ns=timing['skew_ns'],mapped_ros_stamp_ns=ns(am),
        source_pose=dict(position=origin.tolist(),body_rotation=rotation.tolist()),
        raw=cloud_roi(raw_points,contact),adapted=cloud_roi(local_points@rotation.T+origin,contact),
        contact_body_xyz=((contact-origin)@rotation-np.asarray(SBR_BASE_OFFSET)).tolist(),
        fitted_ground_plane=None if plane is None else plane.tolist(),
        roi_filter_stages=dict(before_footprint=len(roi_body),
            after_footprint=cloud_roi(after_foot,contact)['within_xy_40cm'],
            after_body_height=cloud_roi(after_height,contact)['within_xy_40cm'],
            body_z=summary(roi_body[:,2]),plane_residual=summary(residual),
            passing_ground_clearance_if_height_gate_removed=int(np.count_nonzero(np.asarray(residual)>=.25))))
    crop={}
    for label,rows in [('marking',mark_by_stamp),('clearing',clear_by_stamp)]:
        pair=rows.get(ns(am))
        if pair:
            recorded=decode_xyz(pair[1]);replay=replay_mark if label=='marking' else replay_clear
            item[label+'_replay']=dict(recorded=len(recorded),replayed=len(replay),
                exact_shape=recorded.shape==replay.shape,
                maximum_point_error=float(np.max(np.abs(recorded-replay))) if recorded.shape==replay.shape and len(recorded) else None)
            world=(recorded+np.asarray(SBR_BASE_OFFSET))@rotation.T+origin
            item[label]=cloud_roi(world,contact)
            roi=np.linalg.norm(world[:,:2]-contact[:2],axis=1)<=1.
            crop[label]=world[roi].tolist()
    for label,cloud in [('raw',raw_points),('adapted',local_points@rotation.T+origin)]:
        crop[label]=cloud[np.linalg.norm(cloud[:,:2]-contact[:2],axis=1)<=1.].tolist()
    ot,om=latest(odom,wall_ns);position=xy(om.pose.pose);heading=yaw(om.pose.pose)
    trimmed=trim_moving_path(route,position,heading)
    assert trimmed is not None,'Actual pose cannot join recorded first section'
    prefix=[xy(p.pose) for p in trimmed.poses];h=[yaw(p.pose) for p in trimmed.poses]
    item['public_maps']={}
    for label,maps in [('global',global_maps),('local',local_maps)]:
        mt,grid=latest(maps,wall_ns)
        item['public_maps'][label]=dict(stamp_age_sec=(wall_ns-ns(grid))*1e-9,
            contact_cell_cost=cell_cost(grid,*contact[:2]),
            footprint_blockers=footprint_blockers(grid,position,heading),
            forward_prefix_issue=validate_local_prefix(prefix,h,grid,position,heading))
    item['crop_world_points']=crop;snapshots.append(item)

report=dict(scope=__doc__,run=str(run),outcome=probe['outcome'],execution_passed=False,complete_navigation_passed=False,
    database=dict(path=str(db),bytes=db.stat().st_size,sha256=hashlib.sha256(db.read_bytes()).hexdigest()),
    counts=counts,native_sim_time_missing=len(native)-len(native_sim),clock_mapping=dict(state_records=len(clock_rows),state_source_recorded=sum(r[0] for r in mapped_checks),
        mapped_odom_recorded=sum(r[1] for r in mapped_checks),offset_equal=sum(r[2] for r in mapped_checks),
        resets=sum(v.get('clock_reset') is True for v in clock_rows),
        exact_moving_sdk_payload_anchor=summary(sdk_anchors),
        lidar_statuses=dict(Counter(v.get('status') for t,v in timings if v['sensor']=='lidar'))),
    stall_window=dict(samples=len(window),source_matches=sum(matched),invalid=invalid,
        public_control_reasons=dict(reasons),speed=summary([v['speed'] for v in window]),
        throttle=summary([v['sent']['throttle'] for v in window]),brake=summary([v['sent']['brake'] for v in window]),
        actual_throttle=summary([v['actual']['throttle'] for v in window]),
        gears=dict(Counter(v['actual']['gear'] for v in window)),
        watchdogs=sum(v['watchdog_stop'] for v in window),
        net_displacement_m=math.dist(window[0]['position'],window[-1]['position']),
        sim_interval=[window[0]['sim_time'],window[-1]['sim_time']]),
    body_contacts=dict(events=probe['event_totals']['body'],windows=len(body),
        before_motion_events=sum(s['contacts']['class_events']['body'] for s in probe['contacts'] if s['phase']=='before_motion'),
        peak=peak,sustained_node=161,sustained_normal_dot_forward=summary(opposition),
        sustained_sim_interval=[sustained[0][1]['first_sim_time'],sustained[-1][1]['peak_sim_time']],
        vector_frame_hypothesis='world; dot/ROI comparisons conditional on this hypothesis',full_engine_vector_contract_verified=False,
        object_identity_proven=False),
    snapshots=snapshots,source_runtime_identity=json.loads((run/'running_identity.json').read_text()),
    visual_handoff_present=False,graph_bridge_follow_path_enabled=False,
    stock_respawn_not_historical_damage_or_gear_history=True,
    stop_speed=probe['stop']['physics']['speed'],unloaded=probe['unload']['unloaded'])
manifest=json.loads(Path('/home/ly/smoothing_boundary_repair_20261018/official_wildos_beamng_bundle.manifest.json').read_text())
report['analysis_source_identity']={}
for module_name,prefix in [('wildos_nav2_adapter.pointcloud','nav2_adapter'),
        ('wildos_nav2_adapter.graph_route','nav2_adapter'),('wildos_nav2_adapter.continuous_path','nav2_adapter'),
        ('wildos_nav2_adapter.source_control','nav2_adapter'),('beamng_wildos_adapter.geometry','official_adapter')]:
    path=Path(sys.modules[module_name].__file__)
    member=prefix+'/'+module_name.replace('.','/')+'.py'
    actual=hashlib.sha256(path.read_bytes()).hexdigest()
    report['analysis_source_identity'][module_name]=dict(path=str(path),sha256=actual,equals_frozen_member=actual==manifest['members'][member])
    assert actual==manifest['members'][member],member
(run/'review_recorded.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('snapshots','source_runtime_identity')},indent=2))
for item in snapshots:print(json.dumps({k:v for k,v in item.items() if k!='crop_world_points'},indent=2))
