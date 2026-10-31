"""Read closed snapshots through actual GoalStandoff, instrumenting only diagnostics."""
import collections
import json
import math
import sqlite3
import sys
import hashlib
import inspect
from collections import deque
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import PoseStamped
import beamng_wildos_adapter.goal_standoff as gs
import beamng_wildos_adapter.mapping as mapping
from beamng_wildos_adapter.visible_approach import observation_step
from beamng_wildos_adapter.visible_approach import visible_direction
from beamng_wildos_adapter.geometry import quaternion_matrix
from wildos_nav2_adapter.observation_pose import grid_cell,connected_free_cells
from wildos_nav2_adapter.graph_route import footprint_blockers
from wildos_nav2_adapter.graph_path_bridge import xy,yaw

trial=Path(sys.argv[1]);assert json.loads((trial/'result.json').read_text())['closed']
output_root=Path(sys.argv[2]) if len(sys.argv)>2 else trial.parents[1]
goal=next(trial.glob('sessions/*/goal_001'));assert (goal/'stop.json').exists()
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={name:(idx,get_message(typ)) for idx,name,typ in db.execute('select id,name,type from topics')}
def rows(topic):
    idx,cls=topics[topic]
    return [(t,deserialize_message(data,cls)) for t,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(idx,))]
odoms=rows('/spot1/odom');maps=rows('/spot1/local_map');global_maps=rows('/global_costmap/costmap')
confirmations=[(t,json.loads(m.data)) for t,m in rows('/spot1/object_confirmation')]
valid=[(t,d) for t,d in confirmations if d.get('search_confirmed') is True]
def last(seq,t):return next((m for ts,m in reversed(seq) if ts<=t),None)
mask_id,mask_class=topics['/spot1/object_mask']
rclpy.init();node=gs.GoalStandoff()
parameters={'standoff':3.0,'vehicle_front_extent':3.4}
actual_parameter=node.get_parameter
node.get_parameter=lambda name:S(value=parameters[name]) if name in parameters else actual_parameter(name)
node.publisher=Mock();node.get_logger=Mock()
results=[];safe_original=mapping.safe_travel_distance
for index,(stamp,data) in enumerate(valid):
    node.get_clock=lambda stamp=stamp:S(now=lambda:Time(nanoseconds=stamp))
    node.odom=last(odoms,stamp)
    node.odom_history=deque((m for t,m in odoms if t<=stamp),maxlen=200)
    node.occupancy=None;node.vehicle_grid=None;node.vehicle_connectivity=None
    node.rejected_observation_poses.clear();node.last_goal=None;node.active_query=data['query']
    node.on_occupancy(last(maps,stamp));node.on_vehicle_grid(last(global_maps,stamp))
    if index==0:
        def map_metadata(m):
            i=m.info;p=i.origin.position;q=i.origin.orientation
            return dict(frame=m.header.frame_id,stamp_ns=node.stamp_ns(m),width=i.width,height=i.height,resolution=i.resolution,origin=[p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        def pose_metadata(m):
            p=m.pose.pose.position;q=m.pose.pose.orientation
            return dict(stamp_ns=node.stamp_ns(m),position=[p.x,p.y,p.z],orientation=[q.x,q.y,q.z,q.w])
        meta=dict(now_ns=stamp,confirmation=data,local_map=map_metadata(node.occupancy),vehicle_map=map_metadata(node.vehicle_grid),odom=pose_metadata(node.odom),map_odom=pose_metadata(node.occupancy_odom))
        np.savez_compressed(output_root/'standoff_first_snapshot.npz',local_map=np.asarray(node.occupancy.data,dtype=np.int8),vehicle_map=np.asarray(node.vehicle_grid.data,dtype=np.int8),meta=json.dumps(meta))
    target=PoseStamped();target.header.frame_id='spot1/odom'
    target.header.stamp=Time(nanoseconds=data['observation_stamp_ns']).to_msg()
    target.pose.position.x,target.pose.position.y,target.pose.position.z=map(float,data['object_position'])
    trace=[];rays=[]
    original_filter=gs.GoalStandoff.observation_pose_filter.__get__(node)
    def instrumented_filter(**kw):
        accept=original_filter(**kw)
        connected=connected_free_cells(node.vehicle_grid,xy(node.odom.pose.pose))
        def tested(point):
            cell=grid_cell(node.vehicle_grid,point)
            heading=(math.atan2(kw['target'][1]-point[1],kw['target'][0]-point[0]) if 'target' in kw else kw['bearing'])
            detail=('outside_vehicle_map' if cell is None else 'not_connected_known_free' if not connected[cell[1]*node.vehicle_grid.info.width+cell[0]] else 'vehicle_footprint_blocked' if footprint_blockers(node.vehicle_grid,point,heading,limit=1) else 'accepted')
            result=accept(point)
            trace.append(dict(point=list(map(float,point)),heading=float(heading),accepted=bool(result),reason=detail))
            return result
        return tested
    def safe(*args,**kw):
        result=safe_original(*args,**kw)
        ray=np.asarray(args[3],dtype=float);length=float(np.linalg.norm(ray));desired=min(float(args[4]),length)
        first=None
        for dist in np.arange(max(float(args[1])*.5,.05),desired+max(float(args[1])*.5,.05),max(float(args[1])*.5,.05)):
            loc=ray*dist/max(length,1e-6)
            ix=math.floor((loc[0]-args[2][0])/args[1]);iy=math.floor((loc[1]-args[2][1])/args[1])
            value=int(args[0][iy,ix]) if 0<=ix<args[0].shape[1] and 0<=iy<args[0].shape[0] else None
            if value is None or value<0 or value>=50:
                first=dict(distance=float(dist),value=value,reason='outside_local_map' if value is None else 'unknown' if value<0 else 'occupied')
                break
        rays.append(dict(target=list(map(float,args[3])),desired=float(args[4]),safe=float(result),first_unsafe=first))
        return result
    node.observation_pose_filter=instrumented_filter
    with patch.object(gs,'safe_travel_distance',side_effect=safe),patch.object(mapping,'safe_travel_distance',side_effect=safe):
        reason=node.publish_standoff(target)
    final_trace=list(trace);final_rays=list(rays)
    robot=xy(node.odom.pose.pose);pose=node.occupancy_odom.pose.pose
    map_position=xy(pose);map_heading=yaw(pose)
    bearing=math.atan2(data['object_position'][1]-map_position[1],data['object_position'][0]-map_position[0])
    grid=np.asarray(node.occupancy.data,dtype=np.int16).reshape(node.occupancy.info.height,node.occupancy.info.width)
    trace.clear()
    stage=observation_step(grid,node.occupancy.info.resolution,(node.occupancy.info.origin.position.x,node.occupancy.info.origin.position.y),map_position,map_heading,bearing,maximum=8.,accept=instrumented_filter(target=data['object_position'][:2]))
    stage_trace=list(trace)
    clamp_counterfactual=None
    if final_trace:
        point=final_trace[-1]['point'];cell=grid_cell(node.vehicle_grid,point)
        connected=connected_free_cells(node.vehicle_grid,robot)
        clamp_counterfactual=dict(point=point,at_current_heading_clear=bool(cell is not None and connected[cell[1]*node.vehicle_grid.info.width+cell[0]] and not footprint_blockers(node.vehicle_grid,point,yaw(node.odom.pose.pose),limit=1)),
            current_heading=yaw(node.odom.pose.pose),target_facing_heading=final_trace[-1]['heading'])
    mask_row=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(mask_id,stamp)).fetchone()
    visible_control=None
    if mask_row:
        mask=deserialize_message(mask_row[1],mask_class);mask_ns=node.stamp_ns(mask)
        mask_bearing=visible_direction(mask)
        if mask_bearing is not None and 0<=stamp-mask_ns<=2e9:
            node.visible_target=dict(stamp=mask_ns,bearing=mask_bearing)
            node.visible_target_count=3;node.visible_step_published_ns=0
            node.pending_confirmation=None;node.active_approach=None
            node.visible_approach_pub=Mock()
            node.publish_visible_step()
            without=node.visible_approach_pub.publish.call_count
            payload=json.loads(node.visible_approach_pub.publish.call_args.args[0].data) if without else None
            node.visible_approach_pub=Mock();node.visible_step_published_ns=0
            node.pending_confirmation=(target,data)
            node.publish_visible_step()
            visible_control=dict(mask_stamp_ns=mask_ns,mask_bearing=mask_bearing,controlled_continuity_count=3,
                without_pending=without,with_pending=node.visible_approach_pub.publish.call_count,proposal=payload)
            node.pending_confirmation=None
    result=dict(index=index,stamp_ns=stamp,reason=reason,recorded_reason=data.get('approach_failure_reason'),
        robot=list(robot),target=data['object_position'],standoff=node.approach_distance()[0],
        local_map_age=(stamp-node.stamp_ns(node.occupancy))*1e-9,vehicle_map_age=(stamp-node.stamp_ns(node.vehicle_grid))*1e-9,
        map_pair_skew=(node.stamp_ns(node.occupancy_odom)-node.stamp_ns(node.occupancy))*1e-9,
        filter_counts=dict(collections.Counter(x['reason'] for x in final_trace)),accept_trace=final_trace,
        rays=final_rays,stage=None if stage is None else list(stage),clamp_counterfactual=clamp_counterfactual,visible_control=visible_control,
        stage_filter_counts=dict(collections.Counter(x['reason'] for x in stage_trace)),
        map=dict(width=node.vehicle_grid.info.width,height=node.vehicle_grid.info.height,resolution=node.vehicle_grid.info.resolution,
            origin=[node.vehicle_grid.info.origin.position.x,node.vehicle_grid.info.origin.position.y],
            robot_cell=grid_cell(node.vehicle_grid,robot),target_cell=grid_cell(node.vehicle_grid,data['object_position'][:2])))
    results.append(result)
node.destroy_node();rclpy.shutdown();db.close()
source=Path(inspect.getsourcefile(gs.GoalStandoff))
candidate=output_root/'candidate/official_adapter/beamng_wildos_adapter/goal_standoff.py'
assert hashlib.sha256(source.read_bytes()).digest()==hashlib.sha256(candidate.read_bytes()).digest()
report=dict(scope='Actual publish_standoff with recorded latest map/pose snapshots; historical rejected regions omitted, so acceptance is optimistic. Stage is map-only, not Smac or executed motion. Visible suppression controls assume established count=3, not a full historical mask-callback replay.',
    source=dict(imported=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),matches_candidate=True),
    count=len(results),reproduced=sum(x['reason']==x['recorded_reason'] for x in results),
    stage_available=sum(x['stage'] is not None for x in results),
    final_filter_counts=dict(collections.Counter(k for x in results for k,n in x['filter_counts'].items() for _ in range(n))),
    ray_failure_counts=dict(collections.Counter(r['first_unsafe']['reason'] for x in results for r in x['rays'] if r['first_unsafe'])),
    current_heading_would_fit=sum(bool(x['clamp_counterfactual'] and x['clamp_counterfactual']['at_current_heading_clear']) for x in results),
    suppressed_visible_controls=sum(bool(x['visible_control'] and x['visible_control']['without_pending'] and not x['visible_control']['with_pending']) for x in results),results=results)
out=output_root/('standoff_filters_'+trial.name+'.json');out.write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))
print(json.dumps([{k:v for k,v in result.items() if k not in ('rays','accept_trace')} for result in results if result['visible_control'] and result['visible_control']['without_pending']],indent=2))
