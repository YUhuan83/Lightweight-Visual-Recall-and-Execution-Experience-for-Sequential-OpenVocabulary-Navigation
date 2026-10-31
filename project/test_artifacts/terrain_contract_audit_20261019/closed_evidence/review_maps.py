"""Replay closed raw LiDAR into both actual occupancy and gravity-level terrain."""
from pathlib import Path
import hashlib
import json
import math
import sqlite3
import subprocess
import sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from beamng_wildos_adapter.geometry import body_rotation,xyz
from beamng_wildos_adapter.pointcloud import decode_xyz,limit_points
from beamng_wildos_adapter.mapping import occupancy_from_points
from beamng_wildos_adapter.terrain import terrain_layers,grid_map_values
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
from wildos_nav2_adapter.sensor_geometry import SBR_LIDAR_OFFSET

root=Path(__file__).resolve().parent
run=Path('/home/ly/contact_frame_audit_20261019/frame_20261019_084254')
assert json.loads((run/'result.json').read_text())['component_closed']
assert (run/'interfaces/metadata.yaml').is_file()
db=next((run/'interfaces').glob('*.db3'))
assert hashlib.sha256(db.read_bytes()).hexdigest()=='0a1234726e95c0369bf11446701e2bb1ac16e3a92c4e397feea54d3419177dbc'
probe=json.loads((run/'contact_result.json').read_text())
body=[n for s in probe['contacts'] for n in (s['contacts']['nodes'] or []) if n['class']=='body']
peak=max(body,key=lambda n:n['max_normal_force'])
contact=np.array([peak['peak_reference']['node_world'][a] for a in ('x','y','z')])
anchor=json.loads((run/'sensor.json').read_text())['bridge_start_time_ros_ns']
con=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
topics={n:(i,t) for i,n,t in con.execute('SELECT id,name,type FROM topics')}
def load(name):
    i,typ=topics[name];cls=get_message(typ)
    return [(t,deserialize_message(data,cls)) for t,data in con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(i,))]
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
state={ns(m):m for t,m in load('/vehicles/ego/sensors/state')}
raw={ns(m):m for t,m in load('/vehicles/ego/sensors/front_lidar')}
local={ns(m):m for t,m in load('/spot1/local_map')}
terrain=load('/spot1/local_terrain')
timing={v['mapped_stamp_ns']:v for t,m in load('/spot1/measurement_timing')
        for v in [json.loads(m.data)] if v['sensor']=='lidar' and v.get('status')=='published'}
con.close()

def metrics(points):
    payload=''.join(' '.join(str(v) for v in p)+'\n' for p in points)
    response=subprocess.run([str(root/'terrain_metrics_probe')],input=payload,text=True,capture_output=True,check=True)
    value=json.loads(response.stdout)
    value['within_source_default_terrain_limits']=(value['slope']<=.35 and value['step']<=.1 and value['roughness']<=.08) if value['observed'] else None
    return value

controls={}
controls['inclined_plane']=metrics([(x,y,.1*x) for x in (-.5,0,.5) for y in (-.5,0,.5)])
assert controls['inclined_plane']['observed'] and abs(controls['inclined_plane']['slope']-.1)<1e-10 and controls['inclined_plane']['step']<1e-10
controls['isolated_30cm_step']=metrics([(x,y,.3 if x==0 and y==0 else 0) for x in (-.5,0,.5) for y in (-.5,0,.5)])
assert controls['isolated_30cm_step']['step']>.29 and controls['isolated_30cm_step']['within_source_default_terrain_limits'] is False
controls['insufficient_samples']=metrics([(0,0,0),(1,0,0),(0,1,0),(1,1,0)])
assert controls['insufficient_samples']['observed'] is False
controls['abrupt_17cm_half_plane']=metrics([(x,y,.17 if x>=0 else 0.) for x in (-.5,0,.5) for y in (-.5,0,.5)])
controls['abrupt_17cm_half_plane']['known_input_step_m']=.17
controls['abrupt_17cm_half_plane']['contract_gap_present']=(controls['abrupt_17cm_half_plane']['step']<.1 and controls['abrupt_17cm_half_plane']['within_source_default_terrain_limits'] is True)
assert controls['abrupt_17cm_half_plane']['contract_gap_present'],'Diagnostic expects the current unmodified step-contract gap'
controls['smooth_17percent_ramp']=metrics([(x,y,.17*(x+.5)) for x in (-.5,0,.5) for y in (-.5,0,.5)])
assert controls['smooth_17percent_ramp']['step']<1e-10 and controls['smooth_17percent_ramp']['within_source_default_terrain_limits'] is True

rows=[];unmatched=[];arrays={}
for index,(received,message) in enumerate(terrain):
    stamp=ns(message);tm=timing.get(stamp)
    if tm is None or tm['state_source_stamp_ns'] not in state or tm['source_stamp_ns'] not in raw or stamp not in local:
        unmatched.append(dict(stamp_ns=stamp,reason='matching recorded input missing'));continue
    sm=state[tm['state_source_stamp_ns']]
    world=limit_points(decode_xyz(raw[tm['source_stamp_ns']]),30000)
    body_rotation_matrix=body_rotation(sm.dir,sm.up);origin=xyz(sm.position)
    body_points=(world-origin)@body_rotation_matrix
    in_range=np.linalg.norm(body_points,axis=1)<=40.
    world=world[in_range];body_points=body_points[in_range]
    rear=origin+body_rotation_matrix@np.asarray(SBR_BASE_OFFSET)
    heading=math.atan2(body_rotation_matrix[1,0],body_rotation_matrix[0,0])
    level=np.array([[math.cos(heading),-math.sin(heading),0.],[math.sin(heading),math.cos(heading),0.],[0.,0.,1.]])
    level_points=(world-rear)@level
    sensor_world=origin+body_rotation_matrix@(np.asarray(SBR_BASE_OFFSET)+np.asarray(SBR_LIDAR_OFFSET))
    sensor_in_level=((sensor_world-rear)@level)[:2]
    replay_occ=occupancy_from_points(body_points-np.asarray(SBR_BASE_OFFSET),sensor_origin_xy=SBR_LIDAR_OFFSET[:2],clear_robot_footprint=True,carrier_model='ackermann')
    replay_layers=terrain_layers(level_points,sensor_origin_xy=sensor_in_level,carrier_model='ackermann')
    occ=local[stamp]
    observed_occ=np.asarray(occ.data,np.int8).reshape(occ.info.height,occ.info.width)
    actual_layers={name:np.asarray(data.data,np.float32).reshape(80,80)[::-1,::-1] for name,data in zip(message.layers,message.data)}
    differences={name:dict(unequal=int(np.count_nonzero(actual_layers[name]!=layer.astype(np.float32))),max_abs=float(np.max(np.abs(actual_layers[name]-layer.astype(np.float32))))) for name,layer in replay_layers.items()}
    contact_body=(contact-origin)@body_rotation_matrix-np.asarray(SBR_BASE_OFFSET)
    contact_level=(contact-rear)@level
    def cell(point):return np.floor(point[:2]/.5+[40,40]).astype(int)
    cx,cy=cell(contact_level);ox,oy=cell(contact_body)
    assert 1<=cx<79 and 1<=cy<79
    samples=[];patch={}
    for name,layer in actual_layers.items():patch[name]=layer[cy-1:cy+2,cx-1:cx+2].tolist()
    if actual_layers['elevation_valid'][cy,cx]>.5:
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                if actual_layers['elevation_valid'][cy+dy,cx+dx]>.5:
                    samples.append((.5*dx,.5*dy,float(actual_layers['elevation'][cy+dy,cx+dx])))
    item=dict(stamp_ns=stamp,engine_seconds=(tm['state_source_stamp_ns']-anchor)*1e-9,
        local_map_frame=occ.header.frame_id,terrain_frame=message.header.frame_id,
        occupancy_replay_unequal=int(np.count_nonzero(observed_occ!=replay_occ)),terrain_replay_difference=differences,
        contact_body=contact_body.tolist(),contact_level=contact_level.tolist(),
        body_cell=[int(ox),int(oy)],level_cell=[int(cx),int(cy)],
        body_occupancy=int(observed_occ[oy,ox]),
        terrain_contact={name:float(layer[cy,cx]) for name,layer in actual_layers.items()},
        patch=patch,patch_metrics=metrics(samples),patch_samples=samples)
    rows.append(item)
    if len(rows)<=3 or abs(item['engine_seconds']-peak['peak_sim_time'])<.5:
        arrays['sample'+str(index)+'_occupancy']=observed_occ
        for name,layer in actual_layers.items():arrays['sample'+str(index)+'_'+name]=layer

source_identity={}
manifest=json.loads(Path('/home/ly/smoothing_boundary_repair_20261018/official_wildos_beamng_bundle.manifest.json').read_text())
for module_name in ('beamng_wildos_adapter.mapping','beamng_wildos_adapter.terrain','beamng_wildos_adapter.pointcloud'):
    source=Path(sys.modules[module_name].__file__);sha=hashlib.sha256(source.read_bytes()).hexdigest()
    member='official_adapter/'+module_name.replace('.','/')+'.py'
    assert sha==manifest['members'][member]
    source_identity[module_name]=dict(path=str(source),sha256=sha,equals_frozen=True)
report=dict(scope=__doc__,source_run=str(run),database_sha256=hashlib.sha256(db.read_bytes()).hexdigest(),
    contact_world=contact.tolist(),source_identity=source_identity,cpp_controls=controls,
    snapshots=rows,unmatched=unmatched,all_recorded_occupancy_replayed_exactly=all(r['occupancy_replay_unequal']==0 for r in rows),
    all_recorded_terrain_replayed_exactly=all(v['unequal']==0 for r in rows for v in r['terrain_replay_difference'].values()),
    actual_graph_node_or_edge_replayed=False,source_default_thresholds_not_live_parameter_read=True,
    production_modified=False,complete_navigation_passed=False)
np.savez_compressed(root/'selected_maps.npz',**arrays)
(root/'map_contract_review.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='snapshots'},indent=2))
print(json.dumps([dict(engine_seconds=r['engine_seconds'],occupancy=r['body_occupancy'],terrain=r['terrain_contact'],metrics=r['patch_metrics']) for r in rows],indent=2))
