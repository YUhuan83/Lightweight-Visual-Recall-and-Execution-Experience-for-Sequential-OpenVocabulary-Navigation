"""Read closed raw data into bounded, provenance-preserving replay arrays; no ROS nodes/vehicle."""
from pathlib import Path
import hashlib,importlib,json,math,sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from beamng_wildos_adapter.geometry import body_rotation,xyz
from beamng_wildos_adapter.pointcloud import decode_xyz,limit_points
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
from wildos_nav2_adapter.sensor_geometry import SBR_LIDAR_OFFSET

out=Path(__file__).resolve().parent
run=Path('/home/ly/contact_frame_audit_20261019/frame_20261019_084254')
assert json.loads((run/'result.json').read_text())['component_closed']
db=next((run/'interfaces').glob('*.db3'));assert (run/'interfaces/metadata.yaml').is_file()
digest=hashlib.sha256(db.read_bytes()).hexdigest()
assert digest=='0a1234726e95c0369bf11446701e2bb1ac16e3a92c4e397feea54d3419177dbc'
contact=json.loads((run/'contact_result.json').read_text())
peak=max((n for s in contact['contacts'] for n in s['contacts']['nodes'] or [] if n['class']=='body'),key=lambda n:n['max_normal_force'])
world_contact=np.array([peak['peak_reference']['node_world'][a] for a in ('x','y','z')])
anchor=json.loads((run/'sensor.json').read_text())['bridge_start_time_ros_ns']
con=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
topics={n:(i,t) for i,n,t in con.execute('SELECT id,name,type FROM topics')}
def load(name):
    ident,typ=topics[name];cls=get_message(typ)
    return [(t,deserialize_message(b,cls)) for t,b in con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(ident,))]
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
states={ns(m):m for _,m in load('/vehicles/ego/sensors/state')}
raw={ns(m):m for _,m in load('/vehicles/ego/sensors/front_lidar')}
terrain=load('/spot1/local_terrain')
timing={v['mapped_stamp_ns']:v for _,m in load('/spot1/measurement_timing')
        for v in [json.loads(m.data)] if v['sensor']=='lidar' and v.get('status')=='published'}
con.close();arrays={};rows=[]
for index,(_,message) in enumerate(terrain):
    stamp=ns(message);info=timing[stamp]
    state=states[info['state_source_stamp_ns']]
    world=limit_points(decode_xyz(raw[info['source_stamp_ns']]),30000)
    rotation=body_rotation(state.dir,state.up);position=xyz(state.position)
    body=(world-position)@rotation;inside=np.linalg.norm(body,axis=1)<=40.
    world=world[inside];body=body[inside]-np.asarray(SBR_BASE_OFFSET)
    rear=position+rotation@np.asarray(SBR_BASE_OFFSET)
    heading=math.atan2(rotation[1,0],rotation[0,0])
    level=np.array([[math.cos(heading),-math.sin(heading),0.],[math.sin(heading),math.cos(heading),0.],[0.,0.,1.]])
    grounded=(world-rear)@level
    sensor_world=position+rotation@(np.asarray(SBR_BASE_OFFSET)+np.asarray(SBR_LIDAR_OFFSET))
    key=f'scan_{index:02d}'
    arrays[key+'_body']=body.astype(np.float32)
    arrays[key+'_level']=grounded.astype(np.float32)
    arrays[key+'_contact_body']=((world_contact-position)@rotation-np.asarray(SBR_BASE_OFFSET))
    arrays[key+'_contact_level']=(world_contact-rear)@level
    arrays[key+'_sensor_level']=(sensor_world-rear)@level
    for name,data in zip(message.layers,message.data):arrays[key+'_recorded_'+name]=np.asarray(data.data,np.float32).reshape(80,80)[::-1,::-1]
    rows.append(dict(key=key,source_stamp_ns=info['source_stamp_ns'],state_source_stamp_ns=info['state_source_stamp_ns'],
        mapped_stamp_ns=stamp,engine_seconds=(info['state_source_stamp_ns']-anchor)*1e-9,points=len(body)))
assert len(rows)==22,len(rows)
target=out/'replay_inputs.npz';assert not target.exists()
np.savez_compressed(target,**arrays)
identities={}
for name in ('beamng_wildos_adapter.geometry','beamng_wildos_adapter.pointcloud'):
    p=Path(importlib.import_module(name).__file__)
    identities[name]=dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
report=dict(scope=__doc__,database=dict(path=str(db),sha256=digest),snapshots=rows,contact_world=world_contact.tolist(),
    input_modules=identities,archive_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
    archive_bytes=target.stat().st_size,production_modified=False,vehicle_started=False,
    true_acquisition_timestamp_proven=False)
(out/'input_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='snapshots'},indent=2))
