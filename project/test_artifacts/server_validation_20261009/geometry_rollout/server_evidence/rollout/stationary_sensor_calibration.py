"""Read-only actual sensor calibration check after the test has stopped."""
import json,pathlib,time,numpy as np
from types import SimpleNamespace as S
from beamngpy import BeamNGpy
from beamngpy.sensors import Camera,Lidar
from beamng_wildos_adapter.geometry import body_rotation
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
p=pathlib.Path('/home/ly/town_memory_validation_20261009')
assert p.exists()
b=BeamNGpy('127.0.0.1',25252,socket_timeout=5);b.open(launch=False)
cam=Camera.__new__(Camera);cam.name='front_cam';cam.bng=b
lid=Lidar.__new__(Lidar);lid.name='front_lidar';lid.bng=b
rows=[]
try:
 for i in range(3):
  t0=time.time_ns();c=np.array(cam.get_position());l=np.array(lid.get_position());direction=list(cam.get_direction())
  d=json.loads(b.control.queue_lua_command("local v=scenetree.findObject('ego');local p=v:getPosition();local f=v:getDirectionVector();local u=v:getDirectionVectorUp();return jsonEncode({position={p.x,p.y,p.z},direction={f.x,f.y,f.z},up={u.x,u.y,u.z}})",response=True))
  r=body_rotation(S(x=d['direction'][0],y=d['direction'][1],z=d['direction'][2]),S(x=d['up'][0],y=d['up'][1],z=d['up'][2]));native=np.array(d['position']);nav=native+r@np.array(SBR_BASE_OFFSET);offset=r.T@(c-nav)
  rows.append(dict(wall_ns=t0,camera_actual=c.tolist(),lidar_actual=l.tolist(),native_position=native.tolist(),native_direction=d['direction'],camera_direction=direction,nav_position=nav.tolist(),camera_actual_body_offset=offset.tolist(),camera_configured_body_offset=[2.751995092,0.000008318,2.209999084],camera_position_error_m=float(np.linalg.norm(offset-np.array([2.751995092,0.000008318,2.209999084])))))
  time.sleep(.2)
finally:b.disconnect()
(p/'stationary_sensor_calibration.json').write_text(json.dumps({'scope':'Three read-only BeamNG GE samples at Town spawn before the sequence with motion disabled','samples':rows},indent=2));print(json.dumps(rows))
