import json,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from beamngpy import BeamNGpy
from beamngpy.connection.comm_base import CommBase
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
from beamng_wildos_adapter.geometry import body_rotation
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
install_fail_closed_reads(PrefixedLengthSocket)
b=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
lua="""local v=scenetree.findObject('ego'); assert(v); local p=v:getPosition(); local d=v:getDirectionVector(); local u=v:getDirectionVectorUp(); return jsonEncode({position={p.x,p.y,p.z},direction={d.x,d.y,d.z},up={u.x,u.y,u.z}})"""
def pose():return json.loads(b.control.queue_lua_command(lua,response=True))
def vec(a):return SimpleNamespace(x=a[0],y=a[1],z=a[2])
rows=[]
try:
 for i in range(30):
  before=pose();sensor=CommBase._send_recv(b.connection,'GetLidarSensorPosition',name='front_lidar')['data'];after=pose()
  p=np.array([sensor[n] for n in 'xyz']);rotation=body_rotation(vec(before['direction']),vec(before['up']))
  offset=(p-np.array(before['position']))@rotation-np.array(SBR_BASE_OFFSET)
  rows.append({'before':before,'sensor_world':sensor,'after':after,'pose_translation_between_reads_m':float(np.linalg.norm(np.array(before['position'])-after['position'])),'lidar_offset_in_base':offset.tolist()})
  time.sleep(.1)
finally:b.disconnect()
p=Path('/home/ly/full_score_search_20261015/lidar_origin_observation.json');p.write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
