from pathlib import Path
import json,os,signal,time,subprocess
from beamngpy import BeamNGpy
root=Path('/home/ly/town_memory_validation_20261009');old=Path('/home/ly/town_memory_validation_20261008')
for pidfile,marker in [(root/'sensor_bridge.pid',b'/home/ly/town_memory_validation_20261009/sensor_bridge.py'),(old/'kinematic.pid',b'wildos_nav2_adapter.kinematic_vehicle_control')]:
 pid=int(pidfile.read_text());p=Path('/proc')/str(pid)
 if p.exists() and (p/'cmdline').read_bytes():
  assert marker in (p/'cmdline').read_bytes();os.kill(pid,signal.SIGINT)
  until=time.monotonic()+10
  while p.exists() and (p/'cmdline').read_bytes() and time.monotonic()<until:time.sleep(.1)
  assert not p.exists() or not (p/'cmdline').read_bytes(),'Component did not stop'
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8);b.open(launch=False)
try:
 b.control.pause()
 b.control.queue_lua_command("local v=scenetree.findObject('ego'); extensions.tech_sensors.removeAllSensorsFromVehicle(v:getID()); core_vehicle_partmgmt.setConfigOfVehicle(v,{wildosKinematicProfile='all-fixed-v1'},true); return true",response=True)
 time.sleep(2)
 report=json.loads(b.control.queue_lua_command("local v=scenetree.findObject('ego'); local d=core_vehicle_manager.getVehicleData(v:getID()); local n,f,m=0,0,{}; for _,x in pairs(d.vdata.nodes) do n=n+1; if x.fixed then f=f+1 end; local mask=v:getInitialNodeCollision(x.cid); m[tostring(mask)]=(m[tostring(mask)] or 0)+1 end; return jsonEncode({profile=d.config.wildosKinematicProfile,nodes=n,fixed=f,collisionMasks=m})",response=True))
 assert report['nodes']==report['fixed'] and report['collisionMasks'].get('0')==report['nodes']
 b.vehicles.teleport('ego',(734.294,-11.946,52.154),(0.,0.,0.7658087977066911,0.6430683364581344),reset=True)
 (root/'profile_after_restore.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
finally:b.disconnect()
with (root/'sensor_bridge.log').open('a') as log:
 p=subprocess.Popen(['/home/ly/beamng_ros2_0394_ws/.venv/bin/python',str(root/'sensor_bridge.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(root/'sensor_bridge.pid').write_text(str(p.pid));time.sleep(2);assert p.poll() is None
r=subprocess.run(['/home/ly/beamng_ros2_0394_ws/.venv/bin/python',str(old/'restart_component.py'),'kinematic'],capture_output=True,text=True,timeout=30);print(r.stdout,r.stderr,flush=True);assert not r.returncode
