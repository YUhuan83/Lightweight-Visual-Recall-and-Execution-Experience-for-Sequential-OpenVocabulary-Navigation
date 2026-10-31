from pathlib import Path
import os,signal,subprocess,time,json
ROOT=Path('/home/ly/town_memory_validation_20261009');OLD=Path('/home/ly/town_memory_validation_20261008')
rows=[]
def stop(pid,marker):
 p=Path('/proc')/str(pid)
 if not p.exists():return
 assert marker in (p/'cmdline').read_bytes(), 'Unexpected identity'
 rows.append({'pid':pid,'identity_marker':marker.decode()})
 os.kill(pid,signal.SIGTERM)
 until=time.monotonic()+5
 while p.exists() and (p/'cmdline').read_bytes() and time.monotonic()<until:time.sleep(.1)
 if p.exists() and (p/'cmdline').read_bytes():os.kill(pid,signal.SIGKILL)
for p in Path('/proc').iterdir():
 if p.name.isdigit():
  try:
   tokens=(p/'cmdline').read_bytes().split(b'\0')
   if b'/home/ly/town_memory_validation_20261008/sensor_bridge.py' in tokens:stop(int(p.name),b'/home/ly/town_memory_validation_20261008/sensor_bridge.py')
  except FileNotFoundError:pass
stop(799609,b'6744dc1e-efd4-412d-373adeb6-150a2c8a.dmp')
stop(474590,b'/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0/BinLinux/BeamNG.tech.x64')
(ROOT/'simulation_recovery.json').write_text(json.dumps({'stopped':rows,'scope':'Recovery before a fresh test; not a reload during the target sequence'},indent=2))
print('CRASHED_PROCESSES_STOPPED',flush=True)
from beamngpy import BeamNGpy,Scenario,Vehicle
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
install_fail_closed_reads(PrefixedLengthSocket)
b=BeamNGpy('127.0.0.1',25252,home='/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0',user='/home/ly/beamng-east-coast-user/current',headless=True,quit_on_close=False,socket_timeout=20)
b.open(launch=True,listen_ip='127.0.0.1')
print('BEAMNG_HANDSHAKE_READY',flush=True)
spec=json.loads(Path('/home/ly/full_town_memory_20261002/east_coast_town_reconnect.json').read_text());v=spec['vehicles'][0]
s=Scenario(spec['level'],'wildos_town_recovery_20261009');car=Vehicle(v['name'],v['model'],part_config=v['part_config'],color=v['color'])
s.add_vehicle(car,pos=v['pos'],rot_quat=v['rot']);s.make(b)
b.scenario.load(s,connect_player_vehicle=False,connect_existing_vehicles=False)
b.scenario.start();b.env.set_tod(spec['time_of_day']);b.control.pause()
print('TOWN_SCENARIO_READY',flush=True)
# Record the protocol's actual returned port rather than recycling last run's port.
endpoint=b.vehicles.start_connection(car,car.extensions);port=int(endpoint.get('result') or endpoint.get('port'))
r={'stopped':rows,'new_command':b.last_command_line,'vehicle_port':port,'scenario':s.name,'before_sequence_map_reload':1,'motion_controls_enabled':False}
(ROOT/'simulation_recovery.json').write_text(json.dumps(r,indent=2));b.disconnect();print('RECOVERY_COMPLETE',port,flush=True)
