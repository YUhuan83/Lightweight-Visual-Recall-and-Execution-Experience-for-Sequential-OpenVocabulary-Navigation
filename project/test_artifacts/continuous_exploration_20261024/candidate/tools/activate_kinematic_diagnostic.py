"""Server-side reversible fixed-node diagnostic setup on the loaded Town map.

Pause first, stop the specific current controllers and sensor owner, respawn
only ego, recreate sensors, and route actual pose into the official adapter.
Writes the original config/process manifest for restoration. No task success
is inferred from this setup or its short straight-line motion probe.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from beamngpy import BeamNGpy
from beamngpy.connection import Connection

ROOT = Path('/home/ly/town_memory_validation_20261003')
OUT = ROOT/'physics_isolation_20261006'
OUT.mkdir(exist_ok=True)
rows = {r['name']:r for r in json.loads((ROOT/'restart_snapshot.json').read_text())}
env = dict(rows['official_adapter']['environment'])
env.update(ROS_DOMAIN_ID='40', ROS_LOCALHOST_ONLY='1')
bng = BeamNGpy('127.0.0.1',25252,socket_timeout=8).open(launch=False)
report = {'map_reload_count':0, 'applied':False}
if (OUT/'vehicle_port.json').exists():
    report['known_vehicle_port'] = json.loads((OUT/'vehicle_port.json').read_text())['port']
elif (OUT/'application.json').exists():
    old_report = json.loads((OUT/'application.json').read_text())
    if 'known_vehicle_port' in old_report:
        report['known_vehicle_port'] = old_report['known_vehicle_port']


def lua(code):
    return bng.control.queue_lua_command(code,response=True)


def stop_pid(pid, identity):
    path = Path('/proc')/str(pid)
    if not path.exists():
        return
    assert identity.encode() in (path/'cmdline').read_bytes(), 'Process identity mismatch'
    os.kill(pid,signal.SIGINT)
    end = time.monotonic()+15
    while path.exists() and 'State:\tZ' not in (path/'status').read_text():
        if time.monotonic()>end:
            raise RuntimeError('Process did not stop: '+identity)
        time.sleep(.2)


def start_row(name, extra=()):
    row = rows[name]
    with (OUT/(name+'.log')).open('a') as log:
        proc = subprocess.Popen(row['args']+list(extra),cwd=row['cwd'],env=env,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (ROOT/(name+'.pid')).write_text(str(proc.pid))
    time.sleep(1)
    assert proc.poll() is None, name+' failed to start'
    return proc.pid


try:
    bng.control.pause()
    before = json.loads(lua("local v=scenetree.findObject('ego'); assert(v); "
        "local d=core_vehicle_manager.getVehicleData(v:getID()); "
        "return jsonEncode({config=d.config,position=v:getPosition():toTable(),"
        "rotation=quat(v:getClusterRotationSlow(v:getRefNodeId())):toTable(),"
        "mission=getMissionFilename(),cameras=extensions.tech_sensors.getActiveCameraSensors(),"
        "lidars=extensions.tech_sensors.getActiveLidarSensors()})"))
    if (OUT/'before.json').exists():
        saved = json.loads((OUT/'before.json').read_text())
        assert before['config'].get('wildosKinematicProfile') in (None,'all-fixed-v1'), 'Unexpected profile'
        before = saved
    else:
        (OUT/'before.json').write_text(json.dumps(before,indent=2))
        (OUT/'process_manifest.json').write_text(json.dumps(rows,indent=2))
    # Identify the actuator independently of old saved PID files.
    native = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            cmd = (path/'cmdline').read_bytes()
        except OSError:
            continue
        if b'/lib/wildos_nav2_adapter/native_vehicle_control\x00' in cmd:
            native.append({'pid':int(path.name),'args':[s.decode() for s in cmd.split(b'\x00') if s]})
    if native:
        (OUT/'native_processes.json').write_text(json.dumps(native,indent=2))
    vehicle = bng.vehicles.get_current()['ego']
    if 'known_vehicle_port' in report:
        vehicle.connection = Connection('127.0.0.1',report['known_vehicle_port'],timeout=8)
    vehicle.connect(bng)
    known_port = vehicle.connection.port
    vehicle.disconnect()
    report['known_vehicle_port'] = known_port
    (OUT/'vehicle_port.json').write_text(json.dumps({'port':known_port}))
    for name in ('graph_bridge','speed_control','official_adapter','sensor_bridge'):
        pid_file = ROOT/(name+'.pid')
        pid = int(pid_file.read_text()) if pid_file.exists() else int(rows[name]['pid'])
        # A repeated diagnostic setup may already own a generated sensor script.
        if name == 'sensor_bridge' and Path('/proc',str(pid),'cmdline').exists():
            current = Path('/proc',str(pid),'cmdline').read_bytes()
            identity = str(OUT/'attach_fixed_vehicle.py') if str(OUT/'attach_fixed_vehicle.py').encode() in current else rows[name]['args'][1]
        else:
            identity = rows[name]['args'][1]
        stop_pid(pid,identity)
    # The original bridge was started directly, not by the older attach helper.
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            cmd = (path/'cmdline').read_bytes()
        except OSError:
            continue
        if b'/lib/beamng_ros2/beamng_bridge\x00' in cmd:
            stop_pid(int(path.name),'/lib/beamng_ros2/beamng_bridge')
    for item in native:
        stop_pid(item['pid'],'native_vehicle_control')
    # Respawning ego does not necessarily remove old tech sensor handles.
    # Their owner is now stopped; remove ego's sensors before recreating them.
    lua("local v=scenetree.findObject('ego'); extensions.tech_sensors.removeAllSensorsFromVehicle(v:getID()); return true")
    source = Path('/home/ly/wildosKinematic.lua').read_text()
    # JSON string escaping is Lua-compatible for this ASCII-only source.
    lua('local source='+json.dumps(source)+'; assert(loadstring(source)); '
        "FS:directoryCreate('lua/ge/extensions'); writeFile('lua/ge/extensions/wildosKinematic.lua',source); "
        "extensions.unload('wildosKinematic'); extensions.load('wildosKinematic'); assert(extensions.wildosKinematic); return true")
    lua("local v=scenetree.findObject('ego'); "
        "core_vehicle_partmgmt.setConfigOfVehicle(v,{wildosKinematicProfile='all-fixed-v1'},true); return true")
    time.sleep(2)
    lua("local v=scenetree.findObject('ego'); "
        "extensions.wildosKinematic.start(v:getID(),-2.172,0.330); return true")
    report['fixed_state'] = json.loads(lua('return jsonEncode(extensions.wildosKinematic.status())'))
    assert report['fixed_state']['mission']==before['mission']
    masks = json.loads(lua("local v=scenetree.findObject('ego'); local d=core_vehicle_manager.getVehicleData(v:getID()); "
        "local masks={}; for _,n in pairs(d.vdata.nodes) do local m=v:getInitialNodeCollision(n.cid); "
        "masks[tostring(m)]=(masks[tostring(m)] or 0)+1 end; return jsonEncode(masks)"))
    report['collision_masks'] = masks
    assert set(masks)=={'0'}, 'Some engine collisions remain enabled'
    attach = Path('/home/ly/attach_current_town_bridge.py').read_text()
    attach = attach.replace("profile=json.loads((ROOT/'body_collision_full/profile/application.json').read_text())",
                            'profile='+repr({'known_vehicle_port':known_port}))
    attach = attach.replace('b.control.resume()', 'b.control.pause()')
    (OUT/'attach_fixed_vehicle.py').write_text(attach)
    rows['sensor_bridge']['args'] = [sys.executable,str(OUT/'attach_fixed_vehicle.py')]
    start_row('sensor_bridge')
    time.sleep(2)
    report['sensors'] = json.loads(lua('return jsonEncode(extensions.wildosKinematic.status())'))
    assert report['sensors']['cameras'] and report['sensors']['lidars'], 'Sensors not recreated'
    assert len(report['sensors']['cameras'])==1 and len(report['sensors']['lidars'])==1, 'Duplicate sensors'
    # Check that this engine can actually move its fixed-node cluster.
    origin = report['sensors']['position']
    bng.control.resume()
    samples = []
    for velocity in (1.2,-1.2):
        end = time.monotonic()+1.5
        while time.monotonic()<end:
            samples.append(json.loads(lua(f'extensions.wildosKinematic.command({velocity},0); '
                'return jsonEncode(extensions.wildosKinematic.status())')))
            time.sleep(.05)
        lua('extensions.wildosKinematic.stop(); return true')
    bng.control.pause()
    report['probe_samples'] = samples
    report['maximum_probe_displacement_m'] = max(sum((a-b)**2 for a,b in zip(s['position'][:2],origin[:2]))**.5 for s in samples)
    assert report['maximum_probe_displacement_m']>.2, 'Fixed vehicle did not move'
    start_row('official_adapter',('--ros-args','-r','/vehicles/ego/sensors/state:=/wildos/kinematic_state'))
    with (OUT/'kinematic.log').open('a') as log:
        proc = subprocess.Popen([sys.executable,'-m','wildos_nav2_adapter.kinematic_vehicle_control',
            '--ros-args','-p','enable:=false'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (OUT/'kinematic.pid').write_text(str(proc.pid))
    time.sleep(2)
    assert proc.poll() is None, 'Kinematic ROS backend failed to start'
    report['kinematic_pid'] = proc.pid
    # Keep follow control stopped until a diagnostic task explicitly starts.
    report['applied'] = True
    report['simulation_paused'] = True
    report['height_policy'] = 'Initial height and pitch/roll held; flat-area diagnostic only'
except BaseException as exc:
    report['error'] = repr(exc)
    raise
finally:
    try:
        bng.control.pause()
    finally:
        bng.disconnect()
        (OUT/'application.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('probe_samples','fixed_state','sensors')}))
