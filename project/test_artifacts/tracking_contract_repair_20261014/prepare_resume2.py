"""Owned same-map preparation. No model process or vehicle command gate."""
import hashlib,json,math,os,signal,subprocess,time
from pathlib import Path
BASE=Path('/home/ly/town_memory_validation_20261014/tracking_contract_candidate')
OUT=BASE/'resume_session2'
BUNDLE=BASE/'bundle'
PY='/home/ly/beamng_ros2_0394_ws/.venv/bin/python'

def spawn(role,args):
    with (OUT/(role+'.log')).open('xb') as log:
        p=subprocess.Popen(args,cwd=BUNDLE,stdin=subprocess.DEVNULL,stdout=log,
            stderr=subprocess.STDOUT,start_new_session=True)
    (OUT/(role+'.pid')).write_text(str(p.pid)+'\n')
    return p

def main():
    OUT.mkdir(exist_ok=False)
    report={'passed':False,'map_reload_count':0,'preparation_teleports':0,'pids':{}}
    bng=None
    try:
        assert hashlib.sha256((BASE/'candidate.zip').read_bytes()).hexdigest()==\
            '46ff51f2355a15fb4f92c79299b774d532dd72432380420e72dae7613978be8e'
        assert json.loads((BASE/'installation_identity.json').read_text())['passed']
        from server_test_preflight import process_rows
        rows=process_rows()
        for role in ('graph_path_bridge','native_vehicle_control','cmd_vel_to_control',
                     'kinematic_vehicle_control','attach_current_vehicle_sensors.py','visual_navigation.wildos.nav'):
            assert not any(role in r['tokens'] or any(Path(t).name==role for t in r['tokens'])
                           for r in rows if r['domain']=='88'),'old owner: '+role
        assert Path('/proc/1709748/cmdline').read_bytes().find(b'BeamNG.tech.x64')>=0
        from beamngpy import BeamNGpy
        from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
        from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
        from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
        install_fail_closed_reads(PrefixedLengthSocket)
        bng=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
        before=read_physics_snapshot(bng);report['before']=before
        assert not physical_snapshot_errors(before,require_sensors=False)
        assert before['speed']<.05 and not before.get('cameras') and not before.get('lidars')
        assert 'east_coast_usa' in before['mission']
        bng.control.pause()
        assert bng.vehicles.teleport('ego',(734.294,-11.946,52.154),
            rot_quat=(0.,0.,.7658087977066911,.6430683364581344),reset=True,safe_spawn=False)
        report['preparation_teleports']=1
        bng.control.resume();bng.disconnect();bng=None
        brake=subprocess.run([PY,str(BASE/'component_tools/brake_and_hold_spawn.py'),
            '--output',str(OUT/'brake_preparation.json')],text=True,capture_output=True,timeout=30)
        (OUT/'brake_preparation.log').write_text(brake.stdout+brake.stderr)
        assert brake.returncode==0,'fresh brake preparation failed'
        bng=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
        after=read_physics_snapshot(bng);report['after_preparation']=after
        assert not physical_snapshot_errors(after,require_sensors=False)
        assert after['mission']==before['mission'] and after['speed']<.05
        assert math.dist(after['position'][:2],(734.294,-11.946))<1.25
        p=spawn('sensor',[PY,'tools/attach_current_vehicle_sensors.py','--rig',
            'config/sbr_physical_sensor_rig.json','--output',str(OUT/'sensor_attachment.json'),
            '--engine-pid','1709748','--resume'])
        report['pids']['sensor']=p.pid
        deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            assert p.poll() is None,'sensor exited'
            if (OUT/'sensor_attachment.json').exists():
                sensor=json.loads((OUT/'sensor_attachment.json').read_text())
                if sensor.get('ready'):break
            time.sleep(.2)
        else:raise RuntimeError('sensor preparation timeout')
        p=spawn('stack',['/opt/ros/humble/bin/ros2','launch','wildos_nav2_adapter',
            'physical_vehicle_control.launch.py','vehicle_port:='+str(sensor['vehicle_port'])])
        report['pids']['stack']=p.pid
        # The component publishes its own stamped route/metadata. Retire only
        # the new stack's GraphNav planner, using exact owner and group checks.
        deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            assert p.poll() is None,'stack exited'
            matches=[r for r in process_rows() if r['domain']=='88' and
                any(Path(t).name=='planner_node' and t.startswith(str(BASE/'install')) for t in r['tokens'])]
            if len(matches)==1:break
            time.sleep(.2)
        else:raise RuntimeError('owned graph planner not found')
        owner=matches[0];assert os.getpgid(owner['pid'])==p.pid
        os.kill(owner['pid'],signal.SIGINT)
        report['retired_component_route_owner']=owner['pid']
        report['passed']=True
    except BaseException as exc:
        report['error']=type(exc).__name__+': '+str(exc)
        if bng:
            try:bng.control.pause()
            except Exception as error:report['pause_error']=repr(error)
    finally:
        if bng:bng.disconnect()
        (OUT/'preparation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('before','after_preparation')},indent=2))
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
