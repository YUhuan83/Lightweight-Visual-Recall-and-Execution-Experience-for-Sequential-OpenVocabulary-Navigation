"""Start the frozen search stack after a recorded fresh-scene stationary check."""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

BASE=Path('/home/ly/town_memory_validation_20261014')
OUT=BASE/'green_house_session'
CANDIDATE=Path('/home/ly/town_memory_validation_20261013/navigation_transactions_candidate')
BUNDLE=CANDIDATE/'bundle'
INSTALL=CANDIDATE/'install'
VENV='/home/ly/beamng_ros2_0394_ws/.venv/bin/python'
VISION='/home/ly/vision-cu128/bin/python'


def spawn(name, command):
    with (OUT/f'{name}.log').open('xb') as log:
        child=subprocess.Popen(command,cwd=BUNDLE,env=os.environ.copy(),stdin=subprocess.DEVNULL,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (OUT/f'{name}.pid').write_text(str(child.pid)+'\n')
    return child


def main():
    OUT.mkdir(exist_ok=False)
    report={'passed':False,'production_changed':False,'map_reload_count':0,'teleports':0,'pids':{}}
    bng=None
    try:
        scene=json.loads((BASE/'scene_preflight.json').read_text())
        brake=json.loads((BASE/'brake_spawn.json').read_text())
        report['initial_scene_passed']=scene.get('passed')
        report['initial_scene_error']=scene.get('error')
        if brake.get('passed') is not True:
            raise RuntimeError('Required stationary preparation failed')
        if scene.get('map_reload_count') != 1 or scene.get('collision_override_requested') is not False:
            raise RuntimeError('Fresh same-configuration scene provenance missing')
        digest=hashlib.sha256((CANDIDATE/'candidate.zip').read_bytes()).hexdigest()
        report['candidate_sha256']=digest
        if digest!='f9561be85a67cc1b47044f090e8fbb80cda00e14fbb0b0bf52e7c49d116e526f':
            raise RuntimeError('Frozen archive differs')
        report['installed_source_sha256']={}
        for rel in ('graph_path_bridge.py','continuous_path.py','navigation_transactions.py',
                    'memory_selection.py','cmd_vel_to_control.py','native_vehicle_control.py',
                    'execution_feedback.py','vehicle_control_lease.lua'):
            expected=hashlib.sha256((BUNDLE/'nav2_adapter/wildos_nav2_adapter'/rel).read_bytes()).hexdigest()
            actual=hashlib.sha256((INSTALL/'lib/python3.10/site-packages/wildos_nav2_adapter'/rel).read_bytes()).hexdigest()
            if expected!=actual: raise RuntimeError('Frozen installation differs: '+rel)
            report['installed_source_sha256'][rel]=actual
        from beamngpy import BeamNGpy
        from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
        from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
        from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
        install_fail_closed_reads(PrefixedLengthSocket)
        bng=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
        before=read_physics_snapshot(bng)
        report['before']=before
        errors=physical_snapshot_errors(before,require_sensors=False)
        if (errors or before['speed']>.05 or before.get('cameras') or before.get('lidars')
                or 'east_coast_usa' not in before['mission']
                or math.dist(before['position'][:2],(734.294,-11.946))>1.25):
            raise RuntimeError('Stationary clean physical scene required: '+str(errors))
        (BASE/'scene_after_settle.json').write_text(json.dumps({
            'passed':True,'physics_snapshot':before,'map_reload_count':0,
            'initial_scene_report':'scene_preflight.json','stationary_report':'brake_spawn.json',
            'note':'Fresh read after settling; earlier instant-spawn failure remains unchanged.'},indent=2)+'\n')
        engine_pid=int((BASE/'beamng.pid').read_text())
        p=spawn('sensor',[VENV,'tools/attach_current_vehicle_sensors.py','--rig',
            'config/sbr_physical_sensor_rig.json','--output',str(OUT/'sensor_attachment.json'),
            '--engine-pid',str(engine_pid),'--resume'])
        report['pids']['sensor']=p.pid
        deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            if p.poll() is not None: raise RuntimeError('Sensor owner exited')
            if (OUT/'sensor_attachment.json').exists():
                sensor=json.loads((OUT/'sensor_attachment.json').read_text())
                if sensor.get('ready'): break
            time.sleep(.2)
        else: raise RuntimeError('Sensor attachment timed out')
        port=sensor['vehicle_port']; report['vehicle_port']=port
        p=spawn('stack',['/opt/ros/humble/bin/ros2','launch','wildos_nav2_adapter',
            'physical_vehicle_control.launch.py',f'vehicle_port:={port}'])
        report['pids']['stack']=p.pid
        tri=INSTALL/'lib/python3.10/site-packages/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
        p=spawn('triangulation',[VISION,str(tri),'--config',
            str(INSTALL/'share/beamng_wildos_adapter/config/beamng_triangulation.yaml')])
        report['pids']['triangulation']=p.pid
        p=spawn('vision',[VISION,'-m','visual_navigation.wildos.nav','--config','beamng_wildos',
            '--do_object_search','true','--memory_in_process_only'])
        report['pids']['vision']=p.pid
        report['passed']=True
    except BaseException as exc:
        report['error']=f'{type(exc).__name__}: {exc}'
        if bng:
            try: bng.control.pause()
            except Exception as pause: report['pause_error']=repr(pause)
    finally:
        if bng: bng.disconnect()
        (OUT/'stack_preparation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('before','installed_source_sha256')},indent=2),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
