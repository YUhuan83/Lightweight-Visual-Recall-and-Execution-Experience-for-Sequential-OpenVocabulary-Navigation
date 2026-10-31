"""Single-map real Native timing diagnostic; no planner, throttle, vision model or video."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
sys.path.insert(0,'/home/ly/planning_phase_physical_20261018/candidate/tools')
from wildos_console import OwnedProcesses
from check_continuous_start_execution import settle

root=Path('/home/ly/native_delay_20261018')
out=root/('physical_brake_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir()
home=Path('/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0')
user='/home/ly/beamng-east-coast-user/current'
owned=OwnedProcesses(out,root);errors=[];report=dict(scope=__doc__,passed=False)
bng=None
try:
    subprocess.run([sys.executable,'/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,stdout=(out/'prestart.log').open('w'))
    os.environ['DISPLAY']=':148'
    owned.spawn('display',['Xvfb',':148','-screen','0','1920x1080x24','-nolisten','tcp'])
    limit=time.monotonic()+5
    while not Path('/tmp/.X11-unix/X148').exists() and time.monotonic()<limit:time.sleep(.1)
    assert Path('/tmp/.X11-unix/X148').exists()
    engine=owned.spawn('engine',[str(home/'BinLinux/BeamNG.tech.x64'),'-headless','-nosfx','-userpath',user,'-tcom','-tport','25252','-tcom-listen-ip','127.0.0.1','-nosteam'],cwd=home)
    limit=time.monotonic()+90
    while True:
        assert engine.poll() is None,'engine exit'
        try:
            with socket.create_connection(('127.0.0.1',25252),timeout=.3):break
        except OSError:
            if time.monotonic()>limit:raise TimeoutError('engine ready')
            time.sleep(.2)
    scene=subprocess.run([sys.executable,'/home/ly/planning_phase_physical_20261018/field/scene_left90.py','--home',str(home),'--user',user,'--output',str(out/'scene.json')],stdout=(out/'scene.log').open('w'),stderr=subprocess.STDOUT,timeout=150)
    assert scene.returncode==0,'scene failed'
    sensor=owned.spawn('sensor',[sys.executable,'/home/ly/planning_phase_physical_20261018/candidate/tools/attach_current_vehicle_sensors.py','--rig','/home/ly/planning_phase_physical_20261018/candidate/config/sbr_physical_sensor_rig.json','--output',str(out/'sensor.json'),'--engine-pid',str(engine.pid),'--resume'])
    limit=time.monotonic()+45
    while True:
        owned.healthy()
        if (out/'sensor.json').exists() and json.loads((out/'sensor.json').read_text()).get('ready'):break
        if time.monotonic()>limit:raise TimeoutError('sensor ready')
        time.sleep(.2)
    port=json.loads((out/'sensor.json').read_text())['vehicle_port']
    owned.spawn('traffic',[sys.executable,str(root/'traffic_reader.py'),str(out/'traffic.json')])
    if '--stack' in sys.argv:
        env=dict(os.environ)
        env['PYTHONPATH']=str(root/'instrumentation')+os.pathsep+env.get('PYTHONPATH','')
        env['WILDOS_NATIVE_TIMING_FILE']=str(out/'native_timing.jsonl')
        stack=owned.spawn('stack',['ros2','launch','wildos_nav2_adapter','physical_vehicle_control.launch.py','vehicle_port:='+str(port)],env=env)
        if '--vision' in sys.argv:
            import importlib.util
            from ament_index_python.packages import get_package_share_directory
            vision_env=dict(os.environ,CUDA_VISIBLE_DEVICES='0')
            vision='/home/ly/vision-cu128/bin/python'
            spec=importlib.util.find_spec('visual_navigation')
            tri=Path(next(iter(spec.submodule_search_locations)))/'explorfm_triangulation/obj_mask_triangulation.py'
            share=Path(get_package_share_directory('beamng_wildos_adapter'))/'config'
            owned.spawn('triangulation',[vision,str(tri),'--config',str(share/'beamng_triangulation.yaml')],env=vision_env)
            owned.spawn('vision',[vision,'-m','visual_navigation.wildos.nav','--config','/home/ly/planning_phase_physical_20261018/field/baseline_no_memory.yaml','--do_object_search','true','--memory_in_process_only'],env=vision_env)
            query=owned.spawn('query_load',[sys.executable,str(root/'query_braked.py'),str(out/'query_load.json')])
            deadline=time.monotonic()+205
            while query.poll() is None:
                owned.healthy()
                if time.monotonic()>deadline:raise TimeoutError('braked vision workload')
                time.sleep(.2)
            assert query.returncode==0,'braked query readiness failed'
            owned.children=[c for c in owned.children if c[0]!='query_load']
            report['vision_workload']=json.loads((out/'query_load.json').read_text())
        limit=time.monotonic()+100
        while time.monotonic()<limit:
            owned.healthy();time.sleep(.2)
        report['stack_brake_only']=True
    else:
        probe=owned.spawn('native_probe',[sys.executable,str(root/'probe_native_executor.py'),'--real','--external','--seconds','90','--vehicle-port',str(port),'--output',str(out/'native.json')])
        limit=time.monotonic()+120
        while probe.poll() is None:
            owned.healthy()
            if time.monotonic()>limit:raise TimeoutError('Native diagnostic')
            time.sleep(.2)
        assert probe.returncode==0,'Native diagnostic failed'
        owned.children=[c for c in owned.children if c[0]!='native_probe']
        report['metrics']=json.loads((out/'native.json').read_text())['metrics']
    report['passed']=True
except BaseException as exc:errors.append(repr(exc))
finally:
    try:
        from beamngpy import BeamNGpy
        from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
        bng=BeamNGpy('127.0.0.1',25252,quit_on_close=False,socket_timeout=3).open(launch=False)
        state,samples=settle(lambda:read_physics_snapshot(bng),time.sleep,timeout=12)
        report['stop']=dict(physics=state,samples=samples)
        errors.extend(physical_snapshot_errors(state))
        bng.control.pause()
    except Exception as exc:errors.append('stop: '+repr(exc))
    finally:
        if bng:bng.disconnect()
    errors.extend(owned.close())
    report['errors']=errors;report['passed']=report['passed'] and not errors
    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(out),**report),indent=2))
sys.exit(0 if report['passed'] else 1)
