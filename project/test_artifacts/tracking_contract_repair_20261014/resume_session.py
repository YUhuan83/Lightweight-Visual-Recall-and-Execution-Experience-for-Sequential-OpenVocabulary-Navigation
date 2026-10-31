"""New owned physical session after the user explicitly resumed testing."""
import argparse,json,os,signal,socket,subprocess,time,sys
from pathlib import Path

BASE=Path('/home/ly/town_memory_validation_20261014/tracking_contract_candidate')
OUT=BASE/'resume_session1'
PY='/home/ly/beamng_ros2_0394_ws/.venv/bin/python'
HOME=Path('/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0')
sys.path.insert(0,str(BASE/'bundle/tools'))

def spawn(role,args,cwd):
    with (OUT/(role+'.log')).open('xb') as log:
        p=subprocess.Popen(args,cwd=cwd,stdin=subprocess.DEVNULL,stdout=log,
            stderr=subprocess.STDOUT,start_new_session=True)
    (OUT/(role+'.pid')).write_text(str(p.pid)+'\n')
    return p

def main():
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['engine','prepare','stack'])
    stage=parser.parse_args().stage
    from server_test_preflight import process_rows
    if stage=='engine':
        assert not any(any(Path(t).name=='BeamNG.tech.x64' for t in r['tokens']) for r in process_rows())
        OUT.mkdir(exist_ok=False)
        env=os.environ;env['CUDA_VISIBLE_DEVICES']='0'
        p=spawn('engine',[str(HOME/'BinLinux/BeamNG.tech.x64'),'-headless','-nosfx',
            '-userpath','/home/ly/beamng-east-coast-user/current','-tcom','-tport','25252',
            '-tcom-listen-ip','127.0.0.1','-nosteam'],HOME)
        print(json.dumps({'engine_pid':p.pid}),flush=True);return
    engine=int((OUT/'engine.pid').read_text())
    assert b'BeamNG.tech.x64' in Path(f'/proc/{engine}/cmdline').read_bytes()
    if stage=='prepare':
        p=subprocess.run([PY,'/home/ly/town_memory_validation_20261013/prepare_town_scene.py',
            '--output',str(OUT/'scene_preflight.json')],text=True,capture_output=True,timeout=90)
        (OUT/'scene_prepare.log').write_text(p.stdout+p.stderr)
        scene=json.loads((OUT/'scene_preflight.json').read_text())
        # Preserve the original instantaneous-spawn result, then independently
        # require the original physics and sustained fresh stop after landing.
        from wildos_nav2_adapter.physical_vehicle import physical_snapshot_errors
        assert not physical_snapshot_errors(scene['physics_snapshot'],require_sensors=False)
        assert scene['map_reload_count']==1 and not scene['collision_override_requested']
        p=subprocess.run([PY,str(BASE/'component_tools/brake_and_hold_spawn.py'),
            '--output',str(OUT/'brake_preparation.json')],text=True,capture_output=True,timeout=30)
        (OUT/'brake_preparation.log').write_text(p.stdout+p.stderr)
        assert p.returncode==0,p.stdout+p.stderr
        print(json.dumps({'prepared':True,'map_loads':1,'teleports':0}));return
    assert json.loads((OUT/'brake_preparation.json').read_text())['passed']
    assert json.loads((BASE/'installation_identity.json').read_text())['passed']
    for r in process_rows():
        assert not any(Path(t).name in ('graph_path_bridge','native_vehicle_control',
            'cmd_vel_to_control','attach_current_vehicle_sensors.py') for t in r['tokens']),r['command']
    p=spawn('sensor',[PY,'tools/attach_current_vehicle_sensors.py','--rig',
        'config/sbr_physical_sensor_rig.json','--output',str(OUT/'sensor_attachment.json'),
        '--engine-pid',str(engine),'--resume'],BASE/'bundle')
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        assert p.poll() is None,'sensor exited'
        if (OUT/'sensor_attachment.json').exists():
            sensor=json.loads((OUT/'sensor_attachment.json').read_text())
            if sensor.get('ready'):break
        time.sleep(.2)
    else:raise RuntimeError('Sensor preparation timeout')
    p=spawn('stack',['/opt/ros/humble/bin/ros2','launch','wildos_nav2_adapter',
        'physical_vehicle_control.launch.py','vehicle_port:='+str(sensor['vehicle_port'])],BASE/'bundle')
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        assert p.poll() is None,'stack exited'
        matches=[r for r in process_rows() if r['domain']=='88' and
            any(Path(t).name=='planner_node' and t.startswith(str(BASE/'install')) for t in r['tokens'])]
        if len(matches)==1:break
        time.sleep(.2)
    else:raise RuntimeError('Owned GraphNav planner not found')
    r=matches[0];assert os.getpgid(r['pid'])==p.pid;os.kill(r['pid'],signal.SIGINT)
    (OUT/'stack_preparation.json').write_text(json.dumps({'passed':True,'engine_pid':engine,
        'stack_pid':p.pid,'sensor_pid':int((OUT/'sensor.pid').read_text()),
        'vehicle_port':sensor['vehicle_port'],'retired_graph_planner':r['pid'],
        'map_loads':1,'teleports':0},indent=2))
    print((OUT/'stack_preparation.json').read_text())

if __name__=='__main__':main()
