"""Explicit localhost-only scenario runner for the standalone map; statics live in the level."""
import argparse
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import socket
import subprocess
import time
import uuid
from install import ROOT, LEVEL, effective_user, installed, validate


def build_scenario():
    from beamngpy import Scenario, Vehicle
    manifest=validate()
    scene=Scenario(LEVEL,'tc_drive_'+uuid.uuid4().hex,
                   description='Standalone countryside: native D-Series, no overlay objects')
    car=Vehicle('nc_ego',model='pickup',part_config='vehicles/pickup/d15_4wd_A.pc')
    scene.add_vehicle(car,pos=tuple(manifest['spawn']['pos']),rot_quat=tuple(manifest['spawn']['rot_quat']))
    return scene,car


def parse_args(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--home',type=Path,default=os.environ.get('BEAMNG_HOME'))
    p.add_argument('--user',type=Path,default=os.environ.get('BEAMNG_USER'))
    modes=p.add_mutually_exclusive_group(required=True)
    modes.add_argument('--launch',action='store_true')
    modes.add_argument('--connect',action='store_true')
    p.add_argument('--replace-current',action='store_true',help='Explicit consent to discard current runtime world')
    p.add_argument('--port',type=int,default=64256)
    p.add_argument('--timeout',type=float,default=180)
    p.add_argument('--duration',type=float,default=10)
    p.add_argument('--cadence',type=float,default=.2)
    p.add_argument('--output',type=Path)
    sensors=p.add_mutually_exclusive_group()
    sensors.add_argument('--no-sensors',action='store_true')
    sensors.add_argument('--sensors',action='store_true',help='Front RGB + IMU; default is no sensors')
    p.add_argument('--side-cameras',action='store_true')
    p.add_argument('--lidar',action='store_true')
    p.add_argument('--headless',action='store_true',help='Linux rendering still requires configured GPU/display')
    a=p.parse_args(argv)
    if not a.replace_current:
        p.error('--replace-current required; nothing connected or changed')
    if not a.home or not a.home.is_dir() or not a.user:
        p.error('Existing --home and --user required')
    a.home=a.home.resolve()
    a.user=effective_user(a.user)
    if not 1<=a.port<=65535 or any(not math.isfinite(x) or x<=0 for x in (a.timeout,a.duration,a.cadence)):
        p.error('Port/duration/cadence/timeout invalid')
    if (a.side_cameras or a.lidar) and not a.sensors:
        p.error('--side-cameras and --lidar require --sensors')
    return a


def guard(a):
    if a.launch:
        cmd=['tasklist','/FO','CSV','/NH'] if platform.system()=='Windows' else ['ps','-A','-o','comm=']
        result=subprocess.run(cmd,capture_output=True,text=True,check=True,timeout=10)
        if 'beamng' in result.stdout.lower():
            raise ValueError('A BeamNG process exists. Close it yourself before --launch; no second device/session started.')
        with socket.socket() as s:
            if platform.system()=='Windows':
                s.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
            s.bind(('127.0.0.1',a.port))
    else:
        with socket.create_connection(('127.0.0.1',a.port),timeout=2):
            pass


def execute(a):
    if importlib.metadata.version('beamngpy')!='1.36':
        raise ValueError('Requires beamngpy==1.36')
    print('Verified map: '+str(installed(a.user)),flush=True)
    guard(a)
    from beamngpy import BeamNGpy
    from beamngpy.sensors import Camera, AdvancedIMU, Lidar
    binary='Bin64/BeamNG.tech.x64.exe' if platform.system()=='Windows' else 'BinLinux/BeamNG.tech.x64'
    # 0.39 appends current/ to -userpath; a.user is the effective mod directory.
    if a.user.name != 'current':
        raise ValueError('Expected effective user directory ending in current')
    bng=BeamNGpy('127.0.0.1',a.port,home=str(a.home),user=str(a.user.parent),binary=binary,
                 quit_on_close=False,socket_timeout=a.timeout,headless=a.headless)
    scene,car=build_scenario()
    sensors=[]
    output=(a.output or ROOT/'captures'/time.strftime('%Y%m%d_%H%M%S')).resolve()
    output.mkdir(parents=True,exist_ok=False)
    try:
        previous=Path.cwd()
        try:
            if a.launch: os.chdir(a.home)
            bng.open(launch=a.launch,listen_ip='127.0.0.1')
        finally:
            os.chdir(previous)
        if Path(bng.user_with_version).resolve() != a.user.resolve():
            raise ValueError(f'Wrong running user directory: {bng.user_with_version}; expected {a.user}. Restart with -userpath {a.user.parent}')
        levels=bng.scenario.get_levels()
        if LEVEL not in levels:
            raise ValueError('New map not in mounted level list. Save editor work, restart the intended user profile, then retry.')
        scene.make(bng)
        bng.scenario.load(scene)
        bng.scenario.start()
        car.control(parkingbrake=1)
        if a.sensors:
            cameras=[('front',(0,-1.5,2),(0,-1,0))]
            if a.side_cameras:
                cameras += [('left',(-1.2,0,2),(-1,0,0)),('right',(1.2,0,2),(1,0,0))]
            for name,pos,direction in cameras:
                sensor=Camera('nc_'+name,bng,car,pos=pos,dir=direction,resolution=(640,360),
                              requested_update_time=a.cadence,is_render_colours=True,is_render_depth=False,
                              is_render_annotations=False,is_render_instance=False,is_using_shared_memory=False,
                              is_visualised=False)
                sensors.append((name,sensor))
            sensors.append(('imu',AdvancedIMU('nc_imu',bng,car,pos=(0,0,1),gfx_update_time=a.cadence,
                            physics_update_time=.01,is_allow_wheel_nodes=False,is_visualised=False)))
            if a.lidar:
                sensors.append(('lidar',Lidar('nc_lidar',bng,car,pos=(0,0,2.2),requested_update_time=a.cadence,
                                vertical_resolution=16,frequency=10,max_distance=60,is_annotated=False,
                                is_using_shared_memory=False,is_visualised=False)))
        begin=time.monotonic()
        index=0
        while time.monotonic()-begin<a.duration:
            for name,sensor in sensors:
                reading=sensor.poll()
                stem=output/f'{index:06d}_{name}'
                if name=='imu':
                    def clean(data):
                        if isinstance(data,(list,tuple)): return [clean(v) for v in data]
                        if isinstance(data,dict):
                            keys={'time','accRaw','accSmooth','angVel','angVelSmooth'}
                            return {k:clean(v) for k,v in data.items() if k in keys} if keys.intersection(data) else {k:clean(v) for k,v in data.items() if isinstance(v,(dict,list,tuple))}
                        if hasattr(data,'tolist'): return clean(data.tolist())
                        return data if isinstance(data,(int,float,str,bool)) or data is None else None
                    stem.with_suffix('.json').write_text(json.dumps(clean(reading)),encoding='utf-8')
                elif name=='lidar':
                    import numpy as np
                    np.save(stem.with_suffix('.npy'),reading['pointCloud'],allow_pickle=False)
                else:
                    reading['colour'].save(stem.with_suffix('.png'))
            index+=1
            time.sleep(a.cadence)
        car.sensors.poll()
        report=dict(level=LEVEL,vehicle=car.vid,parked=True,frames=index,sensors=[n for n,_ in sensors],
                    state=dict(car.state),driving_test=False)
        (output/'runtime_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print('Map left open with one parked D-Series. Release parking brake to drive. Report: '+str(output))
    finally:
        for _,sensor in reversed(sensors):
            try: sensor.remove()
            except Exception as exc: print('Sensor cleanup: '+str(exc))
        bng.disconnect()


if __name__=='__main__':
    execute(parse_args())
