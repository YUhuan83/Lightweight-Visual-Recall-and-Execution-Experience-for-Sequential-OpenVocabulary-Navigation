"""Deploy the inflation fix and prepare an empty in-process Town memory."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import zipfile

sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env


def restart(name, pid, log_name=None):
    proc = Path('/proc', str(pid))
    args = proc.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
    environment = dict(e.split('=', 1) for e in proc.joinpath('environ').read_bytes().decode().split('\0') if '=' in e)
    cwd = str(proc.joinpath('cwd').resolve())
    if name == 'goal_standoff':
        environment.update(runtime_env())
    os.kill(pid, signal.SIGINT)
    end = time.monotonic() + 20
    while proc.exists() and time.monotonic() < end:
        if proc.joinpath('stat').read_text().split()[2] == 'Z':
            break
        time.sleep(.1)
    if proc.exists() and proc.joinpath('stat').read_text().split()[2] != 'Z':
        raise RuntimeError(name + ' did not stop')
    with ROOT.joinpath(log_name or (name + '.log')).open('ab') as log:
        p = subprocess.Popen(args, cwd=cwd, env=environment, stdout=log,
                             stderr=subprocess.STDOUT, start_new_session=True)
    ROOT.joinpath(name + '.pid').write_text(str(p.pid))
    time.sleep(.5)
    if p.poll() is not None:
        raise RuntimeError(name + ' exited at startup; inspect its log')
    print(name, p.pid, flush=True)


def recover_standoff():
    args = [str(WS/'.venv/bin/python'), str(WS/'install/beamng_wildos_adapter/lib/beamng_wildos_adapter/goal_standoff'),
            '--ros-args', '-p', 'standoff:=5.0', '-p', 'min_update_distance:=1.0',
            '-p', 'confirmation_frames:=5', '-p', 'confirmation_radius:=1.5', '-p', 'obstacle_margin:=2.0']
    with ROOT.joinpath('goal_standoff.log').open('ab') as log:
        p = subprocess.Popen(args, cwd=WS, env=runtime_env(), stdout=log,
                             stderr=subprocess.STDOUT, start_new_session=True)
    ROOT.joinpath('goal_standoff.pid').write_text(str(p.pid))
    time.sleep(.5)
    if p.poll() is not None: raise RuntimeError('Goal standoff failed to start')
    print('goal_standoff',p.pid,flush=True)


if sys.argv[1] == 'tracking_guard':
    source=WS/'src/wildos_nav2_adapter'
    shutil.copy2(ROOT/'graph_path_bridge_stage.py',source/'wildos_nav2_adapter/graph_path_bridge.py')
    shutil.copy2(ROOT/'test_graph_path_bridge_stage.py',source/'test/test_graph_path_bridge_routing.py')
    test=subprocess.run([str(WS/'.venv/bin/python'),'-m','unittest','discover','-s',str(source/'test'),
                         '-p','test_graph_path_bridge_routing.py','-v'],cwd=WS,
                        env=dict(runtime_env(),ROS_DOMAIN_ID='49'),capture_output=True,text=True,timeout=60)
    ROOT.joinpath('ros_tracking_regression.log').write_text(test.stdout+test.stderr)
    print((test.stdout+test.stderr)[-180:],flush=True)
    if test.returncode: raise RuntimeError('Tracking regression failed')
    restart('graph_bridge',int(ROOT.joinpath('graph_bridge.pid').read_text()))
elif sys.argv[1] == 'recover_standoff':
    recover_standoff()
elif sys.argv[1] == 'inflation_fix':
    source = WS / 'src/wildos_nav2_adapter'
    target = source / 'wildos_nav2_adapter/graph_route.py'
    backup = ROOT / 'graph_route_before_inflation_fix.py'
    if not backup.exists():
        shutil.copy2(target, backup)
    shutil.copy2(ROOT / 'graph_route_stage.py', target)
    shutil.copy2(ROOT / 'test_graph_route_stage.py', source / 'test/test_graph_route.py')
    testenv = dict(runtime_env(), ROS_DOMAIN_ID='49')
    for pattern in ('test_graph_route.py', 'test_graph_path_bridge_routing.py'):
        result = subprocess.run([str(WS/'.venv/bin/python'), '-m', 'unittest', 'discover',
                                 '-s', str(source/'test'), '-p', pattern, '-v'],
                                cwd=WS, env=testenv, capture_output=True, text=True, timeout=60)
        ROOT.joinpath('inflation_' + pattern + '.log').write_text(result.stdout + result.stderr)
        print(pattern, result.returncode, (result.stdout+result.stderr)[-220:], flush=True)
        if result.returncode:
            raise RuntimeError('Regression failed')
    restart('graph_bridge', int(ROOT.joinpath('graph_bridge.pid').read_text()))
    manifest = json.loads(ROOT.joinpath('deployment_manifest.json').read_text())
    manifest['wildos_nav2_adapter/graph_route.py'] = hashlib.sha256(target.read_bytes()).hexdigest()
    ROOT.joinpath('deployment_manifest.json').write_text(json.dumps(manifest, indent=2))
elif sys.argv[1] == 'clock_fix':
    source = WS/'src/beamng_wildos_adapter'
    if not source.exists():
        raise RuntimeError('Adapter source directory missing')
    backup = ROOT/'official_adapter_before_clock_fix'
    if not backup.exists():
        shutil.copytree(source, backup, symlinks=True)
    with zipfile.ZipFile(ROOT/'clock_fix_stage.zip') as archive:
        for name in archive.namelist():
            if not (source/name).resolve().is_relative_to(source.resolve()):
                raise RuntimeError('Unexpected archive path')
        archive.extractall(source)
    env = runtime_env()
    build = subprocess.run(['colcon', 'build', '--packages-select', 'beamng_wildos_adapter',
                            '--symlink-install', '--event-handlers', 'console_direct+'],
                           cwd=WS, env=env, capture_output=True, text=True, timeout=120)
    ROOT.joinpath('clock_build.log').write_text(build.stdout+build.stderr)
    if build.returncode:
        raise RuntimeError('Adapter build failed: '+(build.stdout+build.stderr)[-500:])
    pidfile = ROOT/'official_adapter.pid'
    restart('official_adapter', int(pidfile.read_text()) if pidfile.exists() else 2197082)
    # Remove obstacle grids and TF caches from the previous failed round before
    # the next sequence, retaining the loaded BeamNG map and current vehicle.
    for name in ('lifecycle', 'controller', 'planner'):
        pid = int(ROOT.joinpath(name+'.pid').read_text())
        proc = Path('/proc', str(pid))
        args = proc.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
        ROOT.joinpath(name+'_clock_args.json').write_text(json.dumps(args))
        os.kill(pid, signal.SIGINT)
        end = time.monotonic()+10
        while proc.exists() and time.monotonic()<end:
            if proc.joinpath('stat').read_text().split()[2]=='Z': break
            time.sleep(.1)
    for name in ('planner', 'controller', 'lifecycle'):
        args = json.loads(ROOT.joinpath(name+'_clock_args.json').read_text())
        with ROOT.joinpath(name+'.log').open('ab') as log:
            p = subprocess.Popen(args, cwd=WS, env=env, stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        ROOT.joinpath(name+'.pid').write_text(str(p.pid))
        print(name, p.pid, flush=True)
        time.sleep(.5)
elif sys.argv[1] == 'recover_creep':
    pid = int(ROOT.joinpath('speed_control.pid').read_text())
    proc = Path('/proc',str(pid))
    args = proc.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
    if 'min_throttle:=0.03' not in args: raise RuntimeError('Unexpected creep limit')
    args[args.index('min_throttle:=0.03')] = 'min_throttle:=0.06'
    env = runtime_env()
    os.kill(pid,signal.SIGINT)
    end=time.monotonic()+10
    while proc.exists() and time.monotonic()<end:
        if proc.joinpath('stat').read_text().split()[2]=='Z': break
        time.sleep(.1)
    with ROOT.joinpath('speed_control.log').open('ab') as log:
        p=subprocess.Popen(args,cwd=WS,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    ROOT.joinpath('speed_control.pid').write_text(str(p.pid))
    source=WS/'src/wildos_nav2_adapter'
    shutil.copy2(ROOT/'graph_path_bridge_stage.py',source/'wildos_nav2_adapter/graph_path_bridge.py')
    shutil.copy2(ROOT/'test_graph_path_bridge_stage.py',source/'test/test_graph_path_bridge_routing.py')
    test=subprocess.run([str(WS/'.venv/bin/python'),'-m','unittest','discover','-s',str(source/'test'),
                         '-p','test_graph_path_bridge_routing.py','-v'],cwd=WS,
                        env=dict(env,ROS_DOMAIN_ID='49'),capture_output=True,text=True,timeout=60)
    ROOT.joinpath('ros_memory_execution_regression.log').write_text(test.stdout+test.stderr)
    print((test.stdout+test.stderr)[-180:],flush=True)
    if test.returncode: raise RuntimeError('Memory execution regression failed')
    restart('graph_bridge',int(ROOT.joinpath('graph_bridge.pid').read_text()))
    print('Creep input restored to adapter default 6%; remembered view retained after execution failure.',flush=True)
elif sys.argv[1] == 'restore_drive_limit':
    source = WS/'src/wildos_nav2_adapter/wildos_nav2_adapter/native_vehicle_control.py'
    shutil.copy2(ROOT/'native_vehicle_control_stage.py', source)
    restart('native_control', int(ROOT.joinpath('native_control.pid').read_text()))
    pid = int(ROOT.joinpath('speed_control.pid').read_text())
    proc = Path('/proc',str(pid))
    args = proc.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
    env = runtime_env()
    ROOT.joinpath('speed_control_before_drive_limit.json').write_text(json.dumps(args,indent=2))
    if 'max_throttle:=0.08' not in args: raise RuntimeError('Unexpected existing drive limit')
    args[args.index('max_throttle:=0.08')] = 'max_throttle:=0.12'
    os.kill(pid,signal.SIGINT)
    end=time.monotonic()+10
    while proc.exists() and time.monotonic()<end:
        if proc.joinpath('stat').read_text().split()[2]=='Z': break
        time.sleep(.1)
    with ROOT.joinpath('speed_control.log').open('ab') as log:
        p=subprocess.Popen(args,cwd=WS,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    ROOT.joinpath('speed_control.pid').write_text(str(p.pid))
    time.sleep(.5)
    if p.poll() is not None: raise RuntimeError('Drive control failed at startup')
    print('Drive input ceiling restored from 8% to existing actuator 12%; speed target unchanged.',flush=True)
elif sys.argv[1] == 'fresh_memory':
    from beamngpy import BeamNGpy
    b = BeamNGpy('127.0.0.1', 25252, socket_timeout=5)
    b.open(launch=False)
    b.control.pause()
    level = str(b.scenario.get_current().level)
    if level != 'east_coast_usa':
        raise RuntimeError('Unexpected map: ' + level)
    b.vehicles.teleport('ego', (734.294, -11.946, 52.154),
                        (0., 0., .7658087977066911, .6430683364581344), reset=True)
    b.disconnect()
    oldroot = Path('/home/ly/full_town_memory_20260930')
    pidfile = ROOT/'goal_standoff.pid'
    pid = int(pidfile.read_text()) if pidfile.exists() else 19167
    if Path('/proc',str(pid)).exists(): restart('goal_standoff',pid)
    else: recover_standoff()
    for name in ('triangulation', 'vision'):
        pidfile = ROOT/(name+'.pid')
        if not pidfile.exists():
            pidfile = oldroot/(name+'.pid')
        restart(name, int(pidfile.read_text()))
    print('Map retained; memory starts empty before the goal sequence.', flush=True)
elif sys.argv[1] == 'full_test':
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'), str(ROOT/'full_town_memory_test.py'),
                                    sys.argv[2]], cwd=WS, env=runtime_env()).returncode)
