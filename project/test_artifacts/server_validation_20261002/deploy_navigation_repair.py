"""Deploy planner repairs while keeping BeamNG and visual memory alive."""
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

out = ROOT/'navigation_repair'
out.mkdir(exist_ok=True)
env = runtime_env()
if sys.argv[1] in ('stage', 'update'):
    stopped = subprocess.run([str(WS/'.venv/bin/python'), '/home/ly/session_tools_20261002.py',
                              'inspect', 'before_navigation_repair'], env=env,
                             capture_output=True, text=True, timeout=30)
    if stopped.returncode:
        raise RuntimeError('Could not pause navigation: '+stopped.stderr[-500:])
    manifest = []
    with zipfile.ZipFile('/home/ly/navigation_repair.zip') as archive:
        for name in archive.namelist():
            relative = Path(name)
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError('Unsafe staging path')
            target = WS/relative
            backup = out/'before'/relative
            backup.parent.mkdir(parents=True, exist_ok=True)
            if target.exists() and not backup.exists():
                shutil.copy2(target, backup)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(archive.read(name))
            manifest.append(dict(file=name, sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2))
    print('Staged', len(manifest), 'files; map and vision preserved', flush=True)
    if sys.argv[1] == 'update':
        raise SystemExit(0)
    command = 'source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select graphnav_planner --event-handlers console_direct+'
    with (out/'build.log').open('w') as log:
        build = subprocess.run(['bash', '-c', command], cwd=WS, env=env,
                               stdout=log, stderr=subprocess.STDOUT, timeout=300)
    print((out/'build.log').read_text()[-1600:], flush=True)
    if build.returncode:
        raise RuntimeError('GraphNav build failed; existing processes still running')
elif sys.argv[1] == 'verify':
    suites = [('wildos_nav2_adapter', 'test_graph_path_bridge_routing.py'),
              ('wildos_nav2_adapter', 'test_graph_route.py'),
              ('wildos_nav2_adapter', 'test_cmd_vel_to_control_ros.py'),
              ('wildos_nav2_adapter', 'test_cmd_vel_to_control.py'),
              ('wildos_nav2_adapter', 'test_test_progress.py')]
    results = []
    for package, pattern in suites:
        test = subprocess.run([str(WS/'.venv/bin/python'), '-m', 'unittest', 'discover',
                               '-s', str(WS/'src'/package/'test'), '-p', pattern, '-v'],
                              env=dict(env, ROS_DOMAIN_ID='53'), cwd=WS,
                              capture_output=True, text=True, timeout=60)
        (out/(pattern+'.log')).write_text(test.stdout+test.stderr)
        print(pattern, test.returncode, (test.stdout+test.stderr)[-180:], flush=True)
        results.append(dict(suite=pattern, passed=test.returncode == 0))
        if test.returncode:
            raise RuntimeError('Server regression failed')
    (out/'tests.json').write_text(json.dumps(results, indent=2))
elif sys.argv[1] == 'restart':
    specs = {}
    for name, pid in [('native_control', int((ROOT/'native_control.pid').read_text())),
                      ('visualization', int((ROOT/'visualization.pid').read_text()))]:
        p = Path('/proc', str(pid))
        specs[name] = dict(pid=pid, args=p.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0'),
                           cwd=str(p.joinpath('cwd').resolve()),
                           environment=dict(v.decode().split('=', 1) for v in p.joinpath('environ').read_bytes().split(b'\0') if b'=' in v))
    vision_pid = int((ROOT/'vision.pid').read_text())
    # Only navigation/control nodes restart, preserving the visual process.
    for name, spec in specs.items():
        os.kill(spec['pid'], signal.SIGINT)
        deadline = time.monotonic()+15
        p = Path('/proc', str(spec['pid']))
        while p.exists() and time.monotonic() < deadline:
            if p.joinpath('stat').read_text().split()[2] == 'Z':
                break
            time.sleep(.1)
        if p.exists() and p.joinpath('stat').read_text().split()[2] != 'Z':
            raise RuntimeError(name+' did not stop')
        overrides = {'graph_bridge': {'follow_path': 'false'},
                     'speed_control': {'enable': 'false', 'max_speed': '0.0', 'overspeed_limit': '0.0'},
                     'native_control': {'overspeed_limit': '0.0', 'command_timeout': '0.8'}}.get(name, {})
        for key, value in overrides.items():
            args = spec['args']
            index = next((i for i, a in enumerate(args) if a.startswith(key+':=')), None)
            if index is None:
                args.extend(['-p', key+':='+value])
            else:
                args[index] = key+':='+value
        with (out/(name+'.log')).open('ab') as log:
            new = subprocess.Popen(spec['args'], cwd=spec['cwd'], env=spec['environment'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        (ROOT/(name+'.pid')).write_text(str(new.pid))
        time.sleep(1)
        if new.poll() is not None:
            raise RuntimeError(name+' failed startup')
        print(name, spec['pid'], '->', new.pid, flush=True)
    assert Path('/proc', str(vision_pid)).exists()
    assert Path('/proc/909616').exists()
    print('BeamNG and in-process visual memory preserved', flush=True)
else:
    raise ValueError('stage, verify or restart required')
