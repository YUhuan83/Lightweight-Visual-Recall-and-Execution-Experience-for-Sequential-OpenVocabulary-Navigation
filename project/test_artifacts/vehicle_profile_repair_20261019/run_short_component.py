"""One fresh stock SBR short execution; no model/search/video, always close controls."""
from pathlib import Path
import hashlib
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'candidate/tools'))
from wildos_console import OwnedProcesses, verify_sources
from check_continuous_start_execution import settle

OUT = ROOT / ('short_' + time.strftime('%Y%m%d_%H%M%S'))
OUT.mkdir()
owned = OwnedProcesses(OUT, ROOT)
halt = threading.Event()
for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(sig, lambda *_: halt.set())
report = dict(scope=__doc__, passed=False, errors=[], map_loads=0, video=False,
              complete_navigation_passed=False, motion_goals=0)
bng = None; probe_tuple = None


def check():
    if halt.is_set():
        raise InterruptedError('Component interrupted')
    owned.healthy()


def wait(predicate, seconds):
    until = time.monotonic() + seconds
    while not predicate():
        check()
        if time.monotonic() > until:
            raise TimeoutError('Readiness deadline')
        time.sleep(.1)


try:
    pre = subprocess.run(['python3', '/home/ly/planning_phase_physical_20261018/prestart.py'],
                         capture_output=True, text=True, check=True, timeout=20)
    (OUT / 'prestart.json').write_text(pre.stdout)
    assert json.loads(pre.stdout)['processes'] == []
    manifest = json.loads((ROOT / 'frozen_source_manifest.json').read_text())
    assert all(hashlib.sha256((ROOT / 'candidate' / n).read_bytes()).hexdigest() == h
               for n, h in manifest['members'].items())
    identity = json.loads((ROOT / 'verified_install_sources.json').read_text())
    report['installed_source_identity'] = verify_sources({'library_sha256': identity['libraries']})
    report['frozen_manifest_sha256'] = hashlib.sha256((ROOT / 'frozen_source_manifest.json').read_bytes()).hexdigest()
    stack_python = '/home/ly/beamng_ros2_0394_ws/.venv/bin/python'
    report['role_interpreters'] = dict(scene_and_probe=sys.executable, ros_launch=stack_python)
    for role, interpreter, modules in (
            ('sdk', sys.executable, ['beamngpy', 'rclpy', 'nav2_msgs.action']),
            ('launch', stack_python, ['launch', 'launch_ros', 'lark'])):
        code = 'import importlib,json; names=' + repr(modules) + '; print(json.dumps({n:str(importlib.import_module(n).__file__) for n in names}))'
        result = subprocess.run([interpreter, '-c', code], capture_output=True, text=True, timeout=20)
        (OUT / (role + '_imports.log')).write_text(result.stdout + result.stderr)
        assert result.returncode == 0, role + ': dependency preflight failed before map loading'
    os.environ.update(DISPLAY=':148', CUDA_VISIBLE_DEVICES='0', ROS_DOMAIN_ID='204', ROS_LOCALHOST_ONLY='1')
    owned.spawn('display', ['Xvfb', ':148', '-screen', '0', '1920x1080x24', '-nolisten', 'tcp'])
    wait(lambda: Path('/tmp/.X11-unix/X148').exists(), 5)
    home = Path('/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0')
    user = '/home/ly/beamng-east-coast-user/current'
    engine = owned.spawn('engine', [str(home / 'BinLinux/BeamNG.tech.x64'), '-headless', '-nosfx',
        '-userpath', user, '-tcom', '-tport', '25252', '-tcom-listen-ip', '127.0.0.1', '-nosteam'], cwd=home)

    def ready():
        try:
            with socket.create_connection(('127.0.0.1', 25252), timeout=.3):
                return True
        except OSError:
            return False

    wait(ready, 90)
    old = Path('/home/ly/contact_frame_audit_20261019')
    subprocess.run([sys.executable, str(old / 'scene_recorded.py'), '--home', str(home),
                    '--user', user, '--output', str(OUT / 'scene.json')],
                   stdout=(OUT / 'scene.log').open('w'), stderr=subprocess.STDOUT,
                   timeout=150, check=True)
    scene = json.loads((OUT / 'scene.json').read_text())
    assert scene['passed'] and scene['map_loads'] == 1
    report['map_loads'] = 1
    owned.spawn('sensor', [sys.executable, str(old / 'attach_current_vehicle_sensors_observed.py'),
        '--rig', str(ROOT / 'candidate/config/sbr_physical_sensor_rig.json'),
        '--output', str(OUT / 'sensor.json'), '--engine-pid', str(engine.pid), '--resume'])
    wait(lambda: (OUT / 'sensor.json').exists() and json.loads((OUT / 'sensor.json').read_text()).get('ready'), 45)
    port = json.loads((OUT / 'sensor.json').read_text())['vehicle_port']
    native = Path('/home/ly/native_delay_20261018')
    owned.spawn('traffic', [sys.executable, str(native / 'traffic_reader.py'), str(OUT / 'traffic.json')])
    env = dict(os.environ)
    env['PYTHONPATH'] = str(native / 'instrumentation') + os.pathsep + env.get('PYTHONPATH', '')
    env['WILDOS_NATIVE_TIMING_FILE'] = str(OUT / 'native_timing.jsonl')
    env.pop('WILDOS_NATIVE_DIAGNOSTIC_BEST_EFFORT', None)
    stack = owned.spawn('stack', [stack_python, str(ROOT / 'execution_stack_only.py'),
        '--vehicle-port', str(port), '--output', str(OUT / 'stack_variant.json')], env=env)
    libraries = {}

    def runtime_ready():
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():
                continue
            try:
                if os.getpgid(int(proc.name)) != stack.pid:
                    continue
                for line in (proc / 'maps').read_text().splitlines():
                    name = line.split()[-1]
                    path = Path(name)
                    if name.startswith('/') and path.name in identity['libraries']:
                        digest = hashlib.sha256(path.read_bytes()).hexdigest()
                        assert digest == identity['libraries'][path.name], 'Wrong runtime library: ' + name
                        libraries[path.name] = dict(path=name, sha256=digest, pid=int(proc.name))
            except (FileNotFoundError, ProcessLookupError):
                continue
        return set(libraries) == set(identity['libraries'])

    wait(runtime_ready, 40.)
    (OUT / 'runtime_libraries.json').write_text(json.dumps(libraries, indent=2) + '\n')
    report['runtime_libraries'] = libraries
    topics = ['/spot1/odom', '/spot1/nav_graph', '/spot1/local_terrain', '/spot1/local_map',
              '/global_costmap/costmap', '/local_costmap/costmap', '/wildos/points',
              '/wildos/clearing_points', '/vehicles/ego/sensors/front_lidar',
              '/vehicles/ego/sensors/state', '/tf', '/wildos/cmd_vel_stamped',
              '/wildos/control_status', '/wildos/native_control_status', '/wildos/native_control_command',
              '/received_global_plan', '/lookahead_point', '/spot1/measurement_timing']
    recorder = owned.spawn('recorder', ['ros2', 'bag', 'record', '-o', str(OUT / 'interfaces'), *topics])
    probe = owned.spawn('probe', [sys.executable, str(ROOT / 'direct_execution_probe.py'),
                                  '--output', str(OUT / 'execution')])
    probe_tuple = owned.children.pop()
    deadline = time.monotonic() + 130.
    while probe.poll() is None:
        check()
        if time.monotonic() > deadline:
            raise TimeoutError('Bounded execution probe deadline')
        time.sleep(.1)
    owned.children.append(probe_tuple); probe_tuple = None
    report['execution'] = json.loads((OUT / 'execution/result.json').read_text())
    report['motion_goals'] = report['execution']['motion_goals']
    recorder.send_signal(signal.SIGINT); recorder.wait(timeout=15)
    assert (OUT / 'interfaces/metadata.yaml').is_file()
    report['passed'] = report['execution']['passed'] and probe.returncode == 0
except BaseException as exc:
    report['errors'].append(repr(exc))
finally:
    if probe_tuple is not None:
        owned.children.append(probe_tuple)
    gate_ok = False
    try:
        gate = subprocess.run(['ros2', 'param', 'set', '/wildos_cmd_vel_to_control', 'enable', 'false'],
            stdout=(OUT / 'stop_gate.log').open('w'), stderr=subprocess.STDOUT, timeout=6)
        gate_ok = gate.returncode == 0
    except Exception as exc:
        report['errors'].append('Stop gate: ' + repr(exc))
    try:
        from beamngpy import BeamNGpy
        from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors
        bng = BeamNGpy('127.0.0.1', 25252, quit_on_close=False, socket_timeout=3).open(launch=False)
        if not gate_ok:
            bng.control.pause()
            raise RuntimeError('No confirmed stop gate; paused without stop acceptance')
        after, samples = settle(lambda: read_physics_snapshot(bng), time.sleep, timeout=12.)
        assert not physical_snapshot_errors(after)
        report['stop'] = dict(physics=after, samples=samples)
        bng.control.pause()
    except Exception as exc:
        report['errors'].append('Stop/physics: ' + repr(exc))
    finally:
        if bng:
            bng.disconnect()
    report['errors'].extend(owned.close())
    report['passed'] = report['passed'] and not report['errors']
    (OUT / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in dict(output=str(OUT), **report).items()
                      if k not in ('installed_source_identity', 'stop', 'execution')}, indent=2))
raise SystemExit(0 if report['passed'] else 1)
