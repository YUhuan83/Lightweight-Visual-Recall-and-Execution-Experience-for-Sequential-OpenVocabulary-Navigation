"""Recorded short-section execution/contact diagnostic; no object search or video."""
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

sys.path.insert(0, '/home/ly/planning_phase_physical_20261018/candidate/tools')
from wildos_console import OwnedProcesses, proc_identity
from check_continuous_start_execution import settle
ROOT = Path('/home/ly/recorded_section_replay_20261018')
NATIVE = Path('/home/ly/native_delay_20261018')
OUT = ROOT/('recorded_'+time.strftime('%Y%m%d_%H%M%S'))
OUT.mkdir()
owned = OwnedProcesses(OUT, ROOT)
halt = threading.Event()
for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP): signal.signal(sig, lambda *_: halt.set())
report = dict(scope=__doc__, passed=False, errors=[], map_loads=0, motion_goals=0, video=False)
bng = None; contact_tuple = None


def check():
    if halt.is_set(): raise InterruptedError('Diagnostic interrupted')
    owned.healthy()


def wait(predicate, seconds):
    deadline = time.monotonic()+seconds
    while not predicate():
        check()
        if time.monotonic() >= deadline: raise TimeoutError('Component readiness')
        time.sleep(.1)


try:
    manifest_root = Path('/home/ly/smoothing_boundary_repair_20261018')
    manifest = json.loads((manifest_root/'official_wildos_beamng_bundle.manifest.json').read_text())
    assert manifest['sha256'] == '6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b'
    for name, want in manifest['members'].items():
        assert hashlib.sha256((manifest_root/'candidate'/name).read_bytes()).hexdigest() == want, name
    report['frozen_candidate'] = manifest['sha256']
    report['diagnostic_files'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in ROOT.iterdir() if p.suffix in ('.py', '.lua', '.sh') and p.is_file()}
    subprocess.run([sys.executable, '/home/ly/planning_phase_physical_20261018/prestart.py'],
                   check=True, stdout=(OUT/'prestart.log').open('w'))
    os.environ['DISPLAY'] = ':148'
    os.environ['CUDA_VISIBLE_DEVICES'] = '0'
    owned.spawn('display', ['Xvfb', ':148', '-screen', '0', '1920x1080x24', '-nolisten', 'tcp'])
    wait(lambda: Path('/tmp/.X11-unix/X148').exists(), 5)
    home = Path('/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0')
    user = '/home/ly/beamng-east-coast-user/current'
    engine = owned.spawn('engine', [str(home/'BinLinux/BeamNG.tech.x64'), '-headless', '-nosfx',
        '-userpath', user, '-tcom', '-tport', '25252', '-tcom-listen-ip', '127.0.0.1', '-nosteam'], cwd=home)
    def engine_ready():
        try:
            with socket.create_connection(('127.0.0.1', 25252), timeout=.3): return True
        except OSError: return False
    wait(engine_ready, 90)
    scene = subprocess.run([sys.executable, str(ROOT/'scene_recorded.py'),
        '--home', str(home), '--user', user, '--output', str(OUT/'scene.json')],
        stdout=(OUT/'scene.log').open('w'), stderr=subprocess.STDOUT, timeout=150)
    assert scene.returncode == 0
    report['map_loads'] = json.loads((OUT/'scene.json').read_text())['map_loads']
    owned.spawn('sensor', [sys.executable, '/home/ly/planning_phase_physical_20261018/candidate/tools/attach_current_vehicle_sensors.py',
        '--rig', '/home/ly/planning_phase_physical_20261018/candidate/config/sbr_physical_sensor_rig.json',
        '--output', str(OUT/'sensor.json'), '--engine-pid', str(engine.pid), '--resume'])
    wait(lambda: (OUT/'sensor.json').exists() and json.loads((OUT/'sensor.json').read_text()).get('ready'), 45)
    port = json.loads((OUT/'sensor.json').read_text())['vehicle_port']
    owned.spawn('traffic', [sys.executable, str(NATIVE/'traffic_reader.py'), str(OUT/'traffic.json')])
    env = dict(os.environ)
    env['PYTHONPATH'] = str(NATIVE/'instrumentation')+os.pathsep+env.get('PYTHONPATH', '')
    env['WILDOS_NATIVE_TIMING_FILE'] = str(OUT/'native_timing.jsonl')
    env.pop('WILDOS_NATIVE_DIAGNOSTIC_BEST_EFFORT', None)
    owned.spawn('stack', ['ros2', 'launch', 'wildos_nav2_adapter', 'physical_vehicle_control.launch.py', 'vehicle_port:='+str(port)], env=env)
    topics = ['/vehicles/ego/sensors/front_lidar', '/spot1/ouster/front/points_filtered',
              '/wildos/points', '/wildos/clearing_points', '/tf', '/spot1/odom',
              '/global_costmap/costmap', '/local_costmap/costmap', '/spot1/local_terrain',
              '/spot1/local_map', '/wildos/native_control_status', '/wildos/native_control_command',
              '/wildos/control_status', '/wildos/diagnostic/contact_snapshot',
              '/spot1/measurement_timing', '/vehicles/ego/sensors/state',
              '/wildos/cmd_vel_stamped', '/wildos/cmd_vel', '/received_global_plan', '/lookahead_point']
    recorder = owned.spawn('recorder', ['ros2', 'bag', 'record', '-o', str(OUT/'interfaces'), *topics])
    contact = owned.spawn('contact_probe', [sys.executable, str(ROOT/'execute_recorded.py'),
        '--vehicle-port', str(port), '--output', str(OUT/'contact_result.json')])
    # Its normal exit is expected. Keep ownership for finally, exclude it from
    # checks which require long-lived services to remain running.
    contact_tuple = owned.children.pop()
    deadline = time.monotonic()+100
    while contact.poll() is None:
        check()
        assert proc_identity(contact.pid) == contact_tuple[2]
        if time.monotonic() >= deadline: raise TimeoutError('Contact probe')
        time.sleep(.1)
    owned.children.append(contact_tuple); contact_tuple = None
    assert contact.returncode == 0, 'Contact probe failed; inspect contact_probe.log'
    report['contact_result'] = json.loads((OUT/'contact_result.json').read_text())
    identity = subprocess.run([sys.executable, str(NATIVE/'verify_running_smac.py'), str(OUT)],
                              stdout=(OUT/'runtime_identity.log').open('w'), stderr=subprocess.STDOUT, timeout=10)
    assert identity.returncode == 0, 'Runtime identity check'
    recorder.send_signal(signal.SIGINT); recorder.wait(timeout=15)
    assert (OUT/'interfaces/metadata.yaml').is_file(), 'Recorder did not finalize'
    report['component_closed'] = bool(report['contact_result']['protocol_closed'])
    report['passed'] = bool(report['contact_result']['execution_passed'])
    report['motion_goals'] = int(report['contact_result']['motion_goals'])
    report['outcome'] = report['contact_result']['outcome']
except BaseException as exc:
    report['errors'].append(repr(exc))
finally:
    if contact_tuple is not None: owned.children.append(contact_tuple)
    gate_ok=False
    try:
        gate=subprocess.run(['ros2','param','set','/wildos_cmd_vel_to_control','enable','false'],
            stdout=(OUT/'parent_stop_gate.log').open('w'),stderr=subprocess.STDOUT,timeout=6)
        gate_ok=gate.returncode==0
        if not gate_ok:report['errors'].append('Parent speed gate not confirmed')
    except Exception as exc:report['errors'].append('Parent speed gate: '+repr(exc))
    try:
        from beamngpy import BeamNGpy
        from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors
        bng = BeamNGpy('127.0.0.1', 25252, quit_on_close=False, socket_timeout=3).open(launch=False)
        if not gate_ok:
            bng.control.pause()
            raise RuntimeError('Actuation gate not confirmed; paused, no stop acceptance')
        state, samples = settle(lambda: read_physics_snapshot(bng), time.sleep, timeout=12)
        report['stop'] = dict(physics=state, samples=samples)
        report['errors'].extend(physical_snapshot_errors(state))
        bng.control.pause()
    except Exception as exc: report['errors'].append('stop: '+repr(exc))
    finally:
        if bng: bng.disconnect()
    report['errors'].extend(owned.close())
    report['passed'] = report['passed'] and not report['errors']
    (OUT/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in dict(output=str(OUT), **report).items() if k not in ('contact_result', 'stop')}, indent=2))
raise SystemExit(0 if report.get('component_closed') and not report['errors'] else 1)
