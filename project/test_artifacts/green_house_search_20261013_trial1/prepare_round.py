"""Prepare one frozen candidate search; no vehicle commands during the search."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path('/home/ly/town_memory_validation_20261013')
CANDIDATE = ROOT / 'navigation_transactions_candidate'
BUNDLE = CANDIDATE / 'bundle'
INSTALL = CANDIDATE / 'install'
VENV = '/home/ly/beamng_ros2_0394_ws/.venv/bin/python'
VISION = '/home/ly/vision-cu128/bin/python'


def read_process(pid, marker):
    p = Path('/proc') / str(pid)
    tokens = [x for x in (p/'cmdline').read_bytes().decode().split('\0') if x]
    if not any(marker == x or Path(x).name == marker for x in tokens):
        raise RuntimeError(f'PID {pid} does not own {marker}')
    env = dict(x.split('=', 1) for x in (p/'environ').read_bytes().decode().split('\0') if '=' in x)
    return {'pid': pid, 'tokens': tokens, 'env': env}


def alive(pid):
    try:
        state = (Path('/proc')/str(pid)/'stat').read_text().split(') ', 1)[1].split()[0]
        return state not in ('Z', 'X')
    except FileNotFoundError:
        return False


def stop_owned(row):
    os.kill(row['pid'], signal.SIGINT)
    end = time.monotonic() + 25
    while time.monotonic() < end and alive(row['pid']):
        time.sleep(.1)
    if alive(row['pid']):
        raise RuntimeError(f"Owned PID {row['pid']} did not stop after SIGINT")


def spawn(out, name, command, env):
    with (out/f'{name}.log').open('xb') as log:
        p = subprocess.Popen(command, cwd=BUNDLE, env=env, stdin=subprocess.DEVNULL,
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    (out/f'{name}.pid').write_text(str(p.pid)+'\n')
    return p


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    report = {'passed': False, 'map_reload_count': 0, 'setup_vehicle_teleports': 0,
              'production_changes': False, 'started_pids': {}}
    bng = None
    try:
        stack = read_process(755413, 'physical_vehicle_control.launch.py')
        vision = read_process(812102, 'visual_navigation.wildos.nav')
        sensor = read_process(752397, 'attach_current_vehicle_sensors.py')
        if any(row['env'].get('ROS_DOMAIN_ID') != '87' for row in (stack, vision, sensor)):
            raise RuntimeError('Unexpected running ROS domain')
        archive = CANDIDATE/'candidate.zip'
        report['candidate_sha256'] = hashlib.sha256(archive.read_bytes()).hexdigest()
        if report['candidate_sha256'] != 'f9561be85a67cc1b47044f090e8fbb80cda00e14fbb0b0bf52e7c49d116e526f':
            raise RuntimeError('Frozen archive identity mismatch')
        report['source_checks'] = {}
        for rel in ('graph_path_bridge.py', 'continuous_path.py', 'navigation_transactions.py',
                    'memory_selection.py', 'cmd_vel_to_control.py', 'native_vehicle_control.py',
                    'execution_feedback.py', 'vehicle_control_lease.lua'):
            source = BUNDLE/'nav2_adapter/wildos_nav2_adapter'/rel
            installed = INSTALL/'lib/python3.10/site-packages/wildos_nav2_adapter'/rel
            expected = hashlib.sha256(source.read_bytes()).hexdigest()
            actual = hashlib.sha256(installed.read_bytes()).hexdigest()
            report['source_checks'][rel] = {'expected': expected, 'actual': actual}
            if actual != expected:
                raise RuntimeError('Installed source mismatch: '+rel)
        import rclpy
        from rcl_interfaces.srv import GetParameters
        rclpy.init()
        node = rclpy.create_node('green_house_prepare_gate_inspector')
        try:
            report['gates'] = {}
            for owner, name in (('/wildos_cmd_vel_to_control', 'enable'),
                                ('/wildos_graph_path_bridge', 'follow_path')):
                client = node.create_client(GetParameters, owner+'/get_parameters')
                if not client.wait_for_service(timeout_sec=5):
                    raise RuntimeError('Missing gate service: '+owner)
                future = client.call_async(GetParameters.Request(names=[name]))
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                if not future.done() or future.exception():
                    raise RuntimeError('Cannot inspect gate: '+owner)
                value = future.result().values[0]
                report['gates'][owner+'/'+name] = value.bool_value
                if value.type != 1 or value.bool_value:
                    raise RuntimeError('Gate is open: '+owner)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        from beamngpy import BeamNGpy
        from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
        from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
        from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors
        install_fail_closed_reads(PrefixedLengthSocket)
        bng = BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
        before = read_physics_snapshot(bng)
        report['before'] = before
        if physical_snapshot_errors(before) or before['speed'] > .05:
            raise RuntimeError('Original physical car must be healthy and stationary before reset')
        for row in (stack, vision, sensor):
            stop_owned(row)
        report['old_owners_stopped'] = [stack['pid'],vision['pid'],sensor['pid']]
        stopped = read_physics_snapshot(bng)
        if stopped['speed'] > .05 or stopped.get('cameras') or stopped.get('lidars'):
            raise RuntimeError('Vehicle is moving or stopped sensor owner left sensors')
        bng.control.pause()
        if not bng.vehicles.teleport('ego',(734.294,-11.946,52.154),
                rot_quat=(0.,0.,.7658087977066911,.6430683364581344),
                reset=True,safe_spawn=False):
            raise RuntimeError('Town setup teleport rejected')
        report['setup_vehicle_teleports'] = 1
        after = read_physics_snapshot(bng)
        report['after_reset'] = after
        errors = physical_snapshot_errors(after,require_sensors=False)
        if errors or after['mission'] != before['mission'] or math.dist(after['position'][:2],(734.294,-11.946)) > 1.25:
            raise RuntimeError('Same-map reset physical check: '+str(errors))
        p = spawn(out,'sensor',['/home/ly/beamng_ros2_0394_ws/.venv/bin/python',
            'tools/attach_current_vehicle_sensors.py','--rig','config/sbr_physical_sensor_rig.json',
            '--output',str(out/'sensor_attachment.json'),'--engine-pid','741456','--resume'],sensor['env'])
        report['started_pids']['sensor'] = p.pid
        deadline = time.monotonic()+25
        while time.monotonic()<deadline:
            if p.poll() is not None: raise RuntimeError('Sensor attachment exited')
            if (out/'sensor_attachment.json').exists():
                sensor_report = json.loads((out/'sensor_attachment.json').read_text())
                if sensor_report.get('ready'): break
            time.sleep(.2)
        else: raise RuntimeError('Sensor attachment did not become ready')
        port = sensor_report['vehicle_port']
        report['vehicle_port'] = port
        p = spawn(out,'stack',['/opt/ros/humble/bin/ros2','launch','wildos_nav2_adapter',
            'physical_vehicle_control.launch.py',f'vehicle_port:={port}'],stack['env'])
        report['started_pids']['stack'] = p.pid
        tri = INSTALL/'lib/python3.10/site-packages/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
        p = spawn(out,'triangulation',[VISION,str(tri),'--config',
            str(INSTALL/'share/beamng_wildos_adapter/config/beamng_triangulation.yaml')],vision['env'])
        report['started_pids']['triangulation'] = p.pid
        p = spawn(out,'vision',[VISION,'-m','visual_navigation.wildos.nav','--config',
            'beamng_wildos','--do_object_search','true','--memory_in_process_only'],vision['env'])
        report['started_pids']['vision'] = p.pid
        report['passed'] = True
    except BaseException as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        if bng:
            try: bng.control.pause()
            except Exception as pause: report['pause_error'] = repr(pause)
    finally:
        if bng:
            try: bng.disconnect()
            except Exception as exc: report['disconnect_error'] = repr(exc)
        (out/'round_preparation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('before','after_reset','source_checks')},indent=2),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
