"""Bounded real BeamNG sensor attachment / SIGTERM / SIGINT cleanup check.

Requires a stationary paused physical ego with no rig or other clients. Does
not load a map, resume physics, move the car, record video or kill the engine.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'nav2_adapter'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--rig', type=Path, required=True)
    p.add_argument('--engine-pid', type=int, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    from beamngpy import BeamNGpy
    from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
    from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
    from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors
    install_fail_closed_reads(PrefixedLengthSocket)
    report = dict(passed=False, phases=[], errors=[], map_loads=0, teleports=0,
                  physics_resumed=False, actual_engine_killed=False, video_recording=False)
    bng = child = None
    try:
        bng = BeamNGpy('127.0.0.1', 25252, socket_timeout=5).open(launch=False)
        before = read_physics_snapshot(bng)
        report['before'] = before
        assert not physical_snapshot_errors(before, require_sensors=False)
        assert before['speed'] < .05 and not before['cameras'] and not before['lidars']
        for signame in ('SIGTERM', 'SIGINT'):
            path = args.output / (signame + '.json')
            command = [sys.executable, str(Path(__file__).with_name('attach_current_vehicle_sensors.py')),
                       '--output', str(path), '--rig', str(args.rig),
                       '--engine-pid', str(args.engine_pid)]
            with (args.output / (signame + '.log')).open('x') as log:
                child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                         start_new_session=True)
            phase = dict(signal=signame, pid=child.pid, command=command)
            report['phases'].append(phase)
            (args.output / 'owned.json').write_text(json.dumps(report['phases'], indent=2))
            deadline = time.monotonic() + 60
            while not path.exists():
                if child.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Attachment failed or readiness deadline exceeded')
                time.sleep(.1)
            attached = json.loads(path.read_text())
            assert attached.get('ready') and attached['after']['vehicleID'] == before['vehicleID']
            child.send_signal(getattr(signal, signame))
            phase['exit'] = child.wait(timeout=15)
            phase['attachment'] = json.loads(path.read_text())
            after = read_physics_snapshot(bng)
            phase['after'] = after
            assert phase['exit'] == 0 and phase['attachment'].get('stopped')
            assert not any('error' in key for key in phase['attachment'])
            assert phase['attachment']['owning_engine_alive_at_cleanup']
            assert not after['cameras'] and not after['lidars']
            assert not physical_snapshot_errors(after, require_sensors=False)
            assert after['vehicleID'] == before['vehicleID'] and after['mission'] == before['mission']
            assert after['position'] == before['position'] and after['speed'] < .05
            phase['passed'] = True
            child = None
        report['passed'] = True
    except BaseException as exc:
        report['errors'].append(repr(exc))
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGTERM)
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait()
                report['errors'].append('Owned attachment required force stop; sensor cleanup unverified')
        if bng is not None:
            bng.disconnect()
        (args.output / 'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(dict(passed=report['passed'], errors=report['errors'],
                              phases=len(report['phases']))), flush=True)
    return 0 if report['passed'] and not report['errors'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
