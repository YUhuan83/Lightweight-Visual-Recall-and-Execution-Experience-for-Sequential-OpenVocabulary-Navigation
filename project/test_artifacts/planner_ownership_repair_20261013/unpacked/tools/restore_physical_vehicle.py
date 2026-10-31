"""Restore original ego JBeam after sensor and motion clients have stopped.

Read-only by default. --apply changes only the vehicle on the loaded map.
Reattach the current sensor rig and run physical preflight before arming.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'nav2_adapter'))
from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors


def original_config(document):
    config = dict(document.get('config', document))
    if (config.get('model') != 'sbr'
            or config.get('partConfigFilename') != 'vehicles/sbr/TT_AWD_S2_DCT.pc'
            or not config.get('partsTree')):
        raise ValueError('Expected original Town SBR config with complete parts tree')
    if any(config.get(key) is True for key in ('wildosKinematicProfile', 'wildosBodyCollisionDisabled')):
        raise ValueError('Saved config is a physics override, not the original car')
    for key in tuple(config):
        if key.startswith('wildos'):
            del config[key]
    return config


def active_vehicle_clients():
    markers = ('kinematic_vehicle_control', 'native_vehicle_control', 'cmd_vel_to_control',
               'beamng_bridge', 'beamng_agent', 'sensor_bridge.py', 'attach_fixed_vehicle.py',
               'attach_current_vehicle_sensors.py')
    clients = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name) == os.getpid():
            continue
        try:
            tokens = p.joinpath('cmdline').read_bytes().decode(errors='replace').split('\0')
            tokens = [t for t in tokens if t]
            if not tokens or Path(tokens[0]).name in ('bash', 'sh', 'ssh', 'timeout'):
                continue
            env = dict(value.split('=', 1) for value in p.joinpath('environ').read_bytes()
                       .decode(errors='replace').split('\0') if '=' in value)
            # BeamNG owns a single vehicle globally, independent of ROS domain.
            if any(
                    any(t == marker or Path(t).name == marker or t.endswith('.'+marker)
                        for marker in markers) for t in tokens):
                clients.append(dict(pid=int(p.name), domain=env.get('ROS_DOMAIN_ID', '0'),
                                    command=' '.join(tokens)))
        except (OSError, ValueError):
            continue
    return clients


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--original-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    config = original_config(json.loads(args.original_config.read_text()))
    report = dict(applied=False, passed=False, map_reload_count=0,
                  scope='Vehicle JBeam only; sensor and navigation acceptance pending',
                  original_config=str(args.original_config), phase='inspection')
    if args.output.exists():
        raise FileExistsError('Use a new output evidence path')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    bng = None
    try:
        from beamngpy import BeamNGpy
        bng = BeamNGpy('127.0.0.1', 25252, socket_timeout=5).open(launch=False)
        before = read_physics_snapshot(bng)
        report['before'] = before
        report['active_clients'] = active_vehicle_clients()
        if not args.apply:
            report['phase'] = 'read_only'
            report['physics_errors'] = physical_snapshot_errors(before)
            report['passed'] = not report['physics_errors']
        else:
            if report['active_clients']:
                raise RuntimeError('Stop the listed sensor/motion clients before respawning ego')
            if before.get('speed', 99.) > .15:
                raise RuntimeError('Vehicle must be stationary before restoring physics')
            bng.control.pause()
            report['phase'] = 'rebuild_original_jbeam'
            payload = json.dumps(json.dumps(config))
            bng.control.queue_lua_command(
                "local v=scenetree.findObject('ego'); assert(v); "
                "local kinematic=rawget(extensions,'wildosKinematic'); "
                "if kinematic then kinematic.stop(); "
                "extensions.unload('wildosKinematic') end; "
                "if rawget(extensions,'wildosBodyCollision') then extensions.unload('wildosBodyCollision') end; "
                "extensions.tech_sensors.removeAllSensorsFromVehicle(v:getID()); "
                f"core_vehicle_partmgmt.setConfigOfVehicle(v,jsonDecode({payload}),true); return true",
                response=True)
            report['applied'] = True
            time.sleep(2.)
            after = read_physics_snapshot(bng)
            report['after'] = after
            report['physics_errors'] = physical_snapshot_errors(after, require_sensors=False)
            if after['mission'] != before['mission']:
                report['physics_errors'].append('Loaded map changed')
            if math.dist(after['position'], before['position']) > .5:
                report['physics_errors'].append('Vehicle pose changed during paused rebuild')
            report['passed'] = not report['physics_errors']
            report['phase'] = 'sensor_reattachment_required'
            report['simulation_paused'] = True
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'[:500]
    finally:
        if bng is not None:
            if args.apply:
                try:
                    bng.control.pause()
                except Exception as exc:
                    report['pause_error'] = str(exc)[:240]
                    report['passed'] = False
            try:
                bng.disconnect()
            except Exception as exc:
                report['disconnect_error'] = str(exc)[:240]
                report['passed'] = False
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: report.get(key) for key in ('passed', 'applied', 'phase', 'error')}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
