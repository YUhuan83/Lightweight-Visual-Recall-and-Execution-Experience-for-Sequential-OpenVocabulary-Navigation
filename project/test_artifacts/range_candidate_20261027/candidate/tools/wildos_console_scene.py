"""Prepare a new console's single Town map, original SBR physics and stationary start."""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--home', required=True)
    parser.add_argument('--user', required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    from beamngpy import BeamNGpy, Scenario, Vehicle
    from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
    from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
    from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors
    from check_continuous_start_execution import settle
    install_fail_closed_reads(PrefixedLengthSocket)
    bng = BeamNGpy('127.0.0.1', 25252, home=args.home, user=args.user,
                   quit_on_close=False, socket_timeout=20).open(launch=False)
    report = dict(passed=False, map_loads=0, teleports=0, collision_override=False)
    vehicle = None
    try:
        scenario = Scenario('east_coast_usa', 'wildos_interactive_town')
        vehicle = Vehicle('ego', 'sbr', license='WILDOS', color='red',
                          part_config='vehicles/sbr/TT_AWD_S2_DCT.pc')
        scenario.add_vehicle(vehicle, pos=(734.294, -11.946, 52.154),
            rot_quat=(0., 0., .7658087977066911, .6430683364581344), safe_spawn=False)
        scenario.make(bng)
        bng.scenario.load(scenario, connect_player_vehicle=False, connect_existing_vehicles=False)
        report['map_loads'] = 1
        bng.scenario.start()
        bng.env.set_tod(0.)
        vehicle.connect(bng)
        # Match native_vehicle_control's contract before the first brake input.
        # In the default assisted mode a brake input can command reverse motion.
        vehicle.set_shift_mode('realistic_automatic')
        vehicle.control(throttle=0., brake=1., parkingbrake=0., steering=0.)
        observed = time.monotonic()
        telemetry = args.output.with_suffix('.samples.jsonl')
        report['telemetry'] = str(telemetry)
        def snapshot():
            value = read_physics_snapshot(bng)
            with telemetry.open('a') as log:
                log.write(json.dumps(dict(elapsed_s=time.monotonic()-observed, **value))+'\n')
            return value
        after, samples = settle(snapshot, time.sleep, timeout=12.)
        errors = physical_snapshot_errors(after, require_sensors=False)
        if 'east_coast_usa' not in after.get('mission', ''):
            errors.append('wrong map')
        if math.dist(after['position'][:2], (734.294, -11.946)) > 1.25:
            errors.append('spawn drift exceeds 1.25m')
        report.update(physics=after, stop_samples=samples)
        if errors:
            raise RuntimeError(str(errors))
        report['passed'] = True
    except BaseException as exc:
        report['error'] = repr(exc)
    finally:
        try:
            bng.control.pause()
        finally:
            if vehicle and vehicle.is_connected():
                vehicle.disconnect()
            bng.disconnect()
            args.output.write_text(json.dumps(report, indent=2)+'\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
