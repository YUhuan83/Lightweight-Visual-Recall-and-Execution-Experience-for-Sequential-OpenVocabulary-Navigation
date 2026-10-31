"""Load one fresh, physical SBR scene at the validated Town spawn."""
import argparse
import json
import math
from pathlib import Path

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors


LEVEL = "east_coast_usa"
SPAWN = (734.294, -11.946, 52.154)
HEADING = (0.0, 0.0, 0.7658087977066911, 0.6430683364581344)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)

    install_fail_closed_reads(PrefixedLengthSocket)
    bng = BeamNGpy(
        "127.0.0.1",
        25252,
        home="/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0",
        user="/home/ly/beamng-east-coast-user/current",
        headless=True,
        quit_on_close=False,
        socket_timeout=20,
    ).open(launch=False)
    report = {
        "passed": False,
        "scenario": "East Coast Town gas-station spawn; physical SBR",
        "map_reload_count": 1,
        "vehicle_teleports": 0,
        "collision_override_requested": False,
        "spawn_expected": SPAWN,
    }
    try:
        levels = bng.scenario.get_levels()
        if LEVEL not in levels:
            raise RuntimeError(f"Required map unavailable: {LEVEL}")

        scenario = Scenario(LEVEL, "wildos_nav_transaction_20261013")
        vehicle = Vehicle(
            "ego",
            "sbr",
            license="WILDOS",
            color="red",
            part_config="vehicles/sbr/TT_AWD_S2_DCT.pc",
        )
        scenario.add_vehicle(vehicle, pos=SPAWN, rot_quat=HEADING, safe_spawn=False)
        scenario.make(bng)
        report["scenario_path"] = scenario.path
        bng.scenario.load(scenario, connect_player_vehicle=False, connect_existing_vehicles=False)
        bng.scenario.start()
        bng.env.set_tod(0.0)

        snapshot = read_physics_snapshot(bng)
        report["physics_snapshot"] = snapshot
        errors = physical_snapshot_errors(snapshot, require_sensors=False)
        mission = str(snapshot.get("mission", "")).lower()
        if LEVEL not in mission:
            errors.append(f"Unexpected loaded map: {mission}")
        position = snapshot.get("position", [])
        if len(position) < 2 or math.dist(position[:2], SPAWN[:2]) > 1.25:
            errors.append("Vehicle did not spawn within 1.25 m of the Town start")
        if snapshot.get("speed", 99.0) > 0.15:
            errors.append("Vehicle is moving before the test stack starts")
        if errors:
            raise RuntimeError("Physical scene check failed: " + "; ".join(errors))
        report["passed"] = True
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        bng.disconnect()
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
