"""Brake the spawned physical SBR without changing its map or pose."""
import argparse
import json
import math
from pathlib import Path
import time

from beamngpy import BeamNGpy
from beamngpy.connection import Connection
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    install_fail_closed_reads(PrefixedLengthSocket)
    bng = BeamNGpy("127.0.0.1", 25252, socket_timeout=5).open(launch=False)
    report = {"passed": False, "map_reload_count": 0, "vehicle_teleports": 0}
    vehicle = None
    try:
        report["before"] = read_physics_snapshot(bng)
        vehicle = bng.vehicles.get_current().get("ego")
        if vehicle is None or vehicle.vid != "ego":
            raise RuntimeError("Current scene does not own the expected ego vehicle")
        endpoint = bng.vehicles.start_connection(vehicle, vehicle.extensions)
        if endpoint.get("vid") != vehicle.vid:
            raise RuntimeError("Current engine returned a different vehicle endpoint owner")
        port = endpoint.get("result")
        if isinstance(port, bool) or not isinstance(port, (int, float)) or int(port) != port:
            raise RuntimeError("Current vehicle endpoint is not a valid port")
        vehicle.connection = Connection("127.0.0.1", int(port), timeout=5)
        vehicle.connect(bng, tries=1)
        vehicle.ai.set_mode("disabled")
        vehicle.set_shift_mode("realistic_automatic")
        started = time.monotonic()
        stable_since = None
        samples = []
        while time.monotonic() - started < 8.0:
            vehicle.control(throttle=0.0, brake=1.0, parkingbrake=1.0,
                            steering=0.0, clutch=0.0, gear=0)
            snapshot = read_physics_snapshot(bng)
            samples.append({"elapsed_s": time.monotonic() - started,
                            "position": snapshot["position"], "speed_m_s": snapshot["speed"]})
            if snapshot["speed"] < 0.05:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 0.5:
                    break
            else:
                stable_since = None
            time.sleep(0.05)
        bng.control.pause()
        after = read_physics_snapshot(bng)
        report.update(vehicle_port=int(port), samples=samples, after=after,
                      stopped=after.get("speed", 99.0) < 0.05)
        report["displacement_m"] = math.dist(report["before"]["position"], after["position"])
        report["passed"] = bool(report["stopped"] and after["mission"] == report["before"]["mission"]
                                 and report["displacement_m"] < 3.0)
        if not report["passed"]:
            report["error"] = "Vehicle did not reach a safe stationary state"
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if vehicle is not None:
            try:
                vehicle.disconnect()
            except Exception as exc:
                report["cleanup_error"] = f"vehicle_disconnect: {exc}"
                report["passed"] = False
        try:
            bng.disconnect()
        except Exception as exc:
            report["cleanup_error"] = f"beamng_disconnect: {exc}"
            report["passed"] = False
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({key: report.get(key) for key in
                      ("passed", "stopped", "displacement_m", "vehicle_port", "error")}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
