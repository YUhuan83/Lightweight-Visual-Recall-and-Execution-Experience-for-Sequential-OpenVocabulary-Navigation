"""Load one Town map with the original SBR and the agreed left-facing start."""

import argparse
import json
import math
from pathlib import Path
import time
from beamngpy import BeamNGpy, Scenario, Vehicle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--home", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=25252)
    args = parser.parse_args()
    bng = BeamNGpy(
        args.host,
        args.port,
        home=args.home,
        user=args.user,
        quit_on_close=False,
        # Map loading can exceed 20 s on the shared host. This connection is
        # used only for scene startup; live control uses separate connections.
        socket_timeout=120,
    ).open(launch=False)
    vehicle = Vehicle(
        "ego",
        "sbr",
        license="WILDOS",
        color="red",
        part_config="vehicles/sbr/TT_AWD_S2_DCT.pc",
    )
    try:
        scene = Scenario("east_coast_usa", "wildos_cleanroom_town")
        # Match the verified v74 Town orientation (BeamNG quaternion convention).
        heading = 2 * math.atan2(0.7658087977066911, 0.6430683364581344) - math.pi / 2
        scene.add_vehicle(
            vehicle,
            pos=(734.294, -11.946, 52.154),
            rot_quat=(0.0, 0.0, math.sin(heading / 2), math.cos(heading / 2)),
            safe_spawn=False,
        )
        scene.make(bng)
        bng.scenario.load(
            scene, connect_player_vehicle=False, connect_existing_vehicles=False
        )
        bng.scenario.start()
        bng.env.set_tod(0.0)
        vehicle.connect(bng)
        vehicle.set_shift_mode("realistic_automatic")
        vehicle.control(throttle=0.0, brake=1.0, parkingbrake=0.0, steering=0.0)
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            vehicle.sensors.poll()
            if math.hypot(*vehicle.state["vel"][:2]) < 0.05:
                break
            time.sleep(0.1)
        bng.control.pause()
        args.output.write_text(
            json.dumps(dict(ready=True, map_loads=1, original_physics=True))
        )
    finally:
        vehicle.disconnect()
        bng.disconnect()


if __name__ == "__main__":
    main()
