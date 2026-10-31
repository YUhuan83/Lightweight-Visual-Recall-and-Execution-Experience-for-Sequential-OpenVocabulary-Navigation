"""Headless motion probe: can the local pickup actually move?"""
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "integration"))
import numpy as np
import capture_american as ca

HOME = r"D:\BEAMNG\BeamNG.tech.v0.39.4.0"
USER = r"D:\BEAMNG\rural-user"
ZIP = r"D:\BEAMNG\rural-user\current\mods\american_road_0.39.zip"

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.connection import Connection
from beamngpy.sensors import Electrics

spawn = ca.read_spawn(Path(ZIP), "spawns_city")
bng = BeamNGpy("127.0.0.1", 64256, home=HOME, user=USER, binary="Bin64/BeamNG.tech.x64.exe",
               quit_on_close=False, socket_timeout=300, headless=True)
bng.open(launch=True, listen_ip="127.0.0.1")
scene = Scenario("american_road", "probe", description="motion probe")
car = Vehicle("probe_ego", model="pickup", part_config="vehicles/pickup/d15_4wd_A.pc")
scene.add_vehicle(car, pos=tuple(spawn["position"]), rot_quat=tuple(spawn["rot_quat"]), safe_spawn=True)
scene.make(bng)
bng.scenario.load(scene, connect_player_vehicle=False, connect_existing_vehicles=False)
ep = bng.vehicles.start_connection(car, car.extensions)
car.connection = Connection("127.0.0.1", int(ep["result"]), timeout=300)
car.connect(bng)
car.sensors.attach("electrics", Electrics())
bng.scenario.start()
car.ai.set_mode("disabled")
car.set_shift_mode("realistic_automatic")
car.control(throttle=0, brake=1, parkingbrake=1, steering=0, gear=0)
bng.control.resume()
time.sleep(3)

def snapshot(tag):
    car.sensors.poll()
    st = dict(car.state)
    el = car.sensors["electrics"].get("gear"), car.sensors["electrics"].get("throttle"), \
         car.sensors["electrics"].get("brake"), car.sensors["electrics"].get("parkingbrake"), \
         car.sensors["electrics"].get("rpm"), car.sensors["electrics"].get("wheelspeed"), \
         car.sensors["electrics"].get("engineRunning")
    print("%-10s pos=%s vel=%.3f gear/thr/brk/pb/rpm/ws/eng=%s" % (
        tag, [round(v, 2) for v in st["pos"]], float(np.linalg.norm(st["vel"])), el), flush=True)

snapshot("before")
print("full throttle 6s ...", flush=True)
deadline = time.monotonic() + 6
while time.monotonic() < deadline:
    car.control(steering=0.0, throttle=1.0, brake=0.0, parkingbrake=0, gear=1)
    time.sleep(0.1)
snapshot("after")
print("brake", flush=True)
car.control(throttle=0, brake=1, parkingbrake=1, steering=0, gear=0)
bng.disconnect()
