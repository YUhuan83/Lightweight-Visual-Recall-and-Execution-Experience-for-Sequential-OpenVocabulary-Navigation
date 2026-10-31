"""Reconnect using the actual listener recorded at vehicle respawn."""
from pathlib import Path
import json
from beamngpy import BeamNGpy
from beamngpy.connection import Connection

root=Path('/home/ly/full_town_memory_20261002/body_collision')
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8);b.open(launch=False)
vehicle=None
try:
    vehicle=b.vehicles.get_current()['ego']
    vehicle.connection=Connection('127.0.0.1',43675,timeout=8)
    vehicle.connect(b)
    vehicle.control(throttle=0,brake=1,parkingbrake=1,steering=0,clutch=0,gear=0)
    vehicle.sensors.poll()
    serialized=json.loads(vehicle.queue_lua_command('return jsonEncode(tech_techCore.onSerialize())',response=True))
    result={'vehicle_connection_restored':True,'position':vehicle.state['pos'],'listener':serialized}
    (root/'direct_reconnect.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
finally:
    if vehicle and vehicle.is_connected():vehicle.disconnect()
    b.control.pause();b.disconnect()
