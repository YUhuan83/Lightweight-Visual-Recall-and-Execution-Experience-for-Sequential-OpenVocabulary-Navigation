"""Toggle body-node collisions while preserving wheel contact and the map.

Stop active vehicle controllers before running this tool. Only the vehicle is
respawned; the loaded map and unrelated processes are never restarted.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

PROFILE_VERSION = 'body-nodes-v1'


def lua_literal(text: str) -> str:
    separator = '='
    while f']{separator}]' in text:
        separator += '='
    return f'[{separator}[{text}]{separator}]'


def set_body_collision(*, host: str, port: int, vehicle_name: str,
                       disabled: bool, extension: Path, output: Path) -> dict:
    from beamngpy import BeamNGpy
    from beamngpy.connection import Connection

    output.mkdir(parents=True, exist_ok=True)
    source = extension.read_text(encoding='utf-8')
    bng = BeamNGpy(host, port, socket_timeout=8)
    bng.open(launch=False)
    vehicle = None
    result = {'requested_disabled': disabled, 'map_reload_count': 0,
              'profile_version': PROFILE_VERSION}
    try:
        bng.control.pause()
        name = lua_literal(vehicle_name)
        before = f'''
local veh=scenetree.findObject({name})
assert(veh, 'Requested vehicle missing')
local bundle=core_vehicle_manager.getVehicleData(veh:getID())
local masks={{}}
for _,node in pairs(bundle.vdata.nodes) do
  masks[tostring(node.cid)]=veh:getInitialNodeCollision(node.cid)
end
return jsonEncode({{vehicleID=veh:getID(),config=bundle.config,nodeMasks=masks,
  cameras=extensions.tech_sensors.getActiveCameraSensors(),
  lidars=extensions.tech_sensors.getActiveLidarSensors(),
  mission=getMissionFilename()}})
'''
        result['before'] = json.loads(bng.control.queue_lua_command(before, response=True))
        # Establish a vehicle socket before the respawn. BeamNG.tech 0.39.4
        # can omit the new port in its post-respawn response; the existing
        # listener is reusable and gives us a deterministic fallback.
        vehicle = bng.vehicles.get_current()[vehicle_name]
        vehicle.connect(bng)
        known_port = vehicle.connection.port
        vehicle.disconnect()
        result['known_vehicle_port'] = known_port
        (output / 'before.json').write_text(json.dumps(result['before'], indent=2), encoding='utf-8')
        install = f'''
local source={lua_literal(source)}
assert(loadstring(source), 'Body collision extension syntax invalid')
local target='lua/ge/extensions/wildosBodyCollision.lua'
local previous=readFile(target)
if previous and previous~=source then
  writeFile(target..'.before_body_nodes_v1.bak', previous)
end
FS:directoryCreate('lua/ge/extensions')
writeFile(target,source)
assert(readFile(target)==source, 'Extension file verification failed')
extensions.unload('wildosBodyCollision')
extensions.load('wildosBodyCollision')
assert(extensions.wildosBodyCollision, 'Body collision extension did not load')
return jsonEncode({{installed=FS:getUserPath()..target}})
'''
        result['installation'] = json.loads(bng.control.queue_lua_command(install, response=True))
        # The mode/version are part of the loader cache key. Do not mutate the
        # live physics nodes with setNode(), which also resets other properties.
        apply = f'''
local veh=scenetree.findObject({name})
core_vehicle_partmgmt.setConfigOfVehicle(veh, {{
  wildosBodyCollisionDisabled={'true' if disabled else 'false'},
  wildosCollisionProfileVersion='{PROFILE_VERSION}'
}}, true)
return 'vehicle respawn requested'
'''
        bng.control.queue_lua_command(apply, response=True)
        deadline = time.monotonic() + 30
        while True:
            query = f'''
local veh=scenetree.findObject({name})
if not veh then return '{{}}' end
local out=extensions.wildosBodyCollision.status(veh:getID())
out.vehicleID=veh:getID()
out.mission=getMissionFilename()
local bundle=core_vehicle_manager.getVehicleData(veh:getID())
out.nodeMasks={{}}
for _,node in pairs(bundle.vdata.nodes) do
  out.nodeMasks[tostring(node.cid)]=veh:getInitialNodeCollision(node.cid)
end
out.cameras=extensions.tech_sensors.getActiveCameraSensors()
out.lidars=extensions.tech_sensors.getActiveLidarSensors()
return jsonEncode(out)
'''
            status = json.loads(bng.control.queue_lua_command(query, response=True))
            if status.get('configMode') is disabled and (not disabled or status.get('liveVerified')):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Body collision profile did not apply: {status}')
            time.sleep(.2)
        vehicle = bng.vehicles.get_current()[vehicle_name]
        vehicle.connection = Connection(host, int(known_port), timeout=8)
        vehicle.connect(bng)
        vehicle.control(throttle=0, brake=1, parkingbrake=1, steering=0, clutch=0, gear=0)
        vehicle.sensors.poll()
        result['after'] = status
        result['state'] = dict(vehicle.state)
        if status['mission'] != result['before']['mission']:
            raise RuntimeError('Loaded map changed unexpectedly')
        if disabled:
            counts = status['liveCounts']
            zero_masks = sum(mask == 0 for mask in status['nodeMasks'].values())
            if zero_masks < counts['bodyNodeCount']:
                raise RuntimeError('Engine collision masks do not match disabled body nodes')
        result['paused'] = True
        result['applied'] = True
        return result
    except BaseException as exc:
        result['error'] = repr(exc)
        raise
    finally:
        try:
            if vehicle is not None:
                vehicle.control(throttle=0, brake=1, parkingbrake=1, steering=0, clutch=0, gear=0)
                vehicle.disconnect()
            bng.control.pause()
        finally:
            bng.disconnect()
            (output / 'application.json').write_text(json.dumps(result, indent=2), encoding='utf-8')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=25252)
    parser.add_argument('--vehicle', default='ego')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--disable', action='store_true')
    mode.add_argument('--restore', action='store_true')
    parser.add_argument('--extension', type=Path,
                        default=Path(__file__).parent/'beamng_extensions/wildosBodyCollision.lua')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = set_body_collision(host=args.host, port=args.port, vehicle_name=args.vehicle,
                               disabled=args.disable, extension=args.extension, output=args.output)
    print(json.dumps({k: result[k] for k in ('applied', 'paused', 'map_reload_count')}
                     | {'counts': result['after'].get('liveCounts')}), flush=True)


if __name__ == '__main__':
    main()
