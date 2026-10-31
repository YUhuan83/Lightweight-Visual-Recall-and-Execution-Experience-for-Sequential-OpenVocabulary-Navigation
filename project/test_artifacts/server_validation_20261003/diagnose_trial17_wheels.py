"""Read the stopped vehicle; no physics steps, driving or teleporting."""
from pathlib import Path
import json
from beamngpy import BeamNGpy
from beamngpy.sensors import Damage, Electrics

ROOT = Path('/home/ly/town_memory_validation_20261003')
b = BeamNGpy('127.0.0.1', 25252, socket_timeout=8).open(launch=False)
vehicle = b.vehicles.get_current()['ego']
vehicle.connect(b)
vehicle.sensors.attach('trial17_wheel_probe_electrics', Electrics())
vehicle.sensors.attach('trial17_wheel_probe_damage', Damage())
lua = '''
local out={devices={},wheels={}}
for name,device in pairs(powertrain.getDevices()) do
  local data={name=name}
  for key,val in pairs(device) do
    if type(val)=='number' or type(val)=='boolean' or type(val)=='string' then
      if type(val)~='number' or (val==val and val~=math.huge and val~=-math.huge) then
        data[tostring(key)]=val
      end
    end
  end
  table.insert(out.devices,data)
end
local origin=obj:getPosition()
for key,w in pairs(wheels.wheels or {}) do
  local data={key=key}
  for field,val in pairs(w) do
    if type(val)=='number' or type(val)=='boolean' or type(val)=='string' then
      if type(val)~='number' or (val==val and val~=math.huge and val~=-math.huge) then
        data[tostring(field)]=val
      end
    end
  end
  data.axleNode1Position=(origin+obj:getNodePosition(w.node1)):toTable()
  data.axleNode2Position=(origin+obj:getNodePosition(w.node2)):toTable()
  table.insert(out.wheels,data)
end
return jsonEncode(out)
'''
try:
    vehicle.sensors.poll()
    result = dict(state=dict(vehicle.state), bbox=vehicle.get_bbox(),
                  damage=dict(vehicle.sensors['trial17_wheel_probe_damage']),
                  electrics=dict(vehicle.sensors['trial17_wheel_probe_electrics']),
                  physics=json.loads(vehicle.queue_lua_command(lua, response=True)))
    (ROOT/'trial17_wheel_diagnostics.json').write_text(json.dumps(result, indent=2))
    keys = ('name','angularVelocity','wheelSpeed','downForce','downForceRaw','contactDepth',
            'contactMaterialID1','contactMaterialID2','propulsionTorque','brakeTorque',
            'lastSlip','lastSideSlip','isBroken','isTireDeflated','radius','axleNode1Position')
    print(json.dumps(dict(state=result['state'], damage=result['damage'].get('damage'),
       electrics={k:result['electrics'].get(k) for k in ('rpm','throttle','brake','parkingbrake',
           'wheelspeed','gear','clutchRatio1','engineRunning')},
       wheels=[{k:w.get(k) for k in keys} for w in result['physics']['wheels']])), flush=True)
finally:
    vehicle.sensors.detach('trial17_wheel_probe_electrics')
    vehicle.sensors.detach('trial17_wheel_probe_damage')
    vehicle.disconnect()
    b.disconnect()
