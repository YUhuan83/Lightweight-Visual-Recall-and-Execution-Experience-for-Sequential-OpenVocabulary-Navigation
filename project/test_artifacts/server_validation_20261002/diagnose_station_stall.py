"""Read powertrain and wheel state on the currently paused failed vehicle."""
import json
from pathlib import Path
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics, Damage
ROOT=Path('/home/ly/full_town_memory_20261002')
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8);b.open(launch=False)
v=b.vehicles.get_current()['ego'];v.connect(b)
v.sensors.attach('station_stall_electrics',Electrics());v.sensors.attach('station_stall_damage',Damage())
lua='''
local result={devices={},wheels={}}
for name,device in pairs(powertrain.getDevices()) do
  local data={name=name}
  for key,value in pairs(device) do
    if type(value)=='number' or type(value)=='boolean' or type(value)=='string' then
      if type(value)~='number' or (value==value and value~=math.huge and value~=-math.huge) then data[tostring(key)]=type(value)=='boolean' and tostring(value) or value end
    end
  end
  table.insert(result.devices,data)
end
for key,w in pairs(wheels.wheels or {}) do
  local data={key=key}
  for field,value in pairs(w) do
    if type(value)=='number' or type(value)=='boolean' or type(value)=='string' then
      if type(value)~='number' or (value==value and value~=math.huge and value~=-math.huge) then data[tostring(field)]=type(value)=='boolean' and tostring(value) or value end
    end
  end
  table.insert(result.wheels,data)
end
return jsonEncode(result)
'''
try:
    v.sensors.poll()
    result={'state':dict(v.state),'bbox':v.get_bbox(),
        'damage':dict(v.sensors['station_stall_damage']),
        'electrics':dict(v.sensors['station_stall_electrics']),
        'lua':json.loads(v.queue_lua_command(lua,response=True))}
    (ROOT/'station_stall_diagnostics.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({'devices':result['lua']['devices'],
        'wheels':[{k:w.get(k) for k in ('name','angularVelocity','brakeTorque','propulsed','isBroken','isTireDeflated','lastSlip')} for w in result['lua']['wheels']]}),flush=True)
finally:
    v.sensors.detach('station_stall_electrics');v.sensors.detach('station_stall_damage');v.disconnect();b.disconnect()
