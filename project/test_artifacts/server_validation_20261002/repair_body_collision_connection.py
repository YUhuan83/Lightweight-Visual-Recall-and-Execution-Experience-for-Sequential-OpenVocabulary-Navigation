"""Restore the port lost by BeamNG's respawn deserialization shadowing bug."""
from pathlib import Path
import json
import time
from beamngpy import BeamNGpy
root=Path('/home/ly/full_town_memory_20261002/body_collision')
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8);b.open(launch=False)
try:
    source='''
local data=tech_techCore.onSerialize()
local result={runningPort=data.runningPort,runningPortType=type(data.runningPort),repaired=false}
local runningPort=tonumber(data.runningPort)
if runningPort and runningPort>0 then
for i=1,30 do
  local name,value=debug.getupvalue(tech_techCore.startConnection,i)
  if not name then break end
  if name=='port' then
    result.previousPort=value or 'nil'
    debug.setupvalue(tech_techCore.startConnection,i,runningPort)
    result.repaired=true
    break
  end
end
end
obj:queueGameEngineLua('wildosVehiclePortRepair='..serialize(result))
'''
    separator='='
    command='scenetree.findObject("ego"):queueLuaCommand(['+separator+'['+source+']'+separator+']); return "repair queued"'
    b.control.queue_lua_command(command,response=True)
    deadline=time.monotonic()+8
    result={}
    while time.monotonic()<deadline:
        result=json.loads(b.control.queue_lua_command('return jsonEncode(wildosVehiclePortRepair or {})',response=True))
        if result.get('repaired'):break
        time.sleep(.1)
    (root/'connection_repair.json').write_text(json.dumps(result,indent=2))
    if not result.get('repaired'):raise RuntimeError('Port repair not confirmed: '+str(result))
    print(json.dumps(result),flush=True)
    vehicle=b.vehicles.get_current()['ego'];vehicle.connect(b)
    vehicle.control(throttle=0,brake=1,parkingbrake=1,steering=0,clutch=0,gear=0)
    vehicle.sensors.poll()
    print(json.dumps({'vehicle_connection_restored':True,'position':vehicle.state['pos']}),flush=True)
    vehicle.disconnect()
finally:b.control.pause();b.disconnect()
