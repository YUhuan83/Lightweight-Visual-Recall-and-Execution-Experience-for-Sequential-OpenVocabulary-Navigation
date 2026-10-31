"""Read live wheel nodes and reference nodes, without changing controls."""
import inspect
import json
from pathlib import Path
from beamngpy import BeamNGpy

root = Path('/home/ly/full_town_memory_20261002')
b = BeamNGpy('127.0.0.1', 25252, socket_timeout=10)
b.open(launch=False)
v = b.vehicles.get_current()['ego']
v.connect(b)
try:
    v.sensors.poll()
    print(inspect.getsource(v.queue_lua_command), flush=True)
    lua = '''
local result = {refNodes=v.data.refNodes, wheels={}, nodes={}, position=obj:getPosition(),
                direction=obj:getDirectionVector(), up=obj:getDirectionVectorUp()}
for k,w in pairs(wheels.wheels or {}) do
  local data = {key=k, name=w.name, node1=w.node1, node2=w.node2,
                radius=w.radius, wheelDir=w.wheelDir}
  for _,n in ipairs({w.node1,w.node2}) do
    if n then
      local p=obj:getNodePosition(n)
      data[tostring(n)]={x=p.x,y=p.y,z=p.z,definition=v.data.nodes[n]}
    end
  end
  table.insert(result.wheels,data)
end
for k,n in pairs(v.data.nodes) do
  if n.name and (string.find(n.name,"ref") or string.find(n.name,"axle")) then
    local p=obj:getNodePosition(n.cid)
    table.insert(result.nodes,{name=n.name,cid=n.cid,x=p.x,y=p.y,z=p.z})
  end
end
return jsonEncode(result)
'''
    result = {'state': dict(v.state), 'bbox': v.get_bbox(),
              'lua': v.queue_lua_command(lua, response=True)}
    root.joinpath('wheel_geometry_probe.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2),flush=True)
finally:
    v.disconnect()
    b.disconnect()
