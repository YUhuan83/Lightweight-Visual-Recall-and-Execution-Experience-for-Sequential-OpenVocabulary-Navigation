"""Read engine methods and wheel-node membership without changing physics."""
from pathlib import Path
import json
from beamngpy import BeamNGpy

root = Path('/home/ly/full_town_memory_20261002/body_collision')
root.mkdir(exist_ok=True)
bng = BeamNGpy('127.0.0.1', 25252, socket_timeout=8)
bng.open(launch=False)
vehicle = bng.vehicles.get_current()['ego']
vehicle.connect(bng)
try:
    lua = '''
local out={methods={},wheelNodes={},nodes={},wheels={}}
for _,name in ipairs({'setNodeCollision','setNodeStaticCollision','setNodeCollisionEnabled',
  'setNodeCollisionMode','getNodeCollision','getNodeStaticCollision','setNode','setInitialNodeCollision',
  'setGhostEnabled','setStaticCollisionEnabled','setCollisionEnabled','getNodeCount',
  'getNodePosition','getNodePositionRelative','getNodePositionRelativeXYZ','setNodePosition'}) do
  local ok,value=pcall(function() return obj[name] end)
  out.methods[name]=ok and type(value) or tostring(value)
end
local protected={}
for _,wheel in pairs(v.data.wheels or {}) do
  local data={name=wheel.name,node1=wheel.node1,node2=wheel.node2,wheelID=wheel.wheelID,nodes={}}
  for _,id in pairs(wheel.nodes or {}) do protected[id]=true;table.insert(data.nodes,id) end
  if wheel.node1 then protected[wheel.node1]=true end
  if wheel.node2 then protected[wheel.node2]=true end
  table.insert(out.wheels,data)
end
for id,node in pairs(v.data.nodes) do
  if node.wheelID~=nil then protected[node.cid]=true end
  local data={key=id,cid=node.cid,name=node.name or '',pos=node.pos:toTable()}
  for k,val in pairs(node) do
    if type(val)=='number' or type(val)=='string' then data[tostring(k)]=val
    elseif type(val)=='boolean' then data[tostring(k)]=tostring(val) end
  end
  data.protected=protected[node.cid] and 'true' or 'false'
  table.insert(out.nodes,data)
end
for id,_ in pairs(protected) do table.insert(out.wheelNodes,id) end
table.sort(out.wheelNodes)
return jsonEncode(out)
'''
    result = json.loads(vehicle.queue_lua_command(lua, response=True))
    (root / 'api_and_nodes.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({'methods': result['methods'], 'nodes':len(result['nodes']),
                      'wheel_nodes':len(result['wheelNodes']),
                      'wheels':[{k:v for k,v in w.items() if k!='nodes'} for w in result['wheels']]}))
finally:
    vehicle.disconnect()
    bng.disconnect()
