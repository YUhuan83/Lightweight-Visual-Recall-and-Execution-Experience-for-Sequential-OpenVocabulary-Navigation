-- Diagnostic auxiliary controller. Reads collision events; writes no inputs/physics.
local M = {type = 'auxiliary', protocol = 'wildos_contact_audit_v1'}
local MAX_NODES, MAX_OUTPUT = 1024, 128
local nodes, classEvents, distinct, invalid, dropped, total, sequence, began
local function finite(value)
  return type(value) == 'number' and value == value and value ~= math.huge and value ~= -math.huge
end
local function resetWindow()
  nodes, classEvents, distinct, invalid, dropped = {}, {body=0, wheel=0, unknown=0}, 0, 0, 0
end
local function init(data)
  M.instance_id = type(data)=='table' and data.instance_id or 'fixture'
  sequence, total = 0, 0
  began = obj:getSimTime()
  resetWindow()
end
local function vector(value)
  local kind = type(value)
  if kind ~= 'table' and kind ~= 'userdata' and kind ~= 'cdata' then return nil end
  local x, y, z = value.x, value.y, value.z
  if finite(x) and finite(y) and finite(z) then return {x=x, y=y, z=z} end
end
local function nodeCollision(p)
  if type(p) ~= 'table' or not finite(p.id1) or p.id1 % 1 ~= 0 or p.id1 < 0
      or not finite(p.normalForce) or not finite(p.slipForce) or not finite(p.depth) then
    invalid = invalid+1
    return
  end
  local node = v.data.nodes[p.id1]
  local class = 'unknown'
  if node then
    if node.wheelID == nil or (finite(node.wheelID) and node.wheelID < 0) then class='body'
    elseif finite(node.wheelID) and node.wheelID >= 0 then class='wheel' end
  end
  classEvents[class], total = classEvents[class]+1, total+1
  local row = nodes[p.id1]
  if not row then
    if distinct >= MAX_NODES then dropped=dropped+1; return end
    distinct = distinct+1
    row = {node_id=p.id1, class=class, events=0, first_sim_time=obj:getSimTime(),
           max_normal_force=-math.huge, max_abs_slip_force=0, max_depth=-math.huge}
    if node then
      row.node_name = type(node.name)=='string' and string.sub(node.name,1,96) or nil
      row.part_origin = type(node.partOrigin)=='string' and string.sub(node.partOrigin,1,128) or nil
      row.wheel_id = finite(node.wheelID) and node.wheelID or nil
    end
    nodes[p.id1] = row
  end
  row.events = row.events+1
  row.max_abs_slip_force = math.max(row.max_abs_slip_force, math.abs(p.slipForce))
  row.max_depth = math.max(row.max_depth, p.depth)
  if p.normalForce > row.max_normal_force then
    row.max_normal_force = p.normalForce
    row.peak_sim_time = obj:getSimTime()
    row.peak_normal = vector(p.normal)
    row.peak_position = vector(p.pos)
    row.material_id_1 = finite(p.materialID1) and p.materialID1 or nil
    row.material_id_2 = finite(p.materialID2) and p.materialID2 or nil
  end
end
local function snapshot()
  local ended = obj:getSimTime()
  sequence = sequence+1
  local rows = {}
  for _, row in pairs(nodes) do rows[#rows+1] = row end
  table.sort(rows, function(a,b)
    -- Keep body contacts visible even if wheel/ground reactions dominate.
    local ca, cb = a.class=='body' and 0 or a.class=='unknown' and 1 or 2,
                   b.class=='body' and 0 or b.class=='unknown' and 1 or 2
    if ca ~= cb then return ca < cb end
    if a.max_normal_force ~= b.max_normal_force then return a.max_normal_force > b.max_normal_force end
    return a.node_id < b.node_id
  end)
  local samples = {}
  for i=1,math.min(#rows, MAX_OUTPUT) do samples[i] = rows[i] end
  local out = {protocol=M.protocol, instance_id=M.instance_id, window_sequence=sequence, begin_sim_time=began,
    end_sim_time=ended, class_events=classEvents, distinct_nodes=distinct,
    invalid_events=invalid, dropped_distinct_events=dropped, truncated_nodes=math.max(0,#rows-MAX_OUTPUT),
    total_events=total, nodes=samples,
    vector_frame='Raw engine callback vectors; frame requires independent verification',
    collision_object_identity_available=false}
  began = ended
  resetWindow()
  return out
end
M.init, M.nodeCollision, M.snapshot = init, nodeCollision, snapshot
return M
