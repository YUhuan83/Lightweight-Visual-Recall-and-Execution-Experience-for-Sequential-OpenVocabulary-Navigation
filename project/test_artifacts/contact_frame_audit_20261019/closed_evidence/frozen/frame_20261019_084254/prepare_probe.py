"""Clone the closed section diagnostic; add independent coordinate/clock reads only."""
from pathlib import Path
import hashlib
import json

root=Path(__file__).resolve().parent
old=root.parent/'contact_observer_20261018'
origins={}
def write(name,source):
    (root/name).write_text(source,encoding='utf-8',newline='\n')
def load(name):
    path=old/name
    origins[name]=hashlib.sha256(path.read_bytes()).hexdigest()
    return path.read_text(encoding='utf-8')
def replace(source,old,new):
    assert source.count(old)==1,(old,source.count(old))
    return source.replace(old,new)

for name in ('execute_recorded.py','scene_recorded.py','recorded_pose.py',
             'recorded_section_fixture.json','test_contact_observer.py','test_recorded_ros_inputs.py'):
    write(name,load(name))
chunks=load('contact_chunks.py').replace('wildosContactAudit20261018','wildosContactFrameAudit20261019')
write('contact_chunks.py',chunks)
observer=load('contact_observer.lua')
marker='local function nodeCollision(p)'
reference="""-- Independent API readings copied at the same physics callback/snapshot.
-- No assumed transform of the callback vectors is applied here.
local function readReference(nodeId)
  local ok, result = pcall(function()
    local origin = assert(vector(obj:getPosition()), 'Invalid engine origin')
    local direction = assert(vector(obj:getDirectionVector()), 'Invalid engine direction')
    local up = assert(vector(obj:getDirectionVectorUp()), 'Invalid engine up')
    local ref = {engine_sim_time=obj:getSimTime(), vehicle_origin=origin,
                 direction=direction, up=up}
    if nodeId ~= nil then
      local relative = assert(vector(obj:getNodePosition(nodeId)), 'Invalid node position')
      ref.node_relative = relative
      ref.node_world = {x=origin.x+relative.x, y=origin.y+relative.y, z=origin.z+relative.z}
    end
    return ref
  end)
  if ok then return result, nil end
  return nil, string.sub(tostring(result),1,160)
end
"""
observer=replace(observer,marker,reference+marker)
observer=replace(observer,'    row.peak_sim_time = obj:getSimTime()',
    '    row.peak_sim_time = obj:getSimTime()\n    row.peak_reference, row.peak_reference_error = readReference(p.id1)')
observer=replace(observer,'  local out = {protocol=M.protocol',
    '  local reference, referenceError = readReference(nil)\n  local out = {reference=reference, reference_error=referenceError, protocol=M.protocol')
write('contact_observer.lua',observer)
driver=load('run_recorded_component.py')
driver=replace(driver,"ROOT = Path('/home/ly/recorded_section_replay_20261018')", "ROOT = Path('/home/ly/contact_frame_audit_20261019')")
driver=replace(driver,"OUT = ROOT/('recorded_'", "OUT = ROOT/('frame_'")
driver=replace(driver,"'/home/ly/planning_phase_physical_20261018/candidate/tools/attach_current_vehicle_sensors.py'", "str(ROOT/'attach_current_vehicle_sensors_observed.py')")
write('run_frame_component.py',driver)
launcher=load('run_recorded_component.sh').replace('/home/ly/recorded_section_replay_20261018','/home/ly/contact_frame_audit_20261019').replace('run_recorded_component.py','run_frame_component.py')
write('run_frame_component.sh',launcher)
write('run_inputs.sh',load('run_recorded_inputs.sh').replace('/home/ly/recorded_section_replay_20261018','/home/ly/contact_frame_audit_20261019'))
sensor=root.parents[1]/'tools/attach_current_vehicle_sensors.py'
origins['attach_current_vehicle_sensors.py']=hashlib.sha256(sensor.read_bytes()).hexdigest()
sensor_source=sensor.read_text(encoding='utf-8')
sensor_source=replace(sensor_source,"        node = bridge._vehicles[args.vehicle]", """        node = bridge._vehicles[args.vehicle]
        # Read the bridge's already established clock anchor; do not reset it.
        report['bridge_start_time_ros_ns'] = int(node.start_time.nanoseconds)
        report['bridge_delay_offset_sec'] = float(node.DELAY_OFFSET_SEC)
        report['clock_anchor_observation'] = 'Read existing vehicle node after initialization; no clock change'
""")
write('attach_current_vehicle_sensors_observed.py',sensor_source)
(root/'preparation.json').write_text(json.dumps(dict(source_sha256=origins,
    output_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in root.iterdir() if p.suffix in ('.py','.sh','.lua','.json') and p.name!='preparation.json'},
    production_changes=False,physics_changes=False),indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(prepared=str(root),outputs=len(origins),production_changes=False)))
