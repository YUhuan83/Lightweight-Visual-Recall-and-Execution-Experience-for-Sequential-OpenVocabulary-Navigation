"""Run lease state tests in the actual VE Lua VM with fake inputs and clock.

Does not install an extension, resume simulation, or issue vehicle inputs.
"""
import argparse
import hashlib
from importlib import resources
import json
from pathlib import Path
import time


CASES = r'''
local source = SOURCE
local passed = {}
local a,b = string.rep('a',32),string.rep('b',32)
local function rig()
  local state={now=1000,speed=0,events={}}
  local env=setmetatable({FILTER_DIRECT=2,FILTER_PAD=1,
    HighPerfTimer=function() return {getTime=function() return state.now end} end,
    input={event=function(k,v,f) state.events[#state.events+1]={k,v,f} end},
    electrics={values={}},
    obj={getVelocity=function() return {length=function() return state.speed end} end}}, {__index=_G})
  local chunk=assert(loadstring(source)); setfenv(chunk,env)
  return chunk(),state,env
end
local function fails(fn) assert(not pcall(fn),'Expected rejection') end
local function test(name,fn) fn(); passed[#passed+1]=name end
test('valid_feed_and_normal_callbacks',function()
  local m,s,e=rig(); assert(m.claim(a,.8)=='wildos_lease_claimed_v1')
  m.feed(a,1); e.electrics.values.brake=0; local n=#s.events
  s.now=1799; m.updateGFX(); assert(not m.status().tripped and #s.events==n)
end)
test('expired_graphics_brakes_and_cannot_rearm',function()
  local m,s,e=rig(); m.claim(a,.8); m.feed(a,1); s.now=1800; m.updateGFX()
  assert(m.status().tripped and e.electrics.values.throttle==0 and e.electrics.values.brake==1)
  assert(e.electrics.values.parkingbrake==1)
  fails(function() m.feed(a,2) end); fails(function() m.claim(a,.8) end)
end)
test('physics_callback_expires_independent_of_graphics',function()
  local m,s,e=rig(); m.claim(a,.8); m.feed(a,1); s.now=1801; m.onPhysicsStep(.02)
  assert(m.status().tripped and e.electrics.values.brake==1)
end)
test('late_request_cannot_rearm_after_callback_gap',function()
  local m,s=rig(); m.claim(a,.8); m.feed(a,1); s.now=2000
  fails(function() m.feed(a,2) end); assert(m.status().last_sequence==1 and m.status().tripped)
end)
test('duplicate_and_out_of_order_do_not_renew',function()
  local m,s=rig(); m.claim(a,.8); m.feed(a,3); s.now=1500
  fails(function() m.feed(a,3) end); fails(function() m.feed(a,2) end)
  assert(m.status().age_sec==.5 and m.status().last_sequence==3)
  s.now=1800; m.updateGFX(); assert(m.status().tripped)
end)
test('active_foreign_owner_has_no_input_mutation',function()
  local m,s=rig(); m.claim(a,.8); m.feed(a,1); local n=#s.events
  fails(function() m.claim(b,.8) end); fails(function() m.feed(b,2) end)
  fails(function() m.release(b) end); assert(#s.events==n and m.status().owner==a)
end)
test('new_owner_requires_expiry_and_real_standstill',function()
  local m,s=rig(); m.claim(a,.8); m.feed(a,1); s.now=1800; s.speed=.2
  fails(function() m.claim(b,.8) end); s.speed=0
  m.claim(b,.8); m.feed(b,1); fails(function() m.feed(a,2) end)
  assert(m.status().owner==b and not m.status().tripped)
end)
test('moving_release_latches_brake_until_new_owner',function()
  local m,s,e=rig(); m.claim(a,.8); m.feed(a,1); s.speed=.5
  assert(m.release(a)=='wildos_lease_brake_held_v1'); assert(m.status().holding)
  fails(function() m.feed(a,2) end)
  s.now=1100; e.electrics.values.brake=0; m.updateGFX(); assert(e.electrics.values.brake==1)
  s.now=1800; s.speed=0; m.claim(b,.8); m.feed(b,1)
end)
test('stationary_release_and_old_message_rejection',function()
  local m=rig(); m.claim(a,.8); m.feed(a,1)
  assert(m.release(a)=='wildos_lease_released_v1' and not m.status().active)
  fails(function() m.feed(a,2) end); m.claim(b,.8); m.feed(b,1)
end)
test('reset_rejects_old_owner',function()
  local m=rig(); m.claim(a,.8); m.feed(a,1); m.onReset()
  assert(m.status().reason=='vehicle_reset'); fails(function() m.feed(a,2) end)
end)
test('clock_regression_brakes',function()
  local m,s,e=rig(); m.claim(a,.8); m.feed(a,1); s.now=999; m.updateGFX()
  assert(m.status().reason=='clock_regression' and e.electrics.values.brake==1)
end)
test('status_read_does_not_cause_expiry_or_input_events',function()
  local m,s,e=rig(); m.claim(a,.8); m.feed(a,1); local n=#s.events
  s.now=2000; assert(not m.status().tripped and #s.events==n)
end)
test('idempotent_claim_does_not_extend_deadline',function()
  local m,s=rig(); m.claim(a,.8); s.now=1500; m.claim(a,.8)
  assert(m.status().age_sec==.5); s.now=1800; m.updateGFX(); assert(m.status().tripped)
end)
test('invalid_configuration_does_not_change_inputs',function()
  local m,s=rig(); fails(function() m.claim(a,0) end)
  fails(function() m.claim(a,3) end); fails(function() m.claim('invalid',.8) end)
  assert(#s.events==0)
end)
return jsonEncode({passed=true,cases=passed,scope='Actual VE Lua VM with isolated fake inputs, clock and speed'})
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--vehicle-port',type=int,required=True)
    args=parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    from beamngpy import BeamNGpy
    from beamngpy.connection import Connection
    from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
    source=resources.files('wildos_nav2_adapter').joinpath('vehicle_control_lease.lua').read_text(encoding='utf8')
    report=dict(passed=False,errors=[],source_sha256=hashlib.sha256(source.encode()).hexdigest(),
        video_recording_enabled=False,map_reload_count=0,vehicle_teleports=0)
    bng=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
    vehicle=None
    try:
        report['before']=read_physics_snapshot(bng)
        assert not physical_snapshot_errors(report['before'],require_sensors=False)
        assert report['before']['speed']<.05
        vehicle=bng.vehicles.get_current()['ego']
        vehicle.connection=Connection('127.0.0.1',args.vehicle_port,timeout=5)
        vehicle.connect(bng)
        result=vehicle.queue_lua_command(CASES.replace('SOURCE',json.dumps(source)),response=True)
        report['lua']=json.loads(result)
        report['after']=read_physics_snapshot(bng)
        for field in ('vehicleID','position','nodeCount','fixedNodeCount','bodyCollisionNodeCount','wheels'):
            assert report['before'][field]==report['after'][field], field+' changed'
        assert report['lua']['passed'] and len(report['lua']['cases'])==14
        report['passed']=True
    except Exception as exc:
        report['errors'].append(repr(exc))
    finally:
        if vehicle: vehicle.disconnect()
        bng.disconnect()
        args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps(report))
    return 0 if report['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
