"""Execute diagnostic Lua against interface fixtures, not the BeamNG physics engine."""
import ctypes
import ctypes.util
import io
import json
from pathlib import Path
import unittest
import contact_chunks

ROOT = Path(__file__).resolve().parent
SOURCE = (ROOT/'contact_observer.lua').read_text()


def execute(fragment):
    name = ctypes.util.find_library('lua5.3')
    if not name: raise RuntimeError('Lua 5.3 shared library unavailable')
    lua = ctypes.CDLL(name)
    lua.luaL_newstate.restype = ctypes.c_void_p
    lua.luaL_openlibs.argtypes = [ctypes.c_void_p]
    lua.luaL_loadstring.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lua.luaL_loadstring.restype = ctypes.c_int
    lua.lua_pcallk.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_longlong, ctypes.c_void_p]
    lua.lua_pcallk.restype = ctypes.c_int
    lua.lua_tolstring.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_size_t)]
    lua.lua_tolstring.restype = ctypes.c_void_p
    lua.lua_close.argtypes = [ctypes.c_void_p]
    state = lua.luaL_newstate(); lua.luaL_openlibs(state)
    prefix = """local simTime=10
obj={getSimTime=function() return simTime end}
v={data={nodes={[1]={name='bumper',partOrigin='bumper',wheelID=-1},
 [2]={name='tire',wheelID=0},[3]={name='body'}}}}
local observer=(function()
"""
    script = prefix+SOURCE+"\nend)()\nobserver.init()\n"+fragment
    try:
        status = lua.luaL_loadstring(state, script.encode())
        if status == 0: status = lua.lua_pcallk(state, 0, 0, 0, 0, None)
        if status:
            size = ctypes.c_size_t()
            pointer = lua.lua_tolstring(state, -1, ctypes.byref(size))
            raise AssertionError(ctypes.string_at(pointer, size.value).decode())
    finally: lua.lua_close(state)


EVENT = "{id1=1,normalForce=200,slipForce=3,depth=.01,normal={x=0,y=0,z=1},pos={x=1,y=2,z=3},materialID1=10,materialID2=4}"


class ContactObserverTests(unittest.TestCase):
    def test_body_wheel_unknown_and_zero_wheel_id(self):
        execute("local p="+EVENT+"; observer.nodeCollision(p); p.id1=2;observer.nodeCollision(p);p.id1=99;observer.nodeCollision(p);local s=observer.snapshot();assert(s.class_events.body==1 and s.class_events.wheel==1 and s.class_events.unknown==1);assert(s.nodes[1].class=='body');assert(s.collision_object_identity_available==false)")

    def test_engine_reused_event_and_vectors_are_copied(self):
        execute("local p="+EVENT+";observer.nodeCollision(p);p.normal.x=8;p.pos.x=9;p.normalForce=1;observer.nodeCollision(p);local s=observer.snapshot();assert(s.nodes[1].peak_normal.x==0 and s.nodes[1].peak_position.x==1);assert(s.nodes[1].max_normal_force==200 and s.nodes[1].events==2)")

    def test_invalid_event_does_not_poison_finite_snapshot(self):
        execute("observer.nodeCollision(nil);local p="+EVENT+";p.normalForce=0/0;observer.nodeCollision(p);p.normalForce=10;p.id1=-1;observer.nodeCollision(p);local s=observer.snapshot();assert(s.invalid_events==3 and s.total_events==0 and #s.nodes==0)")

    def test_snapshot_sequences_reset_window_but_preserve_cumulative_count(self):
        execute("local p="+EVENT+";observer.nodeCollision(p);simTime=11;local a=observer.snapshot();simTime=12;local b=observer.snapshot();assert(a.window_sequence==1 and b.window_sequence==2);assert(a.begin_sim_time==10 and a.end_sim_time==11 and b.begin_sim_time==11);assert(a.class_events.body==1 and b.class_events.body==0 and b.total_events==1);assert(#a.nodes==1 and #b.nodes==0)")

    def test_node_storage_and_output_are_bounded(self):
        execute("local p="+EVENT+";for i=1,1500 do p.id1=i;observer.nodeCollision(p) end;local s=observer.snapshot();assert(s.distinct_nodes==1024 and s.dropped_distinct_events==476 and #s.nodes==128);assert(s.total_events==1500 and s.truncated_nodes==896)")

    def test_body_prioritized_even_when_wheel_forces_are_larger(self):
        execute("local p="+EVENT+";observer.nodeCollision(p);p.id1=2;p.normalForce=9000;observer.nodeCollision(p);local s=observer.snapshot();assert(s.nodes[1].node_id==1 and s.nodes[2].node_id==2)")

    def test_missing_optional_vectors_do_not_block_force_observation(self):
        execute("local p="+EVENT+";p.normal=nil;p.pos=nil;p.materialID1=nil;observer.nodeCollision(p);local s=observer.snapshot();assert(s.class_events.body==1 and s.nodes[1].max_normal_force==200 and s.nodes[1].peak_normal==nil)")

    def test_owned_registration_snapshot_and_unload_preserve_other_controllers(self):
        execute(CONTROLLER+"local other={};controller.loaded.other=other;local result=(function() "+contact_chunks.install('one')+" end)();assert(result.installed);local s=(function() "+contact_chunks.snapshot('one')+" end)();assert(s.instance_id=='one');local u=(function() "+contact_chunks.uninstall('one')+" end)();assert(u.unloaded and controller.loaded.other==other)")

    def test_rejected_install_does_not_delete_unowned_preload(self):
        execute(CONTROLLER+"local key='controller/"+contact_chunks.NAME+"';local foreign=function() return {} end;package.preload[key]=foreign;local ok=pcall(function() "+contact_chunks.install('one')+" end);assert(not ok);local u=(function() "+contact_chunks.uninstall('one')+" end)();assert(u.unloaded==false and package.preload[key]==foreign,'Unowned preload removed')")

    def test_other_instance_cannot_unload_our_controller(self):
        execute(CONTROLLER+"local result=(function() "+contact_chunks.install('one')+" end)();local ok=pcall(function() "+contact_chunks.uninstall('two')+" end);assert(not ok and controller.getController('"+contact_chunks.NAME+"').instance_id=='one')")

    def test_partial_load_can_clean_only_its_own_preload(self):
        execute(CONTROLLER+"controller.loadControllerExternal=function() error('fixture partial load') end;local ok=pcall(function() "+contact_chunks.install('one')+" end);assert(not ok);local u=(function() "+contact_chunks.uninstall('one')+" end)();assert(u.unloaded and package.preload['controller/"+contact_chunks.NAME+"']==nil)")


CONTROLLER = """
controller={loaded={}}
function controller.getController(name) return controller.loaded[name] end
function controller.loadControllerExternal(file,name,data)
  local c=package.preload['controller/'..file]();c.init(data);controller.loaded[name]=c;return c
end
function controller.unloadControllerExternal(name) controller.loaded[name]=nil end
function jsonEncode(value) return value end
"""


if __name__ == '__main__':
    stream = io.StringIO()
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ContactObserverTests))
    (ROOT/'lua_fixture_tests.log').write_text(stream.getvalue())
    report = dict(tests=result.testsRun, failures=len(result.failures), errors=len(result.errors),
                  passed=result.wasSuccessful(), runtime='liblua5.3 with interface fixtures',
                  beamng_verified=False, source_changes='Diagnostic only')
    (ROOT/'lua_fixture_tests.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(not result.wasSuccessful())
