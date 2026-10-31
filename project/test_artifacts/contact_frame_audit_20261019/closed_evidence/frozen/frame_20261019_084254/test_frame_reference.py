"""Actual Lua execution with explicit independent-coordinate API fixtures."""
import io
import json
from pathlib import Path
import unittest
import test_contact_observer as original

APIS="""
local origin={x=100,y=200,z=50}
local relative={x=1,y=2,z=3}
local direction={x=.4,y=-.9,z=.03}
local up={x=0,y=0,z=1}
obj.getPosition=function() return origin end
obj.getNodePosition=function(self,id) assert(id==1);return relative end
obj.getDirectionVector=function() return direction end
obj.getDirectionVectorUp=function() return up end
"""

class FrameReferenceTests(unittest.TestCase):
    def test_node_world_is_independent_origin_plus_relative(self):
        original.execute(APIS+"local p="+original.EVENT+";p.pos={x=101,y=202,z=53};observer.nodeCollision(p);local n=observer.snapshot().nodes[1];assert(n.peak_reference.node_world.x==101 and n.peak_reference.node_world.y==202 and n.peak_reference.node_world.z==53);assert(n.peak_position.x==101 and n.peak_reference_error==nil)")

    def test_engine_vectors_are_copied_at_peak_not_read_later(self):
        original.execute(APIS+"local p="+original.EVENT+";observer.nodeCollision(p);origin.x=900;relative.x=800;direction.x=9;up.z=8;local s=observer.snapshot();local r=s.nodes[1].peak_reference;assert(r.vehicle_origin.x==100 and r.node_relative.x==1 and r.node_world.x==101 and r.direction.x==.4 and r.up.z==1);assert(s.reference.vehicle_origin.x==900)")

    def test_reference_api_failure_preserves_collision_evidence(self):
        original.execute("local p="+original.EVENT+";observer.nodeCollision(p);local s=observer.snapshot();assert(s.class_events.body==1 and s.nodes[1].max_normal_force==200 and s.nodes[1].peak_reference==nil and type(s.nodes[1].peak_reference_error)=='string' and s.invalid_events==0)")

    def test_invalid_engine_vector_is_not_reported_as_reference(self):
        original.execute(APIS+"origin.x=0/0;local p="+original.EVENT+";observer.nodeCollision(p);local s=observer.snapshot();assert(s.reference==nil and type(s.reference_error)=='string' and s.nodes[1].peak_reference==nil and s.nodes[1].events==1)")

    def test_peak_and_snapshot_keep_distinct_exact_sim_times(self):
        original.execute(APIS+"local p="+original.EVENT+";simTime=12.345;observer.nodeCollision(p);simTime=13.5;local s=observer.snapshot();assert(s.nodes[1].peak_reference.engine_sim_time==12.345 and s.nodes[1].peak_sim_time==12.345 and s.reference.engine_sim_time==13.5 and s.end_sim_time==13.5)")

    def test_callback_normal_is_saved_without_assumed_transform(self):
        original.execute(APIS+"local p="+original.EVENT+";p.normal={x=0,y=.96,z=.28};observer.nodeCollision(p);local n=observer.snapshot().nodes[1];assert(n.peak_normal.y==.96 and n.peak_normal.z==.28);assert(n.peak_reference.direction.x==.4)")

    def test_node_api_failure_does_not_break_snapshot_reference(self):
        original.execute(APIS+"obj.getNodePosition=function() error('node read fixture') end;local p="+original.EVENT+";observer.nodeCollision(p);local s=observer.snapshot();assert(s.reference and s.nodes[1].peak_reference==nil and type(s.nodes[1].peak_reference_error)=='string' and s.class_events.body==1)")

if __name__=='__main__':
    suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(original.ContactObserverTests),unittest.defaultTestLoader.loadTestsFromTestCase(FrameReferenceTests)])
    stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    root=Path(__file__).resolve().parent
    (root/'frame_reference_tests.log').write_text(stream.getvalue())
    report=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful(),scope=__doc__,beamng_verified=False)
    (root/'frame_reference_tests.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));raise SystemExit(not result.wasSuccessful())
