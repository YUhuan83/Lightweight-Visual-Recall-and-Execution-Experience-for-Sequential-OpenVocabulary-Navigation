"""Characterize existing defects. PASS means reproduced, never repaired."""
from pathlib import Path
import ast,json,math,sys,time,types,unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'nav2_adapter'),str(ROOT/'nav2_adapter/test'),str(ROOT/'official_adapter')]
from offline_ros_fakes import install
install()
from beamng_msgs.msg import VehicleControl
from test_native_vehicle_control_ros import NativeControlIoTests
from test_graph_path_bridge_routing import GraphBridgeRoutingTests
from wildos_nav2_adapter.tracking_preview import preview_tracking

class KnownContractDefects(unittest.TestCase):
    def native(self):
        fixture=NativeControlIoTests();fixture.setUp();node=fixture.node
        source=ROOT/'nav2_adapter/wildos_nav2_adapter/native_vehicle_control.py'
        cls=next(n for n in ast.parse(source.read_text(encoding='utf8')).body if isinstance(n,ast.ClassDef))
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='on_control')
        ns=dict(math=math,time=time)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),str(source),'exec'),ns)
        node.on_control=types.MethodType(ns['on_control'],node)
        return node

    def test_old_payload_is_made_fresh_at_delayed_callback_entry(self):
        n=self.native();message=VehicleControl(throttle=.25,brake=0.,gear=2)
        source_issued_at=100.
        with patch('time.monotonic',return_value=100.7):n.on_control(message)
        self.assertEqual(n.command_at,100.7)  # Source issue time is absent from the interface.
        with patch('time.monotonic',return_value=100.75):
            n.poll_tick();n.control_tick()
        actual=n.actuator.control.call_args.kwargs
        self.assertGreater(actual['throttle'],0.)
        status=json.loads(n.telemetry.publish.call_args.args[0].data)
        self.assertFalse(status['watchdog_stop'])
        self.assertLess(status['command_age'],100.75-source_issued_at)

    def test_command_hold_near_cusp_is_allowed_for_point_seven_seconds(self):
        n=self.native()
        with patch('time.monotonic',return_value=100.7):
            n.poll_tick();n.control_tick()
        self.assertEqual(n.actuator.control.call_args.kwargs['throttle'],.25)
        status=json.loads(n.telemetry.publish.call_args.args[0].data)
        self.assertFalse(status['watchdog_stop'])
        self.assertGreater(1.2*status['command_age'],2*.2)

    def test_bridge_health_ack_does_not_establish_demand_freshness(self):
        fixture=GraphBridgeRoutingTests();fixture.setUp();n=fixture.node
        try:
            n.follow_enabled=True
            n.native_status.update(command_age=.44,watchdog_stop=False,
                                   sent=dict(throttle=.1,brake=0.))
            self.assertIsNone(n.execution_transport_issue())
        finally:n.destroy_node()

    def test_geometric_preview_admits_all_five_recorded_dynamic_failures(self):
        cases=json.loads((ROOT/'test_artifacts/cusp_execution_repair_20261016/recorded_cases.json').read_text(encoding='utf8'))
        for c in cases:
            with self.subTest(elapsed=c['failure']['elapsed']):
                path=c['path'];start=c['plan']['actual_start_pose']
                points=[(p['pose']['position']['x'],p['pose']['position']['y']) for p in path['poses']]
                headings=[]
                for p in path['poses']:
                    q=p['pose']['orientation']
                    headings.append(math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2)))
                result=preview_tracking(points,headings,start[:2],start[2],8.,position_tolerance=.2)
                self.assertTrue(result.reached)

if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(KnownContractDefects))
    raise SystemExit(0 if result.wasSuccessful() else 1)
