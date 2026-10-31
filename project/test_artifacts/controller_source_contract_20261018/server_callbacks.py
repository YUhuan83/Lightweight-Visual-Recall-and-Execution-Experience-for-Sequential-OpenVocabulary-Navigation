from pathlib import Path
import json,hashlib,sys,unittest,rclpy
root=Path("/home/ly/controller_source_contract_20261018")
manifest=json.loads((root/'candidate.manifest.json').read_text())
for name,want in manifest['members'].items():
    assert hashlib.sha256((root/'candidate'/name).read_bytes()).hexdigest()==want,name
sys.path[:0]=[str(root/'candidate/nav2_adapter/test'),str(root/'candidate/tools')]
import wildos_nav2_adapter.cmd_vel_to_control as implementation
assert str(Path(implementation.__file__).resolve()).startswith(str(root/'install'))
assert hashlib.sha256(Path(implementation.__file__).read_bytes()).hexdigest()==manifest['members']['nav2_adapter/wildos_nav2_adapter/cmd_vel_to_control.py']
rclpy.init()
try:
    suite=unittest.defaultTestLoader.loadTestsFromNames([
        'test_controller_source_ros.ControllerSourceTests',
        'test_source_control_ros.SourceControlCallbackTests',
        'test_execution_transport_health_ros.ExecutionTransportHealthTests',
        'test_tracking_handoff_ros.TrackingHandoffTests',
        'test_motion_intent_handoff_ros.MotionIntentHandoffTests',
        'test_runtime_contract.RuntimeContractTests',
        'test_source_control_preflight.SourceEndpointTests'])
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    print(json.dumps(dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
        passed=result.wasSuccessful(),source=implementation.__file__,simulation=False)))
finally:rclpy.shutdown()
raise SystemExit(not result.wasSuccessful())
