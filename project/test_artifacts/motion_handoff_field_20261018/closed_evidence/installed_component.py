"""Actual ROS message/callback component check; no action server or BeamNG."""
import hashlib
import json
from pathlib import Path
import sys
import unittest
import rclpy

base = Path('/home/ly/motion_handoff_field_20261018/candidate')
manifest = json.loads(Path('/home/ly/motion_handoff_field_20261018/manifest.json').read_text())['members']
for name, sha in manifest.items():
    assert hashlib.sha256((base/name).read_bytes()).hexdigest() == sha, name
sys.path.insert(0,str(base/'nav2_adapter/test'))
import wildos_nav2_adapter.graph_path_bridge as bridge
assert str(Path(bridge.__file__).resolve()).startswith('/home/ly/motion_handoff_field_20261018/install/'), bridge.__file__
assert hashlib.sha256(Path(bridge.__file__).read_bytes()).hexdigest()==manifest['nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py']
rclpy.init()
try:
    suite = unittest.defaultTestLoader.loadTestsFromNames([
        'test_execution_progress_ros.ExecutionProgressTests',
        'test_tracking_handoff_ros.TrackingHandoffTests',
        'test_motion_intent_handoff_ros.MotionIntentHandoffTests'])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    print(json.dumps(dict(scope=__doc__, tests=result.testsRun,
        failures=len(result.failures), errors=len(result.errors),
        passed=result.wasSuccessful(), source=bridge.__file__, simulation=False)))
finally:
    rclpy.shutdown()
raise SystemExit(not result.wasSuccessful())
