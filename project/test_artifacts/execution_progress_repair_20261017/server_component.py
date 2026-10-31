"""Actual ROS message/callback component check; no action server or BeamNG."""
import hashlib
import json
from pathlib import Path
import sys
import unittest
import rclpy

base = Path('/home/ly/execution_progress_review_20261017/isolation')
manifest = json.loads((base/'component_manifest.json').read_text())
for name, entry in manifest.items():
    assert hashlib.sha256((base/name).read_bytes()).hexdigest() == entry['sha256'], name
sys.path[:0] = [str(base/'nav2_adapter'), str(base/'nav2_adapter/test')]
import wildos_nav2_adapter.graph_path_bridge as bridge
assert Path(bridge.__file__).resolve() == base/'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
rclpy.init()
try:
    suite = unittest.defaultTestLoader.loadTestsFromNames([
        'test_execution_progress_ros.ExecutionProgressTests',
        'test_tracking_handoff_ros.TrackingHandoffTests'])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    print(json.dumps(dict(scope=__doc__, tests=result.testsRun,
        failures=len(result.failures), errors=len(result.errors),
        passed=result.wasSuccessful(), source=bridge.__file__, simulation=False)))
finally:
    rclpy.shutdown()
raise SystemExit(not result.wasSuccessful())
