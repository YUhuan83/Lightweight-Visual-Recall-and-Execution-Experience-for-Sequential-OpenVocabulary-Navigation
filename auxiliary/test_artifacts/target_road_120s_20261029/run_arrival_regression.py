"""Initialize real ROS before using existing target-chain test fixtures."""
import importlib.util
import json
from pathlib import Path
import sys
import unittest
import rclpy

before='--before' in sys.argv
if before:
    name='wildos_nav2_adapter.graph_path_bridge'
    spec=importlib.util.spec_from_file_location(name,Path.cwd()/'before_3d/graph_path_bridge.py')
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
rclpy.init()
try:
    suite=unittest.defaultTestLoader.loadTestsFromName('test_upstream_target_approach_ros')
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    report=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful())
    Path('before_3d_business_report.json' if before else 'after_3d_business_report.json').write_text(json.dumps(report,indent=2))
finally:
    if rclpy.ok():rclpy.shutdown()
raise SystemExit(not result.wasSuccessful())
