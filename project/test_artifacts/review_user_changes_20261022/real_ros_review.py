"""Use real ROS message classes and isolated node lifetimes, without BeamNG."""
import importlib
import inspect
import json
import unittest
from pathlib import Path
import rclpy

modules = ('test_execution_continuity_review_ros', 'test_native_status_executor_ros',
           'test_graph_path_bridge_routing',
           'test_motion_intent_handoff_ros', 'test_committed_frontier_path_ros',
           'test_exploration_cost_ros', 'test_reachable_observation_ros',
           'test_planning_phase_ros', 'test_navigation_transactions_ros',
           'test_visible_approach_ros', 'test_goal_confirmation_ros')
report = []
for name in modules:
    module = importlib.import_module(name)
    for _, cls in inspect.getmembers(module, inspect.isclass):
        if not issubclass(cls, unittest.TestCase) or cls.__module__ != name:
            continue
        if cls.setUpClass.__func__ is unittest.TestCase.setUpClass.__func__:
            rclpy.init()
        result = unittest.TextTestRunner(verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromTestCase(cls))
        report.append(dict(suite=name, cls=cls.__name__, tests=result.testsRun,
                           failures=len(result.failures), errors=len(result.errors),
                           passed=result.wasSuccessful()))
        if rclpy.ok():
            rclpy.shutdown()
Path('real_ros_report.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
raise SystemExit(0 if all(item['passed'] for item in report) else 1)
