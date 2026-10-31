"""Run approach recovery callbacks with interface doubles or real ROS types."""
from pathlib import Path
import argparse
import json
import sys
import unittest

root = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(root/'nav2_adapter'), str(root/'nav2_adapter/test'),
               str(root/'official_adapter'), str(root/'tools'),
               str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
p = argparse.ArgumentParser()
p.add_argument('--real-ros', action='store_true')
p.add_argument('--label', required=True)
p.add_argument('--regression', action='store_true')
args = p.parse_args()
if args.real_ros:
    import rclpy
    rclpy.init()
else:
    from offline_ros_fakes import install
    install()
names = ['test_confirmed_recovery_ros']
if args.regression:
    names += ['test_sensor_observation_lifecycle_ros',
              'test_navigation_transactions_ros', 'test_execution_progress_ros',
              'test_visible_approach_ros']
    if not args.real_ros:
        # These legacy suites explicitly construct SimpleNamespace message
        # substitutes. They remain in the offline regression, not ROS ABI tests.
        names += ['test_execution_mechanism_ros', 'test_surface_association_ros']
try:
    result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(
        unittest.defaultTestLoader.loadTestsFromName(n) for n in names))
    report = dict(tests=result.testsRun, failures=len(result.failures), errors=len(result.errors),
                  passed=result.wasSuccessful(), real_ros=args.real_ros,
                  dds_chain=False, complete_navigation_passed=False)
    (Path(__file__).resolve().parent/(args.label+'.json')).write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))
finally:
    if args.real_ros:
        rclpy.shutdown()
raise SystemExit(not result.wasSuccessful())
