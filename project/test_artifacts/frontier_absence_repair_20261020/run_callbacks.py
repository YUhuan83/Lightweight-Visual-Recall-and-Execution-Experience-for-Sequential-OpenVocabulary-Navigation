"""Real messages or offline doubles for frontier retirement callback regressions, not field acceptance."""
from pathlib import Path
import argparse
import importlib
import json
import sys
import unittest

root=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(root/'nav2_adapter'),str(root/'nav2_adapter/test'),str(root/'official_adapter'),str(root/'tools')]
parser=argparse.ArgumentParser();parser.add_argument('--real-ros',action='store_true')
parser.add_argument('--label',required=True);args=parser.parse_args()
if args.real_ros:
    import rclpy
    rclpy.init()
else:
    from offline_ros_fakes import install
    install()
try:
    # This runner owns the single ROS context. These fixtures are also used
    # by standalone unittest runs where the class normally owns that context.
    if args.real_ros:
        fixture=importlib.import_module('test_graph_path_bridge_routing').ExplorationTargetHandoffTests
        fixture.setUpClass=classmethod(lambda cls: None)
        fixture.tearDownClass=classmethod(lambda cls: None)
    suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in (
        'test_motion_intent_handoff_ros','test_runtime_contract',
        'test_graph_path_bridge_routing.ExplorationTargetHandoffTests'))
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    report=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
                passed=result.wasSuccessful(),real_ros=args.real_ros,dds_chain=False,
                complete_navigation_passed=False)
    (Path(__file__).resolve().parent/(args.label+'.json')).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))
finally:
    if args.real_ros:rclpy.shutdown()
raise SystemExit(not result.wasSuccessful())
