"""Reset-intent regression using actual callbacks; doubles or actual ROS interfaces."""
from pathlib import Path
import argparse
import importlib
import io
import hashlib
import json
import sys
import unittest

root=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(root/'tools'),str(root/'nav2_adapter'),str(root/'official_adapter'),str(root/'nav2_adapter/test')]
parser=argparse.ArgumentParser();parser.add_argument('--label',choices=['before','after','real_ros'],required=True)
parser.add_argument('--all',action='store_true');parser.add_argument('--real-ros',action='store_true')
args=parser.parse_args()
if args.real_ros:
    import rclpy
    rclpy.init()
else:
    from offline_ros_fakes import install
    install()
try:
    module=importlib.import_module('test_navigation_transactions_ros')
    source_identity={}
    for name in ('navigation_transactions','visible_approach'):
        implementation=importlib.import_module('wildos_nav2_adapter.'+name)
        path=Path(implementation.__file__)
        expected=root/'nav2_adapter/wildos_nav2_adapter'/(name+'.py')
        assert path.resolve()==expected.resolve(),str(path)
        source_identity[name]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    cls=module.NavigationTransactionTests
    names=['test_successful_visible_releases_have_independent_clear_budgets',
        'test_duplicate_clear_intent_preserves_current_request_authority',
        'test_duplicate_clear_intents_cannot_renew_unanswered_response_budget',
        'test_changed_scope_gets_new_budget_but_still_drains_old_requests']
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(cls) if args.all else unittest.TestSuite(cls(name) for name in names)
    stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    output=Path(__file__).resolve().parent
    (output/(args.label+'.log')).write_text(stream.getvalue(),encoding='utf-8')
    report=dict(label=args.label,scope=__doc__,real_ros=args.real_ros,dds_action_server=False,
        tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful(),
        source_identity=source_identity,complete_navigation_passed=False)
    (output/(args.label+'.json')).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2));print(stream.getvalue() if not result.wasSuccessful() else '')
finally:
    if args.real_ros:rclpy.shutdown()
raise SystemExit(not result.wasSuccessful())
