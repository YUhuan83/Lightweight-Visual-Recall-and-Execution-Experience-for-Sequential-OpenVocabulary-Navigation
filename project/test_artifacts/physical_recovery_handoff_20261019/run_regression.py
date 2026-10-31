"""Physical evidence and visual-intent handoff callbacks; not vehicle acceptance."""
from pathlib import Path
import argparse
import hashlib
import importlib
import io
import json
import sys
import unittest

root=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(root/'tools'),str(root/'nav2_adapter'),str(root/'official_adapter'),str(root/'nav2_adapter/test')]
p=argparse.ArgumentParser();p.add_argument('--label',required=True)
p.add_argument('--real-ros',action='store_true');p.add_argument('--all',action='store_true');args=p.parse_args()
if args.real_ros:
    import rclpy
    rclpy.init()
else:
    from offline_ros_fakes import install
    install()
try:
    mod=importlib.import_module('test_navigation_transactions_ros')
    cls=mod.NavigationTransactionTests
    names=[name for name in unittest.defaultTestLoader.getTestCaseNames(cls)
           if name.startswith('test_visual_') or name=='test_query_change_still_revokes_physical_recovery_after_visual_timeout']
    assert len(names)==11,names
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(cls) if args.all else unittest.TestSuite(cls(name) for name in names)
    identity={}
    for name in ('visible_approach','execution_recovery','navigation_transactions'):
        path=Path(importlib.import_module('wildos_nav2_adapter.'+name).__file__).resolve()
        assert path==root/'nav2_adapter/wildos_nav2_adapter'/(name+'.py')
        identity[name]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    stream=io.StringIO();r=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    out=Path(__file__).resolve().parent
    (out/(args.label+'.log')).write_text(stream.getvalue(),encoding='utf8')
    report=dict(tests=r.testsRun,failures=len(r.failures),errors=len(r.errors),passed=r.wasSuccessful(),
        source_identity=identity,real_ros=args.real_ros,dds_action_server=False,complete_navigation_passed=False)
    (out/(args.label+'.json')).write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print(json.dumps(report,indent=2))
    if not r.wasSuccessful():print(stream.getvalue())
finally:
    if args.real_ros:rclpy.shutdown()
raise SystemExit(not r.wasSuccessful())
