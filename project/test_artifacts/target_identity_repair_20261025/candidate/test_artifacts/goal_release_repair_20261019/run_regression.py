"""Goal-release/search-reset transactions; callbacks, not vehicle acceptance."""
from pathlib import Path
import argparse,hashlib,importlib,io,json,sys,unittest
root=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(root/'tools'),str(root/'nav2_adapter'),str(root/'official_adapter'),str(root/'nav2_adapter/test')]
p=argparse.ArgumentParser();p.add_argument('--label',required=True)
p.add_argument('--all',action='store_true');p.add_argument('--real-ros',action='store_true');a=p.parse_args()
if a.real_ros:
    import rclpy
    rclpy.init()
else:
    from offline_ros_fakes import install
    install()
try:
    cls=importlib.import_module('test_navigation_transactions_ros').NavigationTransactionTests
    names=[n for n in unittest.defaultTestLoader.getTestCaseNames(cls) if n.startswith('test_goal_reset_')]
    assert len(names)==8,names
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(cls) if a.all else unittest.TestSuite(cls(n) for n in names)
    identity={}
    for name in ('navigation_transactions','graph_path_bridge','execution_recovery','visible_approach'):
        path=Path(importlib.import_module('wildos_nav2_adapter.'+name).__file__).resolve()
        assert path==root/'nav2_adapter/wildos_nav2_adapter'/(name+'.py')
        identity[name]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    stream=io.StringIO();r=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    out=Path(__file__).resolve().parent
    (out/(a.label+'.log')).write_text(stream.getvalue(),encoding='utf8')
    report=dict(tests=r.testsRun,failures=len(r.failures),errors=len(r.errors),passed=r.wasSuccessful(),
        real_ros=a.real_ros,dds_action_server=False,complete_navigation_passed=False,source_identity=identity)
    (out/(a.label+'.json')).write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print(json.dumps(report,indent=2))
    if not r.wasSuccessful():print(stream.getvalue())
finally:
    if a.real_ros:rclpy.shutdown()
raise SystemExit(not r.wasSuccessful())
