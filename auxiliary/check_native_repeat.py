import inspect,json,sys,unittest
import test_native_status_executor_ros as module
import wildos_nav2_adapter.graph_path_bridge as bridge
method=module.NativeStatusExecutorTests.odometry_during_navigation_work
def measured(self,direction):
    value=method(self,direction)
    print('NATIVE_MEASUREMENT',json.dumps(dict(direction=direction,**value)),flush=True)
    return value
module.NativeStatusExecutorTests.odometry_during_navigation_work=measured
print('NATIVE_SOURCE',bridge.__file__,flush=True)
result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(module))
raise SystemExit(not result.wasSuccessful())
