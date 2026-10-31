import sys,unittest
sys.path[:0]=['tools','nav2_adapter','nav2_adapter/test','official_adapter']
from offline_ros_fakes import install
install()
from test_motion_intent_handoff_ros import StopPlanPreparationTests
result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(StopPlanPreparationTests))
raise SystemExit(not result.wasSuccessful())
