import sys,unittest
from pathlib import Path
repo=Path(r'C:\Users\Yu\Desktop\拟真');sys.path[:0]=[str(repo/'nav2_adapter/test'),str(repo/'nav2_adapter'),str(repo/'official_adapter')]
from offline_ros_fakes import install
install()
from test_cmd_vel_to_control_ros import SignedGroundFeedbackTests
suite=unittest.TestSuite(SignedGroundFeedbackTests(n) for n in (
'test_physical_profile_stable_reverse_does_not_wait_two_seconds',
'test_physical_profile_stable_forward_does_not_wait_two_seconds'))
result=unittest.TextTestRunner(verbosity=2).run(suite);raise SystemExit(not result.wasSuccessful())
