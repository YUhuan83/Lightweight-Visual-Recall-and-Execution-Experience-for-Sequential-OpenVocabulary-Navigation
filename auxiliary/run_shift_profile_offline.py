import sys,unittest
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\continuous_exploration_20261024/candidate')
sys.path[:0]=[str(root/'tools'),str(root/'nav2_adapter'),str(root/'nav2_adapter/test')]
from offline_ros_fakes import install
install()
import test_physical_shift_profile_ros
result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(test_physical_shift_profile_ros))
raise SystemExit(0 if result.wasSuccessful() else 1)
