import sys,unittest
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\continuous_exploration_20261024\candidate')
sys.path[:0]=[str(root/'nav2_adapter/test'),str(root/'nav2_adapter'),str(root/'official_adapter')]
from offline_ros_fakes import install
install()
import test_continuous_exploration_ros
suite=unittest.defaultTestLoader.loadTestsFromModule(test_continuous_exploration_ros)
result=unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
