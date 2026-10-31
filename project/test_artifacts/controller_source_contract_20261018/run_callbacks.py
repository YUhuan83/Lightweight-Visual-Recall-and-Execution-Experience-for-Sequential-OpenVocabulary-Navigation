from pathlib import Path
import sys,unittest
root=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(root/'nav2_adapter'),str(root/'nav2_adapter/test')]
from offline_ros_fakes import install
install()
result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromName(
    'test_controller_source_ros.ControllerSourceTests'))
raise SystemExit(not result.wasSuccessful())
