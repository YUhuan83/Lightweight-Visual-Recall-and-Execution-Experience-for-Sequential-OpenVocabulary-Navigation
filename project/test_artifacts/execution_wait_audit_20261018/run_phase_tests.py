from pathlib import Path
import sys,unittest
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'nav2_adapter'),str(ROOT/'nav2_adapter/test')]
from offline_ros_fakes import install
install()
result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromName('test_planning_phase_ros.PlanningPhaseTests'))
raise SystemExit(not result.wasSuccessful())
