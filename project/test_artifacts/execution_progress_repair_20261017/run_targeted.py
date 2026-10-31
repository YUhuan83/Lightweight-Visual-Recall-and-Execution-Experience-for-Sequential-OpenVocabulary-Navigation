"""Isolated ROS doubles; does not contact a ROS graph or vehicle."""
import sys, unittest
from pathlib import Path
root = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(root/'nav2_adapter'), str(root/'nav2_adapter/test'),
    str(root/'official_adapter'),
    str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
suite = unittest.defaultTestLoader.loadTestsFromNames(sys.argv[1:] or ['test_execution_progress_ros'])
result = unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
