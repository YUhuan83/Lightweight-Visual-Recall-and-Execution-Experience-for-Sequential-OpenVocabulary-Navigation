from pathlib import Path
import sys, unittest
root = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(root/'nav2_adapter'), str(root/'nav2_adapter/test'),
               str(root/'official_adapter')]
from offline_ros_fakes import install
install()
suite = unittest.defaultTestLoader.loadTestsFromName(
    'test_graph_path_bridge_routing.ExplorationTargetHandoffTests')
result = unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(0 if result.wasSuccessful() else 1)
