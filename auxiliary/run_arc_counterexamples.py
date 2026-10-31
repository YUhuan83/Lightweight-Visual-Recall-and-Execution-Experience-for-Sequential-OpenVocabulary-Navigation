import sys,unittest
from pathlib import Path
repo=Path(r'C:\Users\Yu\Desktop\拟真')
sys.path[:0]=[str(repo/'nav2_adapter/test'),str(repo/'nav2_adapter'),str(repo/'official_adapter')]
from offline_ros_fakes import install
install()
from test_graph_path_bridge_routing import GraphBridgeRoutingTests
suite=unittest.TestSuite(GraphBridgeRoutingTests(n) for n in (
 'test_non_directional_shunt_probes_curvature_compatible_arrival',
 'test_curvature_probe_failure_retains_original_feasible_path'))
result=unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
