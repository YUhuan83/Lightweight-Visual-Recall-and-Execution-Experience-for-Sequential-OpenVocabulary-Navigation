import hashlib,json,sys,unittest
from pathlib import Path
import rclpy
import wildos_nav2_adapter.graph_path_bridge as production
r=Path('/home/ly/wildos-console');m=json.loads((r/'source_identity.json').read_text())
source=Path(production.__file__);assert hashlib.sha256(source.read_bytes()).hexdigest()==m['members']['nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'];print('Installed source verified',source,flush=True)
sys.path.insert(0,str(r/'bundle/nav2_adapter/test'))
names=['test_planner_execution_ownership_ros','test_navigation_transactions_ros','test_tracking_handoff_ros','test_short_endpoint_handoff_ros','test_execution_mechanism_ros','test_graph_path_bridge_routing']
rclpy.init()
try:
 suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(name) for name in names)
 result=unittest.TextTestRunner(verbosity=2).run(suite)
 assert result.wasSuccessful()
finally:rclpy.shutdown()
