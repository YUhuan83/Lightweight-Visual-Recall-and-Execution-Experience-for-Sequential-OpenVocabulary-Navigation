"""Exercise two new requirements against unchanged baseline production modules."""
import hashlib,importlib.util,json,sys,types,unittest
from pathlib import Path
from unittest.mock import patch
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/continuous_exploration_20261024'
manifest=json.loads((root/'baseline_manifest.json').read_text(encoding='utf8'))
for name in ('graph_path_bridge.py','visible_approach.py'):
    rel='nav2_adapter/wildos_nav2_adapter/'+name
    assert hashlib.sha256((repo/rel).read_bytes()).hexdigest()==manifest['members'][rel]['sha256']
sys.path[:0]=[str(repo/'nav2_adapter/test'),str(repo/'nav2_adapter'),str(repo/'official_adapter')]
from offline_ros_fakes import install
install()
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
class BaselineFixture(GraphPathBridge):
    # Bookkeeping for the new fixture only. No routing/action method changed.
    def intent_key(self,route):return ('fixture',)
shim=types.ModuleType('wildos_nav2_adapter.continuous_graph_path_bridge');shim.ContinuousGraphPathBridge=BaselineFixture
sys.modules[shim.__name__]=shim
spec=importlib.util.spec_from_file_location('continuous_before',root/'candidate/nav2_adapter/test/test_continuous_exploration_ros.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
suite=unittest.TestSuite(module.ContinuousExplorationTests(name) for name in (
'test_new_frontier_plans_while_old_action_continues','test_visible_intent_reuses_live_motion_without_reset'))
result=unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
