"""Run the new behavioral test against frozen pre-cost production callbacks."""
from pathlib import Path
import sys
import unittest
import zipfile

root=Path(__file__).resolve().parents[2]
old=root/'deploy/20261016_recovery_handoff_candidate/official_wildos_beamng_bundle.zip'
out=root/'test_artifacts/frontier_cost_repair_20261016/before'
out.mkdir(parents=True,exist_ok=True)
with zipfile.ZipFile(old) as z:
    for name in z.namelist():
        if name.startswith('nav2_adapter/wildos_nav2_adapter/') and not name.endswith('/'):
            target=out/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(z.read(name))
sys.path[:0]=[str(out/'nav2_adapter'),str(root/'nav2_adapter/test')]
from offline_ros_fakes import install
install()
test=unittest.defaultTestLoader.loadTestsFromName('test_exploration_cost_ros.ExplorationCostTests.test_first_feasible_observation_path_cannot_skip_other_frontier_costs')
result=unittest.TextTestRunner(verbosity=2).run(test)
assert len(result.failures)==1 and not result.errors, (result.failures,result.errors)
print('PRE_COST_COUNTEREXAMPLE_CONFIRMED: original first feasible path dispatched before comparison')
