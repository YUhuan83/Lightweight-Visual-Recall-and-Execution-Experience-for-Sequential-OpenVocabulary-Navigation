"""Package a small reversible candidate on the last frozen 417-member source."""
from pathlib import Path
import hashlib
import json
import zipfile

root = Path(__file__).resolve().parent
project = root.parents[1]
parent = json.loads((project / 'captures/20261020_confirmed_recovery_trial225946/frozen_source_manifest.json').read_text())
names = ['nav2_adapter/wildos_nav2_adapter/exploration_cost.py',
         'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
         'nav2_adapter/test/test_committed_frontier_path_ros.py', 'tools/offline_callback_check.py', 'nav2_adapter/wildos_nav2_adapter/native_vehicle_control.py', 'nav2_adapter/test/test_source_control.py']
delta = {n: hashlib.sha256((project / n).read_bytes()).hexdigest() for n in names}
members = dict(parent['members']); members.update(delta)
unrelated = [n for n, h in parent['members'].items()
             if n not in delta and hashlib.sha256((project / n).read_bytes()).hexdigest() != h]
assert not unrelated, unrelated
spec = dict(parent=parent, delta=delta, members=members)
(root / 'delta_manifest_v4.json').write_text(json.dumps(spec, indent=2) + '\n')
with zipfile.ZipFile(root / 'candidate_delta_v4.zip', 'w', zipfile.ZIP_DEFLATED) as z:
    for n in names: z.write(project / n, 'delta/' + n)
    z.write(root / 'delta_manifest_v4.json', 'delta_manifest_v4.json')
print(json.dumps(dict(members=len(members), changed=len(delta),
                      sha256=hashlib.sha256((root/'candidate_delta_v4.zip').read_bytes()).hexdigest())))

