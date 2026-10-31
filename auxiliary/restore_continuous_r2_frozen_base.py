import hashlib,json,shutil,zipfile
from pathlib import Path
target=Path('/home/ly/execution_continuity_review_20261022/v24_continuous_exploration_r2')
assert not list((target/'full').glob('trial_*'))
receipt=json.loads((target/'continuous_r2_delta_receipt.json').read_text())
name='nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
p=target/'candidate'/name
assert hashlib.sha256(p.read_bytes()).hexdigest()==receipt['changed_members'][name]
shutil.copy2(p,target/'graph_bridge_rejected_local_base.py')
shutil.copy2(target/'real_ros_review.log',target/'real_ros_review_rejected_local_base.log')
with zipfile.ZipFile('/tmp/continuous_r2_delta.zip') as z:p.write_bytes(z.read(name))
sha=hashlib.sha256(p.read_bytes()).hexdigest()
receipt['changed_members'][name]=sha
receipt['scope']+=' Base hook is applied to exact frozen v24, not local unshipped arrival changes.'
(target/'continuous_r2_delta_receipt.json').write_text(json.dumps(receipt,indent=2))
p=target/'full/frozen_source_manifest.json'
manifest=json.loads(p.read_text());manifest['members'][name]=sha
p.write_text(json.dumps(manifest,indent=2))
print('Restored frozen base with initialization hook only',sha)
