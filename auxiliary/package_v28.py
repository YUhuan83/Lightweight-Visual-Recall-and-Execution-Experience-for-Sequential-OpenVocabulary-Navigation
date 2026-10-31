import hashlib,json,zipfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\execution_handoff_repair_20261025')
base=json.loads((root/'base_hashes.json').read_text())
changed=[n for n,sha in base.items() if hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()!=sha]
assert set(changed)=={'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
 'nav2_adapter/wildos_nav2_adapter/continuous_graph_path_bridge.py',
 'nav2_adapter/test/test_continuous_exploration_ros.py'},changed
with zipfile.ZipFile(root/'intent_handoff_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed}))
    for n in changed:z.write(root/'candidate'/n,n)
print(json.dumps(dict(changed=changed,sha256=hashlib.sha256((root/'intent_handoff_delta.zip').read_bytes()).hexdigest())))
