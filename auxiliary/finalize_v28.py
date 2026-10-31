import hashlib,json,zipfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\execution_handoff_repair_20261025')
graph='nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
test='nav2_adapter/test/test_graph_path_bridge_routing.py'
with zipfile.ZipFile(root/'intent_handoff_delta.zip') as z:
    expected={graph:hashlib.sha256(z.read(graph)).hexdigest(),
              test:json.loads((root/'base_hashes.json').read_text())[test]}
with zipfile.ZipFile(root/'lookback_removal_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps(expected))
    for n in expected:z.write(root/'candidate'/n,n)
print(json.dumps(dict(changed=list(expected))))
