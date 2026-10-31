import hashlib,inspect,json
from pathlib import Path
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from wildos_nav2_adapter.continuous_graph_path_bridge import ContinuousGraphPathBridge
root=Path('/home/ly/execution_continuity_review_20261022/v28_intent_handoff')
receipt=json.loads((root/'receipt.json').read_text())
rows=[]
for cls in (GraphPathBridge,ContinuousGraphPathBridge):
    path=Path(inspect.getfile(cls));name='nav2_adapter/wildos_nav2_adapter/'+path.name
    sha=hashlib.sha256(path.read_bytes()).hexdigest()
    assert sha==receipt['changed'][name],(path,sha)
    rows.append(dict(cls=cls.__name__,source=str(path),sha256=sha))
assert not hasattr(GraphPathBridge,'maybe_side_lookback')
(root/'source_verified.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows))
