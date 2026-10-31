import hashlib
import json
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v71_heading_freshness')
library=root/'install/graphnav_planner/lib/libgraphnav_planner_component.so'
digest=hashlib.sha256(library.read_bytes()).hexdigest()
config=root/'full/config.json';values=json.loads(config.read_text())
values['library_sha256']['libgraphnav_planner_component.so']=digest
config.write_text(json.dumps(values,indent=2)+'\n')
print('Expected actual GraphNav library:',digest)
