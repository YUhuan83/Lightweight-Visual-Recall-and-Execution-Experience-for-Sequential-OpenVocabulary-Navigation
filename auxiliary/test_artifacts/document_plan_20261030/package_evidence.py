import hashlib,json,zipfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v73_document_plan')
scope=json.loads((root/'candidate_scope.json').read_text())
with zipfile.ZipFile(root/'review_changes.zip','w',zipfile.ZIP_DEFLATED) as z:
    for name in scope['changed']:z.write(root/'candidate'/name,name)
print('Review archive SHA256',hashlib.sha256((root/'review_changes.zip').read_bytes()).hexdigest())
# Historical recordings contain no accepted 3D confirmation. Zero samples
# do not constitute a successful geometric or semantic arrival replay.
p=root/'historical_planar_arrival_replay.json';history=json.loads(p.read_text())
for r in history:
    r['exercised']=r['checked']>0
    if not r['exercised']:r['metric_consistent']=None
p.write_text(json.dumps(history,indent=2)+'\n')
