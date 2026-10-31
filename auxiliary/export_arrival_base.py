"""Export exactly the frozen v25 source members; never alter the old version."""
import hashlib,json,subprocess,zipfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v25_fast_shift')
pre=json.loads(subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,capture_output=True,text=True).stdout)
assert not pre['processes'] and not pre['display148_lock'],pre
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
archive=Path('/tmp/arrival_v25_source.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('_frozen_source_manifest.json',json.dumps(manifest,indent=2))
    for name,sha in manifest['members'].items():
        data=(root/'candidate'/name).read_bytes()
        assert hashlib.sha256(data).hexdigest()==sha,name
        z.writestr(name,data)
print(json.dumps(dict(source=str(root),members=len(manifest['members']),archive=str(archive),sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),preflight=pre)))
