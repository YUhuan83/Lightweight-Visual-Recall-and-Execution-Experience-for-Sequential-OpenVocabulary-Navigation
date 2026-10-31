"""Verify before sources against the preserved archive; restore the original test snapshot."""
from pathlib import Path
import hashlib
import json
import zipfile

root=Path(__file__).resolve().parents[2]
out=Path(__file__).resolve().parent/'before'
bundle=root/'deploy/20261018_smoothing_boundary_candidate/official_wildos_beamng_bundle.zip'
manifest=json.loads(bundle.with_suffix('.manifest.json').read_text(encoding='utf-8'))
files=['nav2_adapter/wildos_nav2_adapter/navigation_transactions.py',
       'nav2_adapter/wildos_nav2_adapter/visible_approach.py',
       'nav2_adapter/test/test_navigation_transactions_ros.py']
records={}
with zipfile.ZipFile(bundle) as zf:
    for name in files:
        data=zf.read(name)
        assert hashlib.sha256(data).hexdigest()==manifest['members'][name]
        if '/test/' not in name:assert (root/name).read_bytes()==data
        target=out/Path(name).name
        if target.exists():assert target.read_bytes()==data
        else:target.write_bytes(data)
        records[name]=manifest['members'][name]
(out/'manifest.json').write_text(json.dumps(records,indent=2)+'\n',encoding='utf-8')
print(json.dumps(records,indent=2))
