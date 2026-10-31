"""Apply the tested ROS fixture update before any field process starts."""
from pathlib import Path
import hashlib
import json
import zipfile

root=Path('/home/ly/execution_continuity_repair_20261021/reachable_observation_v11')
incoming=root.parent/'reachable_observation_incoming'
assert not list((root/'full').glob('trial_*'))
plan=json.loads((incoming/'deployment_plan.json').read_text())
archive=incoming/'candidate_delta.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==plan['archive_sha256']
frozen=json.loads((root/'full/frozen_source_manifest.json').read_text())
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    delta=json.loads(z.read('delta_manifest.json'))
    for name,meta in delta.items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        data=z.read(name)
        assert hashlib.sha256(data).hexdigest()==meta['sha256']
        (root/'candidate'/name).write_bytes(data)
        frozen['members'][name]=meta['sha256']
(root/'full/frozen_source_manifest.json').write_text(json.dumps(frozen,indent=2)+'\n')
p=root/'build.sh'
p.write_text(p.read_text().replace('$PWD/candidate/nav2_adapter/test:$PWD/candidate/official_adapter:$PYTHONPATH',
                                 '$PWD/candidate/nav2_adapter/test:$PYTHONPATH'))
receipt=json.loads((root/'deployment_receipt.json').read_text())
receipt.update(archive_sha256=plan['archive_sha256'],changed=delta)
(root/'deployment_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(dict(archive_sha256=plan['archive_sha256'],members=len(frozen['members']))))
