"""Prepare an explicit checkpoint path list without changing the Git index."""
from pathlib import Path
import hashlib
import json
import subprocess
import zipfile

root=Path(__file__).resolve().parent
project=root.parents[1]
spec=json.loads((root/'delta_manifest_v4.json').read_text())
for name,digest in spec['members'].items():
    assert hashlib.sha256((project/name).read_bytes()).hexdigest()==digest,name
verified=json.loads((root/'local_verification.json').read_text())
assert verified['passed'] and not verified['complete_navigation_passed']
extra=['docs/PROJECT_MEMORY.md','docs/EXECUTION_CONTINUITY_REPAIR_20261021.md',
       'docs/PHYSICAL_TRIAL_COMPARISON_20261021.md']
relative=root.relative_to(project)
for name in ['delta_manifest_v4.json','archive_identity.json','local_verification.json',
             'verify_evidence.py','source_after.log','all_callbacks_native.log']:
    extra.append((relative/name).as_posix())
with zipfile.ZipFile(root/'execution_evidence.zip') as z:
    payloads={
        'native_loss_regression.json':'v4/native_loss_regression.json',
        'physical_trial_summary.json':'v4/full/trial_20261021_085212/field_summary.json',
        'physical_trial_result.json':'v4/full/trial_20261021_085212/result.json',
        'real_callbacks.log':'v4/real_callbacks.log',
        'source_callbacks.log':'v4/source_callbacks.log',
        'drop_one.c':'transport_probe/drop_one.c',
        'probe_native_loss.py':'v4/probe_native_loss.py',
        'run_native_loss.sh':'v4/run_native_loss.sh',
    }
    dest=root/'checkpoint_evidence';dest.mkdir(exist_ok=True)
    for name,member in payloads.items():
        (dest/name).write_bytes(z.read(member))
        extra.append((relative/'checkpoint_evidence'/name).as_posix())
paths=sorted(set(spec['members'])|set(extra))
staged=subprocess.check_output(['git','diff','--cached','--name-only','-z'],cwd=project).decode().split('\0')
outside=[name for name in staged if name and name not in paths]
blobs={name:subprocess.check_output(['git','ls-files','--stage','--',name],cwd=project).decode().strip()
       for name in outside}
plan=dict(paths=paths,source_members=418,preserved_index_entries=blobs,
          head_before=subprocess.check_output(['git','rev-parse','HEAD'],cwd=project).decode().strip(),
          complete_navigation_passed=False,user_authorized_checkpoint=True)
(root/'commit_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
(root/'commit_paths.nul').write_bytes(b'\0'.join(name.encode() for name in paths)+b'\0')
print(json.dumps(dict(paths=len(paths),source_members=418,preserved_staged=len(outside))))
