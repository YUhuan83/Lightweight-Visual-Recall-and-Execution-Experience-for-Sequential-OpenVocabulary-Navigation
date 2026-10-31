from pathlib import Path
import hashlib
import json
import zipfile

r=Path('/home/ly/planning_recovery_handoff_20261016')
identity=json.loads((r/'validation_identity.json').read_text())
assert identity['candidate']=='415c0bef49adf99f0fd7c64de75d7210973715090c6002fe8742064b92f7dde3'
log=(r/'validation.log').read_text()
assert 'Ran 32 tests' in log and 'Ran 35 tests' in log and 'Ran 5 tests' in log
assert '\nFAILED (' not in log
files={}
for name in ('validate_recovery_handoff.sh','official_wildos_beamng_bundle.manifest.json',
             'candidate_delta.json','validation_identity.json','validation.log','final_server_state.txt'):
    p=r/name
    if p.is_file():files['validation/'+name]=p
alt=Path('/home/ly/exploration_efficiency_audit_20261016/frontier_alternatives')
for p in alt.iterdir():
    if p.is_file() and p.name.startswith(('candidates','input','case2')):
        files['frontier_alternatives/'+p.name]=p
for name in ('frontier_alternatives.py','replay_frontiers.sh'):
    files['diagnostics/'+name]=alt.parent/name
cleanup=Path('/home/ly/disk_cleanup_20261016')
for name in ('archive_deleted.json','old_install_compare.json','verified_files_deleted.json'):
    files['cleanup/'+name]=cleanup/name
manifest={name:hashlib.sha256(p.read_bytes()).hexdigest() for name,p in files.items()}
with zipfile.ZipFile(r/'evidence.zip','w',zipfile.ZIP_DEFLATED) as z:
    for name,p in files.items():z.write(p,name)
    z.writestr('MANIFEST.json',json.dumps(manifest,indent=2))
meta=dict(sha256=hashlib.sha256((r/'evidence.zip').read_bytes()).hexdigest(),members=len(files),
    scope='Isolated ROS/DDS recovery handoff, fixed recorded map frontier comparison, explicit cleanup audit. No vehicle test or full search success.')
(r/'evidence_meta.json').write_text(json.dumps(meta,indent=2))
print(json.dumps(meta,indent=2))
