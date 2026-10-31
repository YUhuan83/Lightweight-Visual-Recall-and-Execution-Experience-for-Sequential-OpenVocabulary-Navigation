"""Prepare independent v58 field harness from v57; keep source versions intact."""
from pathlib import Path
import hashlib,json,shutil

base=Path('/home/ly/execution_continuity_review_20261022')
old=base/'v60_replan_horizon';new=base/'v61_confirmed_goal_owner'
full=new/'full'
full.mkdir(exist_ok=False)
for p in (old/'full').iterdir():
    if p.is_file() and not p.is_symlink():
        shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to('../candidate')
for name in ('config.json',):
    p=full/name;p.write_text(p.read_text().replace(str(old),str(new)))
manifest_path=full/'frozen_source_manifest.json'
manifest=json.loads(manifest_path.read_text())
manifest['members']={n:hashlib.sha256((new/'candidate'/n).read_bytes()).hexdigest() for n in manifest['members']}
manifest_path.write_text(json.dumps(manifest,indent=2))
run=(old/'run_full.sh').read_text().replace(str(old),str(new))
run=run.replace('full/driver_full.py','full/driver_arc_capture.py')
(new/'run_full.sh').write_text(run)
print('frozen_members',len(manifest['members']))
