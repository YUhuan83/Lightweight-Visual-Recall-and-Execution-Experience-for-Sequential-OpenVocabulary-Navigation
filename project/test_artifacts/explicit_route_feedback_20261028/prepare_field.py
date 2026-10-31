"""Prepare independent v58 field harness from v57; keep source versions intact."""
from pathlib import Path
import hashlib,json,shutil

base=Path('/home/ly/execution_continuity_review_20261022')
old=base/'v62_pending_route_owner';new=base/'v63_explicit_route_feedback'
full=new/'full'
full.mkdir(exist_ok=False)
for p in (old/'full').iterdir():
    if p.is_file() and not p.is_symlink():
        shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to('../candidate')
for name in ('config.json',):
    p=full/name;p.write_text(p.read_text().replace(str(old),str(new)))
    config=json.loads(p.read_text())
    config['library_sha256']['libgraphnav_planner_component.so']=hashlib.sha256((new/'install/graphnav_planner/lib/libgraphnav_planner_component.so').read_bytes()).hexdigest()
    p.write_text(json.dumps(config,indent=2))
manifest_path=full/'frozen_source_manifest.json'
manifest=json.loads(manifest_path.read_text())
manifest['members']={n:hashlib.sha256((new/'candidate'/n).read_bytes()).hexdigest() for n in manifest['members']}
manifest_path.write_text(json.dumps(manifest,indent=2))
run=(old/'run_full.sh').read_text().replace(str(old),str(new))
run=run.replace('full/driver_full.py','full/driver_arc_capture.py')
(new/'run_full.sh').write_text(run)
print('frozen_members',len(manifest['members']))
