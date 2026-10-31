"""Freeze a single-module ground-classification change from preserved v62."""
from pathlib import Path
import shutil,json,hashlib
base=Path('/home/ly/execution_continuity_review_20261022')
old=base/'v62_pending_route_owner';new=base/'v64_ground_surface'
new.mkdir(exist_ok=False)
shutil.copytree(old/'candidate',new/'candidate')
shutil.copy2(base/'v63_explicit_route_feedback/pointcloud_candidate.py',new/'candidate/nav2_adapter/wildos_nav2_adapter/pointcloud.py')
full=new/'full';full.mkdir()
for p in (old/'full').iterdir():
    if p.is_file() and not p.is_symlink():shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to('../candidate')
p=full/'config.json';p.write_text(p.read_text().replace(str(old),str(new)))
p=full/'frozen_source_manifest.json';manifest=json.loads(p.read_text())
manifest['members']={name:hashlib.sha256((new/'candidate'/name).read_bytes()).hexdigest() for name in manifest['members']}
p.write_text(json.dumps(manifest,indent=2))
(new/'run_full.sh').write_text((old/'run_full.sh').read_text().replace(str(old),str(new)))
review=(base/'v63_explicit_route_feedback/isolated_ros_review.py').read_text()
review=review.replace("root=Path.cwd()/'isolated_ros'","modules += ('test_curved_scan_ground',)\nroot=Path.cwd()/'isolated_ros'")
(new/'isolated_ros_review.py').write_text(review)
print('candidate',new,'frozen_members',len(manifest['members']))
