"""Delete exactly the two trial5 videos explicitly confirmed by the user."""
from pathlib import Path
import json
root=Path('/home/ly/full_town_memory_20261002/stability_frontier_trial5').resolve(strict=True)
files=[]
for name in ('rviz_live.mp4','front_camera.mp4'):
    path=root/name
    if not path.exists():continue
    resolved=path.resolve(strict=True)
    if path.is_symlink() or resolved.parent!=root or not path.is_file():raise RuntimeError('Unexpected video path')
    files.append({'path':str(resolved),'bytes':path.stat().st_size})
(root/'deleted_latest_videos_20261002.json').write_text(json.dumps(files,indent=2))
for entry in files:Path(entry['path']).unlink()
print(json.dumps({'deleted':len(files),'remaining':[name for name in ('rviz_live.mp4','front_camera.mp4') if (root/name).exists()]}))
