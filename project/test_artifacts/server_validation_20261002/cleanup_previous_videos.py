"""Delete an explicit old-trial video list; preserve current trial5 evidence."""
from pathlib import Path
import json

roots=[Path('/home/ly/full_town_memory_20261002'),Path('/home/ly/full_town_memory_20260930')]
old_names={roots[0]:[
    'house_attempt1/front_camera.mp4','house_attempt2/front_camera.mp4',
    'house_attempt3/front_camera.mp4','house_attempt4/front_camera.mp4',
    'house_attempt5/front_camera.mp4','house_attempt6/front_camera.mp4',
    'house_attempt7/front_camera.mp4','house_geometry_rviz_attempt1/front_camera.mp4',
    'house_geometry_rviz_attempt2/front_camera.mp4','navigation_repair_trial1/front_camera.mp4',
    'navigation_repair_trial1/rviz_live.mp4','geometry_rviz_verification/rviz_live.mp4',
    'stability_frontier_trial2/front_camera.mp4','stability_frontier_trial2/rviz_live.mp4',
    'stability_frontier_trial3/front_camera.mp4','stability_frontier_trial3/rviz_live.mp4',
    'stability_frontier_trial4/front_camera.mp4','stability_frontier_trial4/rviz_live.mp4'],
    roots[1]:['front_camera.mp4','attempt2/front_camera.mp4','attempt3/front_camera.mp4',
              'attempt4/front_camera.mp4','attempt5/front_camera.mp4','attempt6/front_camera.mp4']}
files=[]
for root,names in old_names.items():
    base=root.resolve(strict=True)
    for name in names:
        path=root/name
        if not path.exists():continue
        resolved=path.resolve(strict=True)
        if path.is_symlink() or not resolved.is_relative_to(base) or not path.is_file():
            raise RuntimeError('Refusing unexpected path: '+str(path))
        files.append({'path':str(resolved),'bytes':path.stat().st_size})
manifest=roots[0]/'deleted_test_videos_20261002.json'
if manifest.exists():raise RuntimeError('Cleanup already performed; preserve original manifest')
manifest.write_text(json.dumps(files,indent=2))
for entry in files:Path(entry['path']).unlink()
print(json.dumps({'deleted':len(files),'bytes':sum(p['bytes'] for p in files),
    'latest_trial_preserved':all((roots[0]/'stability_frontier_trial5'/p).is_file() for p in ('front_camera.mp4','rviz_live.mp4'))}))
