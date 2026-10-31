"""Archive test evidence without copying large raw ROS bags or credentials."""
from pathlib import Path
import hashlib
import json
import math
import zipfile

root=Path('/home/ly/full_town_memory_20261002')
ws=Path('/home/ly/beamng_ros2_0394_ws')
manifest={}
for package,names in {
    'wildos_nav2_adapter':['graph_path_bridge.py','graph_route.py','motion_path.py','drive_math.py','native_vehicle_control.py'],
    'beamng_wildos_adapter':['node.py','measurement_clock.py']}.items():
    for name in names:
        p=ws/'src'/package/package/name
        manifest[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
root.joinpath('final_deployment_manifest.json').write_text(json.dumps(manifest,indent=2))
summary=[]
for p in sorted(root.glob('house_attempt*/full_test.json')):
    value=json.loads(p.read_text())
    native=p.parent/'native_samples.jsonl'
    if native.exists() and value.get('start_position'):
        born=value['start_position']
        rows=[json.loads(line) for line in native.read_text().splitlines()]
        departures=[math.dist(r['position'][:2],born) for r in rows if r['phase']=='house_search']
        returns=[math.dist(r['position'][:2],born) for r in rows if r['phase']=='station_return']
        value['measured_max_departure']=max(departures,default=0)
        value['closest_return_distance_to_spawn']=min(returns,default=None)
        value['return_started_distance_to_spawn']=returns[0] if returns else None
        value['meaningful_return_motion']=(bool(returns) and returns[0]>2 and returns[0]-min(returns)>.5)
        value['return_flag_is_valid_full_sequence']=(bool(value.get('returned')) and
            value['meaningful_return_motion'] and bool(value.get('house_found')) and max(departures,default=0)>=20)
    summary.append({'attempt':p.parent.name,**{k:v for k,v in value.items() if k!='events'}})
root.joinpath('round_results.json').write_text(json.dumps(summary,indent=2))
health=root/'native_health.json'
if health.exists():
    root.joinpath('vehicle_geometry_check.json').write_text(health.read_text())
archive=root/'evidence_20261002.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(root.rglob('*')):
        if not p.is_file(): continue
        rel=p.relative_to(root)
        if any(part in ('adapter_backup','official_adapter_before_clock_fix') for part in rel.parts): continue
        if 'rosbag' in rel.parts and p.name!='metadata.yaml': continue
        if p.suffix not in ('.json','.jsonl','.log','.png','.npz','.py','.yaml','.mp4'): continue
        if p.suffix=='.mp4' and p.parent.name not in ('house_attempt4','house_attempt7'): continue
        z.write(p,rel.as_posix())
print(json.dumps({'archive':str(archive),'bytes':archive.stat().st_size,
                  'rounds':[{k:r.get(k) for k in ('attempt','passed','house_found','maximum_departure','returned','memory_path_latency','error')} for r in summary]},indent=2))
