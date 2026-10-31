"""Seal completed component/physical evidence; keep raw bags in place with SHA256 identities."""
from pathlib import Path
import hashlib
import json
import math
import shutil
import subprocess
import zipfile

root = Path(__file__).resolve().parent
parent = Path('/home/ly/vehicle_profile_repair_20261019')
run = root/'physical/short_20261020_164936'
assert (run/'interfaces/metadata.yaml').is_file()
result = json.loads((run/'result.json').read_text())
assert result['passed'] and not result['errors']
execution = json.loads((run/'execution/result.json').read_text())
events = [json.loads(line) for line in (run/'execution/events.jsonl').read_text().splitlines()]
points = [e['point'] for e in events if e['kind']=='motion_pose']
initial = execution['initial']; heading = initial[2]
cross_track = [abs((p[0]-initial[0])*math.sin(heading)-(p[1]-initial[1])*math.cos(heading)) for p in points]
native = [e['value'] for e in events if e['kind']=='native']
def percentile(values, fraction):
    values=sorted(values)
    return values[min(len(values)-1,math.ceil(len(values)*fraction)-1)] if values else None
summary = dict(scope='One physical straight reverse action only; not complete navigation or historical delay closure',
    run=str(run), passed=result['passed'], action_status=execution['action_status'],
    planned_length_m=execution['plans'][0]['length_m'], sections=execution['plans'][0]['sections'],
    sdk_translation_xy_m=execution['sdk_translation_xy_m'], odom_displacement_m=execution['displacement_m'],
    endpoint_distance_m=execution['endpoint_distance_m'], stop_gate_closed=execution['stop_gate_closed'],
    stop_samples=len(execution['stop_samples']), trajectory_samples=len(points),
    max_cross_track_to_initial_heading_m=max(cross_track,default=None),
    native_rpc_p95_ms=percentile([n['control_rpc_ms'] for n in native if n.get('control_rpc_ms') is not None],.95),
    runtime_libraries=result['runtime_libraries'], complete_navigation_passed=False,
    visual_search_started=False, video=False)
(root/'physical_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
pre=subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],
                   capture_output=True,text=True,check=True,timeout=20)
assert json.loads(pre.stdout)['processes']==[]
(root/'after_inventory.json').write_text(pre.stdout)
# Preserve the added failing regression separately, then restore original
# source bytes to the directory described by the pre-change manifest.
if not (root/'baseline_regression_test.cpp').exists():
    shutil.copy2(root/'before/test/test_planner.cpp',root/'baseline_regression_test.cpp')
shutil.copy2(parent/'candidate/ackermann_smac/test/test_planner.cpp',root/'before/test/test_planner.cpp')
before=json.loads((root/'before_manifest.json').read_text())
assert all(hashlib.sha256((root/'before'/n).read_bytes()).hexdigest()==h for n,h in before.items())

def seal(base, destination, label):
    files={}; excluded=[]
    for path in sorted(base.rglob('*')):
        if not path.is_file() or path.is_symlink(): continue
        relative=path.relative_to(base)
        if relative.parts[0] in {'build','build_log','install','fixture_build','tmp'}: continue
        if '__pycache__' in relative.parts: continue
        if path.suffix in {'.db3','.mcap'}:
            excluded.append(dict(path=str(relative),bytes=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            continue
        if path.suffix=='.zip' or path.name.endswith('.tgz'): continue
        files[str(relative).replace('\\','/')]=path
    manifest=dict(scope=label,raw_bags_retained_on_server=excluded,
                  members={n:hashlib.sha256(p.read_bytes()).hexdigest() for n,p in files.items()})
    with zipfile.ZipFile(destination,'x',zipfile.ZIP_DEFLATED) as archive:
        archive.writestr('EVIDENCE_MANIFEST.json',json.dumps(manifest,indent=2)+'\n')
        for name,path in files.items(): archive.write(path,name)
    return dict(path=str(destination),bytes=destination.stat().st_size,members=len(files),
                sha256=hashlib.sha256(destination.read_bytes()).hexdigest())

seals=[seal(parent,root/'profile_component_evidence.zip','Vehicle geometry, real ROS/DDS and previous failed short runs; no complete success'),
       seal(root,root/'continuous_goal_evidence.zip','Continuous goal fix and one accepted physical reverse; not complete navigation')]
(root/'evidence_archives.json').write_text(json.dumps(seals,indent=2)+'\n')
print(json.dumps(dict(physical=summary,archives=seals),indent=2))
