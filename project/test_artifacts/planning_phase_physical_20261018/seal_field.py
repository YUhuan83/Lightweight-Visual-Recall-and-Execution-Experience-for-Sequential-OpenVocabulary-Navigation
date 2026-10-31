"""Archive only closed trials; retain the large raw sensor bag on the server."""
import hashlib,json,time,zipfile
from pathlib import Path
root=Path('/home/ly/planning_phase_physical_20261018')
drivers=sorted(root.glob('field_yaw_left90_search_*'))
sessions=sorted((root/'field_results').glob('session_*'))
assert drivers and all((p/'result.json').exists() for p in drivers),'Driver still running'
assert sessions and all((p/'session_result.json').exists() for p in sessions),'Session still running'
manifest=json.loads((root/'manifest.json').read_text())
files=set();external=[]
for folder in [*drivers,*sessions,root/'field']:
    for p in folder.rglob('*'):
        if not p.is_file() or '__pycache__' in p.parts:continue
        if 'sensor_interfaces' in p.parts and p.suffix=='.db3':external.append(p)
        else:files.add(p)
for p in root.iterdir():
    if p.is_file() and p.suffix in ('.json','.log','.py','.sh') and p.name!='field_archive_identity.json':files.add(p)
for p in (root/'initial_a69').glob('*.log'):files.add(p)
for name in ('nav2_adapter/wildos_nav2_adapter/source_control.py',
             'nav2_adapter/wildos_nav2_adapter/native_vehicle_control.py',
             'nav2_adapter/wildos_nav2_adapter/cmd_vel_to_control.py',
             'nav2_adapter/wildos_nav2_adapter/execution_feedback.py',
             'tools/server_test_preflight.py','tools/test_source_control_preflight.py',
             'nav2_adapter/wildos_nav2_adapter/visible_approach.py',
             'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
             'official_adapter/beamng_wildos_adapter/goal_standoff.py',
             'nav2_adapter/test/test_visible_approach_ros.py',
 'ackermann_rpp/src/controller.cpp', 'nav2_adapter/test/test_controller_source_ros.py', 'ackermann_smac/src/planner.cpp', 'ackermann_smac/src/path_contract.cpp',
 'ackermann_smac/include/wildos_ackermann_smac/path_contract.hpp',
 'nav2_adapter/test/test_execution_progress_ros.py'):
    files.add(root/'candidate'/name)
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as stream:
        for part in iter(lambda:stream.read(4*1024*1024),b''):h.update(part)
    return h.hexdigest()
meta=dict(created_wall=time.time(),candidate=manifest['sha256'],
    files={str(p.relative_to(root)):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(files)},
    external_files=[dict(path=str(p),bytes=p.stat().st_size,sha256=sha(p),reason='Closed raw sensor bag retained on server') for p in external])
archive=root/'full_vehicle_field_evidence.zip'
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED,compresslevel=1) as z:
    for p in sorted(files):z.write(p,p.relative_to(root).as_posix())
    z.writestr('field_evidence_manifest.json',json.dumps(meta,indent=2)+'\n')
identity=dict(path=str(archive),bytes=archive.stat().st_size,sha256=sha(archive),members=len(files)+1,
    external_files=meta['external_files'])
(root/'field_archive_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
print(json.dumps(identity,indent=2))
