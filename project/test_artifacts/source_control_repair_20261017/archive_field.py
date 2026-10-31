"""Seal closed test evidence, retaining large raw sensor DB externally by SHA."""
from pathlib import Path
import hashlib,json,zipfile,time
root=Path('/home/ly/source_control_repair_20261017')
drivers=sorted(root.glob('field_scope_baseline_search_*'))
assert drivers and all((p/'result.json').is_file() for p in drivers)
sessions=[]
for p in (root/'field_results').glob('session_*'):
    assert (p/'session_result.json').is_file(),p
    sessions.append(p)
files=[];external=[]
for p in [*drivers,*sessions,root/'field',root/'startup_harness_before_path_fix']:
    for f in p.rglob('*'):
        if not f.is_file() or '__pycache__' in f.parts:continue
        if 'sensor_interfaces' in f.parts and f.suffix=='.db3':external.append(f)
        else:files.append(f)
files.extend(root/x for x in ('live_summary.py','review_field.py','archive_field.py','inspect_closed_progress.py'))
files.extend(root/x for x in ('installed_identity.json','manifest.json','delta.json','build.log',
    'real_ros_callbacks.log','real_ros_callbacks_final.log','ownership_callbacks.log','resolution_dds.log',
    'test_resolution_dds.py','test_graphnav_owner_ros.py','build_and_test.sh','validate_and_drive.sh',
    'driver_launch.log'))
delta=json.loads((root/'delta.json').read_text())
files.extend(root/'candidate'/name for name in delta['changed'])
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(4*1024*1024),b''):h.update(chunk)
    return h.hexdigest()
metadata=dict(created_wall=time.time(),candidate='2e69583ced4ba5be3445d6c1c565e28812441ff2f76f5e7a410117af0fcc82d0',
    files={str(f.relative_to(root)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in sorted(set(files))},
    external_files=[dict(path=str(f),bytes=f.stat().st_size,sha256=sha(f),reason='Large raw sensor/navigation graph database; retained on server') for f in external])
archive=root/'scope_baseline_field_evidence.zip'
with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as bundle:
    for f in sorted(set(files)):bundle.write(f,str(f.relative_to(root)))
    bundle.writestr('field_evidence_manifest.json',json.dumps(metadata,indent=2)+'\n')
result=dict(path=str(archive),bytes=archive.stat().st_size,sha256=sha(archive),members=len(metadata['files'])+1,
    external_files=metadata['external_files'])
(root/'field_archive_identity.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
