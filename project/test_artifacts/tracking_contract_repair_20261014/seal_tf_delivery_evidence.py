"""Seal completed component logs and explicit snapshots of live stack logs."""
import hashlib,json,time,zipfile
from pathlib import Path
BASE=Path('/home/ly/town_memory_validation_20261014/tracking_contract_candidate')
archive=BASE/'tf_delivery_evidence_20261014.zip'
assert not archive.exists()
for trial in range(4,10):
    result=BASE/f'physical_cusp_component{trial}_observer/result.json'
    assert result.is_file(),f'Observer {trial} not sealed'
files=set()
for trial in range(3,10):
    for directory in (BASE/f'physical_cusp_component{trial}',BASE/f'physical_cusp_component{trial}_observer'):
        if directory.exists():
            files.update(p for p in directory.rglob('*') if p.is_file())
for i in range(1,6):
    directory=BASE/f'resume_session{i}'
    files.update(p for p in directory.rglob('*') if p.is_file()
        and 'source' not in p.relative_to(directory).parts and '__pycache__' not in p.parts
        and p.suffix not in ('.zip','.pyc'))
files.update(p for p in (BASE/'component_tools').iterdir() if p.is_file() and p.suffix in ('.py','.sh','.xml'))
files.update(p for p in (BASE/'upstream_audit').iterdir() if p.is_file())
files.add(BASE/'installation_identity.json')
report={'scope':'Physical interface/cusp components; no semantic target arrival or stability rate',
    'sealed_at_wall_ns':time.time_ns(),'members':{},
    'live_log_snapshots':['resume_session5/stack.log','resume_session5/sensor.log'],
    'debug_component6_is_acceptance':False}
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
    for path in sorted(files):
        assert path.resolve().is_relative_to(BASE.resolve()) and not path.is_symlink()
        name=path.relative_to(BASE).as_posix();data=path.read_bytes()
        report['members'][name]={'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
        z.writestr(name,data)
    z.writestr('evidence_manifest.json',json.dumps(report,indent=2))
assert zipfile.ZipFile(archive).testzip() is None
checksum={'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'bytes':archive.stat().st_size,
    'members':len(report['members']),'scope':report['scope']}
archive.with_suffix('.json').write_text(json.dumps(checksum,indent=2));print(json.dumps(checksum))
