import hashlib,json,zipfile
from pathlib import Path
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/continuous_exploration_20261024'
manifest=json.loads((root/'baseline_manifest.json').read_text(encoding='utf8'))
prefixes=('nav2_adapter/','official_adapter/','ackermann_smac/','ackermann_rpp/','wildos_port/repro_check/src/','config/','tools/')
changes=[];checked=0
for name,entry in manifest['members'].items():
    if name.startswith(prefixes):
        checked+=1
        if not (repo/name).exists() or hashlib.sha256((repo/name).read_bytes()).hexdigest()!=entry['sha256']:changes.append(name)
assert not changes,changes
archive=root/'baseline_source.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==manifest['archive_sha256']
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
candidate=root/'candidate';changed=[];added=[]
for p in (candidate/'nav2_adapter').rglob('*'):
    if not p.is_file() or '__pycache__' in p.parts:continue
    name=p.relative_to(candidate).as_posix()
    if name not in manifest['members']:added.append(name)
    elif hashlib.sha256(p.read_bytes()).hexdigest()!=manifest['members'][name]['sha256']:changed.append(name)
result=dict(original_runtime_sources_unchanged=True,original_files_checked=checked,
    archive_sha256=manifest['archive_sha256'],candidate_changed=sorted(changed),candidate_added=sorted(added))
(root/'preservation_verified.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(result,ensure_ascii=False,indent=2))
