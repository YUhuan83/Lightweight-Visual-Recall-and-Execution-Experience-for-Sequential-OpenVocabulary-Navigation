"""Freeze and exercise the extracted candidate, without ROS or network."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
bundle = ROOT/'official_wildos_beamng_bundle.zip'
manifest = json.loads(bundle.with_suffix('.manifest.json').read_text())
assert hashlib.sha256(bundle.read_bytes()).hexdigest() == manifest['sha256']
target = OUT/'unpacked'
target.mkdir(exist_ok=True)
with zipfile.ZipFile(bundle) as archive:
    assert set(archive.namelist()) == set(manifest['members'])
    for name,digest in manifest['members'].items():
        destination = (target/name).resolve()
        assert destination.is_relative_to(target.resolve())
        data = archive.read(name)
        assert hashlib.sha256(data).hexdigest() == digest
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == digest
        destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_bytes(data)
with zipfile.ZipFile(OUT/'before/official_wildos_beamng_bundle.zip') as archive:
    old = {n:hashlib.sha256(archive.read(n)).hexdigest() for n in archive.namelist()}
changed = {n:dict(before=old.get(n),after=h) for n,h in manifest['members'].items() if old.get(n)!=h}
(OUT/'changes.json').write_text(json.dumps(changed,indent=2)+'\n',encoding='utf8')
env = dict(os.environ,TMP=str(OUT),TEMP=str(OUT))
env.pop('PYTHONPATH',None)
commands = {
    'unpacked_callbacks':[sys.executable,'tools/offline_callback_check.py'],
    'unpacked_goal_chains':[sys.executable,'-m','unittest','discover','-s','tools','-p','test_offline_goal_chain_replay.py','-v'],
    'unpacked_state_audit':[sys.executable,'-m','unittest','discover','-s','tools','-p','test_offline_navigation_state_audit.py','-v'],
}
results = []
for name,command in commands.items():
    result = subprocess.run(command,cwd=target,env=env,capture_output=True,text=True,
                            encoding='utf8',errors='replace',timeout=60)
    (OUT/(name+'.log')).write_text(result.stdout+'\n'+result.stderr,encoding='utf8')
    results.append(dict(name=name,returncode=result.returncode))
    print(name,result.returncode,flush=True)
    assert result.returncode == 0,(name,result.stdout[-1500:],result.stderr[-1500:])
report = dict(sha256=manifest['sha256'],members=len(manifest['members']),
              changed_files=len(changed),checks=results,passed=True,
              real_ros=False,beamng=False,field_acceptance=False)
(OUT/'candidate_verification.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
print(json.dumps(report,indent=2))
