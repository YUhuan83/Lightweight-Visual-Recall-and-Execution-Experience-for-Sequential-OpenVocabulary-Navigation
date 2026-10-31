import hashlib,json,shutil,subprocess,time,zipfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022')
source=root/'v25_fast_shift';target=root/'v26_body_arrival'
assert not list((target/'full').glob('trial_*'))
pre=json.loads(subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,capture_output=True,text=True).stdout)
assert not pre['processes'] and not pre['display148_lock']
for name in ('real_ros_review.log','real_ros_report.json'):
    p=target/name
    if p.exists():shutil.copy2(p,target/(name+'.fixture_failure_'+str(time.time_ns())))
old=json.loads((target/'body_arrival_receipt.json').read_text())
with zipfile.ZipFile('/tmp/body_arrival_delta.zip') as z:
    base=json.loads(z.read('base_hashes.json'));names=[n for n in z.namelist() if n!='base_hashes.json']
    assert len(names)==12
    for n,sha in base.items():assert hashlib.sha256((source/'candidate'/n).read_bytes()).hexdigest()==sha,n
    for n in names:
        if '/wildos_nav2_adapter/' in n or '/beamng_wildos_adapter/' in n:
            assert hashlib.sha256(z.read(n)).hexdigest()==old['changed_members'][n],n
        (target/'candidate'/n).write_bytes(z.read(n))
p=target/'full/frozen_source_manifest.json';m=json.loads(p.read_text())
for n in names:m['members'][n]=hashlib.sha256((target/'candidate'/n).read_bytes()).hexdigest()
p.write_text(json.dumps(m,indent=2))
old['changed_members']={n:m['members'][n] for n in names}
old['test_fixture_update']='Use actual Time/GraphNode and explicit publisher mocks in existing assertions; production unchanged.'
(target/'body_arrival_receipt.json').write_text(json.dumps(old,indent=2))
print(json.dumps(dict(changed=len(names),members=len(m['members']),production_unchanged=True)))
