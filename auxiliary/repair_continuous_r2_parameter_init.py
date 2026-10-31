import hashlib,json,shutil,zipfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022')
target=root/'v24_continuous_exploration_r2'
assert not list((target/'full').glob('trial_*'))
receipt=json.loads((target/'continuous_r2_delta_receipt.json').read_text())
with zipfile.ZipFile('/tmp/continuous_r2_delta.zip') as z:
    base=json.loads(z.read('base_hashes.json'))
    names=[n for n in z.namelist() if n!='base_hashes.json']
    assert len(names)==7
    for name in names:
        p=target/'candidate'/name
        expected=receipt['changed_members'].get(name,base.get(name))
        assert expected and hashlib.sha256(p.read_bytes()).hexdigest()==expected,name
    with zipfile.ZipFile(target/'before_parameter_init_fix.zip','w',zipfile.ZIP_DEFLATED) as old:
        for name in names:old.write(target/'candidate'/name,name)
    shutil.copy2(target/'real_ros_review.log',target/'real_ros_review_before_parameter_init.log')
    shutil.copy2(target/'real_ros_report.json',target/'real_ros_report_before_parameter_init.json')
    for name in names:(target/'candidate'/name).write_bytes(z.read(name))
p=target/'full/frozen_source_manifest.json'
manifest=json.loads(p.read_text())
for name in names:manifest['members'][name]=hashlib.sha256((target/'candidate'/name).read_bytes()).hexdigest()
p.write_text(json.dumps(manifest,indent=2))
receipt['changed_members']={name:manifest['members'][name] for name in names}
receipt['scope']+=' Strategy parameter declared before base runtime guard registration; runtime mutation still rejected.'
(target/'continuous_r2_delta_receipt.json').write_text(json.dumps(receipt,indent=2))
print(json.dumps(receipt,indent=2))
