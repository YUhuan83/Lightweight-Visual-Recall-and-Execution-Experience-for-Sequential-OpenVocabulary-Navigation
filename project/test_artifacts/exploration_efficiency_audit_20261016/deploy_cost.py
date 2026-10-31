from pathlib import Path
import hashlib
import json
import os
import subprocess
import zipfile

root=Path('/home/ly/wildos-console').resolve()
r=Path('/home/ly/frontier_cost_repair_20261016').resolve()
identity=json.loads((r/'release_identity.json').read_text())
manifest=json.loads((r/'release.manifest.json').read_text())
assert manifest['sha256']==identity['source']=='e6f2811833ea740887cd9bf08e44814699dbff850646aab9ed01701da381cbeb'
assert 'Final compiled source, Python import and library identity verified' in (r/'release_validation.log').read_text()
assert '\nFAILED (' not in (r/'release_validation.log').read_text()
running=subprocess.check_output(['ps','-u',str(os.getuid()),'-o','comm='],text=True).splitlines()
assert not set(x.strip() for x in running)&{'BeamNG.tech.x64','planner_node','planner_server','controller_server','rviz2','rosbag2_transport'},running
old=json.loads((root/'source_identity.json').read_text())
assert old['sha256']=='11af6c17b0335766c50ce892bf128af5ab88f9a7a82571e0799f3e555e06d9cd'
for name,sha in old['members'].items():assert hashlib.sha256((root/'bundle'/name).read_bytes()).hexdigest()==sha,name
stage=root/'bundle_frontier_cost_e6f28118'
stage.mkdir(exist_ok=False)
with zipfile.ZipFile(r/'release.zip') as z:
    for name,sha in manifest['members'].items():
        value=z.read(name);assert hashlib.sha256(value).hexdigest()==sha
        p=stage/name;assert p.resolve().is_relative_to(stage.resolve())
        p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(value)
backup=root/'bundle_11af6c17_before_cost_20261016'
assert not backup.exists()
oldcfg=(root/'console.json').read_bytes()
(r/'console_before.json').write_bytes(oldcfg)
(r/'source_identity_before.json').write_text(json.dumps(old,indent=2))
os.replace(root/'bundle',backup)
os.replace(stage,root/'bundle')
(root/'source_identity.json').write_text(json.dumps(manifest,indent=2))
cfg=json.loads(oldcfg)
cfg['setups'] += [str(r/'install/setup.bash'),str(r/'release_install/setup.bash')]
cfg['library_sha256']={Path(p).name:sha for p,sha in identity['libraries'].items()}
(root/'console.json').write_text(json.dumps(cfg,indent=2))
field=r/'field';field.mkdir(exist_ok=False)
(field/'sensor_full_console.json').write_text(json.dumps(cfg,indent=2))
previous=Path('/home/ly/exploration_owner_field_20261016')
for name in ('sensor_console.py','sensor_full_console.sh','drive_sensor_full.py','progress.py',
             'audit_runtime.py','analyze_closed.py','record_execution_contract.py','pack_closed.py'):
    text=(previous/name).read_text()
    text=text.replace(str(previous),str(field)).replace('field_exploration_owner_','field_frontier_cost_')
    text=text.replace(old['sha256'],manifest['sha256'])
    if name=='sensor_full_console.sh':
        anchor='source /home/ly/exploration_owner_repair_20261016/final_install/setup.bash'
        text=text.replace(anchor,anchor+'\nsource '+str(r/'install/setup.bash')+'\nsource '+str(r/'release_install/setup.bash'))
    if name=='pack_closed.py':
        text=text.replace("(t,s,r/'field_frontier_cost_search_20261016_095603',r/'audit_results/session_20261016_095604')",'(t,s)')
        text=text.replace("out=r/'exploration_owner_field_20261016_095740_core.zip'","out=r/(t.name+'_core.zip')")
    (field/name).write_text(text,newline='\n')
report=dict(source=manifest['sha256'],previous=old['sha256'],backup=str(backup),
    changed=[n for n,h in manifest['members'].items() if old['members'].get(n)!=h],
    config=cfg,vehicle_started=False)
(r/'deployment.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
