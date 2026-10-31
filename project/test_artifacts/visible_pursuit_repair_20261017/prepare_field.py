"""Freeze the pursuit repair and bind the tested left-90 field harness."""
import ast,hashlib,json,shutil,zipfile
from pathlib import Path

out=Path(__file__).resolve().parent;root=out.parents[1]
previous='/home/ly/source_control_repair_20261017'
yaw='/home/ly/visible_route_yaw90_20261017'
remote='/home/ly/visible_pursuit_repair_20261017'
old_digest='2e69583ced4ba5be3445d6c1c565e28812441ff2f76f5e7a410117af0fcc82d0'
manifest=json.loads((root/'official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
baseline=json.loads((root/'deploy/20261017_source_control_preflight_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
changed={n for n,h in manifest['members'].items() if baseline['members'].get(n)!=h}
assert changed=={'nav2_adapter/wildos_nav2_adapter/visible_approach.py',
                 'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
                 'official_adapter/beamng_wildos_adapter/goal_standoff.py',
                 'nav2_adapter/test/test_visible_approach_ros.py'},changed
assert set(manifest['members'])==set(baseline['members'])
candidate=root/'deploy/20261017_visible_pursuit_candidate';candidate.mkdir(exist_ok=True)
for n in ('official_wildos_beamng_bundle.zip','official_wildos_beamng_bundle.manifest.json','official_wildos_beamng_bundle.sha256'):
    target=candidate/n
    if target.exists():assert target.read_bytes()==(root/n).read_bytes()
    else:shutil.copy2(root/n,target)
with zipfile.ZipFile(candidate/'official_wildos_beamng_bundle.zip') as z:
    assert z.testzip() is None
    for n,h in manifest['members'].items():
        assert hashlib.sha256(z.read(n)).hexdigest()==h
        assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h
field=out/'field';field.mkdir(exist_ok=True)
for src in (root/'test_artifacts/visible_route_yaw90_20261017/field').iterdir():
    if src.suffix not in ('.py','.sh','.json','.yaml') or src.name=='harness_manifest.json':continue
    value=src.read_text(encoding='utf8').replace(yaw,remote).replace(previous,remote).replace(old_digest,manifest['sha256'])
    (field/src.name).write_text(value,encoding='utf8',newline='\n')
config=json.loads((field/'sensor_full_console.json').read_text(encoding='utf8'))
setups='\n'.join('source '+p for p in config['setups'])
run='#!/usr/bin/env bash\nset -eo pipefail\n'+setups+'\nexport ROS_DOMAIN_ID=88 DISPLAY=:148\n'
run+='python3 '+remote+'/field/verify_before.py\npython3 '+remote+'/field/drive_sensor_full.py search\n'
(out/'run_field.sh').write_text(run,encoding='utf8',newline='\n')
for name in ('analyze_source_closed.py','live_summary.py','review_field.py','inspect_closed_progress.py','seal_field.py','finalize_field.sh'):
    value=(root/'test_artifacts/source_control_repair_20261017'/name).read_text(encoding='utf8')
    value=value.replace(previous,remote).replace(old_digest,manifest['sha256']).replace('field_scope_baseline_search_*','field_yaw_left90_search_*')
    value=value.replace('source_control_field_evidence.zip','visible_pursuit_field_evidence.zip')
    if name=='seal_field.py':
        value=value.replace("'tools/server_test_preflight.py','tools/test_source_control_preflight.py'",
             "'tools/server_test_preflight.py','tools/test_source_control_preflight.py',\n             'nav2_adapter/wildos_nav2_adapter/visible_approach.py',\n             'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',\n             'official_adapter/beamng_wildos_adapter/goal_standoff.py',\n             'nav2_adapter/test/test_visible_approach_ros.py'")
    (out/name).write_text(value,encoding='utf8',newline='\n')
harness=json.loads((root/'test_artifacts/visible_route_yaw90_20261017/field/harness_manifest.json').read_text(encoding='utf8'))
harness['candidate_sha256']=manifest['sha256'];harness['condition']['navigation_candidate']=manifest['sha256'];harness['condition']['navigation_root']=remote
harness['condition']['scope']='Pursuit handoff repair, same Town position, left-90 spawn; physical vehicle unchanged'
harness['files']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in field.iterdir()
                  if p.name!='harness_manifest.json' and p.suffix in ('.py','.sh','.json','.yaml')}
(field/'harness_manifest.json').write_text(json.dumps(harness,indent=2),encoding='utf8')
condition=json.loads((field/'trial_condition.json').read_text(encoding='utf8'))
condition.update(harness['condition'])
(field/'trial_condition.json').write_text(json.dumps(condition,indent=2),encoding='utf8')
harness['files']['trial_condition.json']=hashlib.sha256((field/'trial_condition.json').read_bytes()).hexdigest()
(field/'harness_manifest.json').write_text(json.dumps(harness,indent=2),encoding='utf8')
with zipfile.ZipFile(out/'harness.zip','w',zipfile.ZIP_DEFLATED) as z:
    files=[p for p in field.iterdir() if p.suffix in ('.py','.sh','.json','.yaml')]
    files += [out/n for n in ('run_field.sh','analyze_source_closed.py','live_summary.py','review_field.py','inspect_closed_progress.py','seal_field.py','finalize_field.sh')]
    for p in files:
        if p.suffix=='.py':ast.parse(p.read_text(encoding='utf8'))
        z.write(p,p.relative_to(out).as_posix())
report=dict(candidate=manifest['sha256'],members=len(manifest['members']),changed=sorted(changed),
            baseline=old_digest,harness_sha256=hashlib.sha256((out/'harness.zip').read_bytes()).hexdigest(),
            scope='Frozen candidate, no field acceptance yet')
(out/'freeze.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps(report,indent=2))
