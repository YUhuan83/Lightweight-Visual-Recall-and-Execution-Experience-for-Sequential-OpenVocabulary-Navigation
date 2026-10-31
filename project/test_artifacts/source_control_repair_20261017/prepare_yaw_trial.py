"""Freeze a test-only left-90 spawn; reuse unchanged 2e69583c navigation."""
import ast
import hashlib
import json
import math
from pathlib import Path
import zipfile

root = Path(__file__).resolve().parents[2]
old = Path(__file__).resolve().parent
out = root/'test_artifacts/visible_route_yaw90_20261017'
out.mkdir(exist_ok=True)
field = out/'field';field.mkdir(exist_ok=True)
previous = '/home/ly/source_control_repair_20261017'
remote = '/home/ly/visible_route_yaw90_20261017'
manifest = json.loads((root/'deploy/20261017_source_control_preflight_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
assert manifest['sha256']=='2e69583ced4ba5be3445d6c1c565e28812441ff2f76f5e7a410117af0fcc82d0'
for src in (old/'field').iterdir():
    if src.suffix not in ('.py','.sh','.json','.yaml') or src.name=='harness_manifest.json':
        continue
    value=src.read_text(encoding='utf8').replace(previous,remote)
    value=value.replace(remote+'/candidate',previous+'/candidate').replace(remote+'/install',previous+'/install')
    if src.name=='verify_before.py':
        value=value.replace("(root/'official_wildos_beamng_bundle.manifest.json')",f"(Path('{previous}')/'official_wildos_beamng_bundle.manifest.json')")
        value=value.replace("(root/'candidate'",f"(Path('{previous}')/'candidate'")
        value=value.replace("(root/'candidate/official_adapter",f"(Path('{previous}')/'candidate/official_adapter")
    if src.name=='sensor_console.py':
        value=value.replace(" if role=='vision':", " if role=='scene':\n  argv=list(argv)\n  argv[1]='"+remote+"/field/scene_left90.py'\n if role=='vision':")
    if src.name=='drive_sensor_full.py':
        value=value.replace('import argparse,json,os,signal,subprocess,time','import argparse,json,math,os,signal,subprocess,time')
        value=value.replace("    if args.stage=='lifecycle':", "    directions=[r['data']['direction'] for r in report['source_lifecycle']['rows'] if r['kind']=='native']\n    headings=[math.atan2(d[1],d[0]) for d in directions]\n    expected="+repr(math.radians(-100))+"\n    assert headings and all(abs(math.remainder(h-expected,2*math.pi))<math.radians(3) for h in headings), 'left-90 spawn heading not confirmed'\n    report['spawn_heading_check']={'passed':True,'expected_degrees':-100,'observed_degrees':[math.degrees(h) for h in headings]}\n    if args.stage=='lifecycle':")
        value=value.replace("out=root/('field_scope_baseline_'", "out=root/('field_yaw_left90_'")
    if src.suffix=='.py':ast.parse(value)
    (field/src.name).write_text(value,encoding='utf8',newline='\n')
original=(root/'tools/wildos_console_scene.py').read_text(encoding='utf8')
old_z,old_w=.7658087977066911,.6430683364581344
quat=(0.,0.,(old_z-old_w)/math.sqrt(2),(old_z+old_w)/math.sqrt(2))
scene=original.replace('(0., 0., .7658087977066911, .6430683364581344)',repr(quat))
assert scene!=original
(field/'scene_left90.py').write_text(scene,encoding='utf8',newline='\n')
condition=dict(navigation_candidate=manifest['sha256'],navigation_root=previous,
    scope='Only scene spawn quaternion changes; production source bundle and installed navigation unchanged',
    spawn_position=[734.294,-11.946,52.154],spawn_quaternion=list(quat),
    yaw_change_left_degrees=90,expected_native_heading_degrees=-100,
    original_scene_sha256=hashlib.sha256(original.encode()).hexdigest())
(field/'trial_condition.json').write_text(json.dumps(condition,indent=2),encoding='utf8')
files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in field.iterdir()
       if p.name!='harness_manifest.json' and p.suffix in ('.py','.sh','.json','.yaml')}
harness=json.loads((old/'field/harness_manifest.json').read_text(encoding='utf8'))
harness.update(files=files,condition=condition)
(field/'harness_manifest.json').write_text(json.dumps(harness,indent=2),encoding='utf8')
config=json.loads((field/'sensor_full_console.json').read_text(encoding='utf8'))
run='#!/usr/bin/env bash\nset -eo pipefail\n'+'\n'.join('source '+p for p in config['setups'])+'\n'
run+='export ROS_DOMAIN_ID=88 DISPLAY=:148\n'
run+='python3 '+remote+'/field/verify_before.py\npython3 '+remote+'/field/drive_sensor_full.py search\n'
(out/'run_field.sh').write_text(run,encoding='utf8',newline='\n')
with zipfile.ZipFile(out/'harness.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in (*field.iterdir(),out/'run_field.sh'):
        if p.suffix in ('.py','.sh','.json','.yaml'):
            z.write(p,p.relative_to(out).as_posix())
print(json.dumps(condition,indent=2))
