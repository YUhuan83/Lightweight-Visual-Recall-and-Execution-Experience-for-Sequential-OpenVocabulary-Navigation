"""Freeze one bounded field candidate and bind the established test harness."""
from pathlib import Path
import sys, json, hashlib, zipfile

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).parent
OLD=ROOT/'test_artifacts/full_vehicle_acceptance_20261017'
REMOTE='/home/ly/motion_handoff_field_20261018'
PREVIOUS='/home/ly/full_vehicle_acceptance_20261017'
sys.path.insert(0,str(ROOT/'tools'))
from package_official_bundle import write_bundle

old=json.loads((ROOT/'deploy/20261017_full_vehicle_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
names=set(old['members'])|{'nav2_adapter/test/test_motion_intent_handoff_ros.py'}
deploy=ROOT/'deploy/20261018_motion_handoff_candidate'
deploy.mkdir(parents=True,exist_ok=True)
manifest=write_bundle(ROOT,deploy/'official_wildos_beamng_bundle.zip',names)
digest=manifest['sha256']
changes=[n for n in sorted(names) if manifest['members'][n]!=old['members'].get(n)]
assert set(changes)=={'nav2_adapter/wildos_nav2_adapter/continuous_path.py',
    'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
    'nav2_adapter/test/test_graph_path_bridge_routing.py',
    'nav2_adapter/test/test_motion_intent_handoff_ros.py','tools/offline_callback_check.py'},changes

field=OUT/'field';field.mkdir(exist_ok=True)
for p in (OLD/'field').iterdir():
    if not p.is_file() or p.name=='harness_manifest.json' or p.suffix not in ('.py','.sh','.json','.yaml'):
        continue
    content=p.read_text(encoding='utf8').replace(PREVIOUS,REMOTE).replace(old['sha256'],digest)
    if p.name=='sensor_full_console.sh':
        content=content.replace('source '+REMOTE+'/install/setup.bash',
            'source '+PREVIOUS+'/install/setup.bash\nsource '+REMOTE+'/install/setup.bash')
    (field/p.name).write_bytes(content.encode('utf8'))
config=json.loads((field/'sensor_full_console.json').read_text(encoding='utf8'))
config['setups'][-1:]=[PREVIOUS+'/install/setup.bash',REMOTE+'/install/setup.bash']
config['library_sha256']['libwildos_ackermann_smac.so']='3cd81492caf0df265fd9e635e528d1e876efa1e540e5fa94b9e05f09d2dfefd5'
(field/'sensor_full_console.json').write_text(json.dumps(config,indent=2)+'\n',encoding='utf8')
h=json.loads((OLD/'field/harness_manifest.json').read_text(encoding='utf8'))
h['candidate_sha256']=digest
h['condition'].update(navigation_candidate=digest,navigation_root=REMOTE,
    scope='Retired motion/intention separation and current-pose prefix checks; single-target common execution baseline')
h['files']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in field.iterdir()
    if p.is_file() and p.suffix in ('.py','.sh','.json','.yaml') and p.name!='harness_manifest.json'}
(field/'harness_manifest.json').write_text(json.dumps(h,indent=2)+'\n',encoding='utf8')

scripts=('installed_component.py','run_field.sh','analyze_closed_bound.py','review_field.py',
    'analyze_source_closed.py','review_pursuit_closed.py','motion_metrics.py','verify_video.py',
    'seal_field.py','finalize.sh')
for name in scripts:
    content=(OLD/name).read_text(encoding='utf8').replace(PREVIOUS,REMOTE).replace(old['sha256'],digest)
    if name=='run_field.sh':
        content=content.replace('source '+REMOTE+'/install/setup.bash',
            'source '+PREVIOUS+'/install/setup.bash\nsource '+REMOTE+'/install/setup.bash')
    if name=='installed_component.py':
        content=content.replace("'test_tracking_handoff_ros.TrackingHandoffTests'",
            "'test_tracking_handoff_ros.TrackingHandoffTests',\n        'test_motion_intent_handoff_ros.MotionIntentHandoffTests'")
    (OUT/name).write_bytes(content.encode('utf8'))
(OUT/'deploy_build.sh').write_bytes(('''set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source '''+PREVIOUS+'''/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=179
cd '''+REMOTE+'''
python3 -m zipfile -e official_wildos_beamng_bundle.zip candidate
python3 -m zipfile -e harness.zip .
cp official_wildos_beamng_bundle.manifest.json manifest.json
colcon --log-base build_log build --base-paths candidate/nav2_adapter --build-base build --install-base install >build.log 2>&1
source install/setup.bash
python3 installed_component.py >installed_component.log 2>&1
python3 field/verify_before.py >prelaunch_verification.json
''').encode())
with zipfile.ZipFile(OUT/'harness.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(field.iterdir()):
        if p.is_file():z.write(p,'field/'+p.name)
    for name in (*scripts,'deploy_build.sh'):z.write(OUT/name,name)
summary=dict(candidate=digest,members=len(names),changes_from_932=changes,
    remote=REMOTE,local_deploy=str(deploy),harness_sha256=hashlib.sha256((OUT/'harness.zip').read_bytes()).hexdigest(),
    field_acceptance=False)
(OUT/'freeze.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8')
print(json.dumps(summary,indent=2))
