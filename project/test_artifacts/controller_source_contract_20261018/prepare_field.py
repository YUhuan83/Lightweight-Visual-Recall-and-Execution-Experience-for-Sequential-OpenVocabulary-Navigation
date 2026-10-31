"""Clone the closed harness into a new immutable candidate, never edit old evidence."""
from pathlib import Path
import hashlib,json,shutil,zipfile
ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'test_artifacts/motion_handoff_field_20261018'
OUT=ROOT/'test_artifacts/controller_source_field_20261018'
OUT.mkdir(exist_ok=False)
PREVIOUS='/home/ly/motion_handoff_field_20261018'
REMOTE='/home/ly/controller_source_field_20261018'
COMPONENT='/home/ly/controller_source_contract_20261018'
manifest=json.loads((ROOT/'official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
old=json.loads((ROOT/'deploy/20261018_motion_handoff_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
digest=manifest['sha256']
deploy=ROOT/'deploy/20261018_controller_source_candidate';deploy.mkdir(exist_ok=False)
for suffix in ('.zip','.manifest.json','.sha256'):
    shutil.copy2(ROOT/('official_wildos_beamng_bundle'+suffix),deploy)
field=OUT/'field';field.mkdir()
for p in (OLD/'field').iterdir():
    if not p.is_file() or p.name=='harness_manifest.json' or p.suffix not in ('.py','.sh','.json','.yaml'):continue
    text=p.read_text(encoding='utf8').replace(PREVIOUS,REMOTE).replace(old['sha256'],digest)
    if p.name=='sensor_full_console.sh':
        text=text.replace('source '+REMOTE+'/install/setup.bash',
            'source '+PREVIOUS+'/install/setup.bash\nsource '+COMPONENT+'/install/setup.bash\nsource '+REMOTE+'/install/setup.bash')
    if p.name=='record_execution_contract.py':
        text=text.replace('Twist,PointStamped','Twist,TwistStamped,PointStamped')
        text=text.replace("(Twist,'/wildos/cmd_vel'),", "(Twist,'/wildos/cmd_vel'),(TwistStamped,'/wildos/cmd_vel_stamped'),")
    if p.name=='progress.py':text=text.replace('field_scope_baseline_search_*','field_yaw_left90_search_*')
    (field/p.name).write_bytes(text.encode())
config=json.loads((field/'sensor_full_console.json').read_text(encoding='utf8'))
config['setups'][-1:]=[PREVIOUS+'/install/setup.bash',COMPONENT+'/install/setup.bash',REMOTE+'/install/setup.bash']
config['library_sha256']['libwildos_ackermann_rpp.so']='d2fd9cee5ecabb49e8be06a24079813eb46a9e77872ef6982d1ed547bb04d846'
(field/'sensor_full_console.json').write_text(json.dumps(config,indent=2)+'\n',encoding='utf8')
h=json.loads((OLD/'field/harness_manifest.json').read_text(encoding='utf8'))
h['candidate_sha256']=digest
h['condition'].update(navigation_candidate=digest,navigation_root=REMOTE,
    scope='Stamped RPP source lifetime and terminal zero safety; common execution baseline; three memory priors disabled')
h['files']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in field.iterdir() if p.is_file()}
(field/'harness_manifest.json').write_text(json.dumps(h,indent=2)+'\n',encoding='utf8')
names=('installed_component.py','run_field.sh','analyze_closed_bound.py','review_field.py',
    'analyze_source_closed.py','review_pursuit_closed.py','motion_metrics.py','verify_video.py','seal_field.py','finalize.sh')
for name in names:
    text=(OLD/name).read_text(encoding='utf8').replace(PREVIOUS,REMOTE).replace(old['sha256'],digest)
    if name=='run_field.sh':text=text.replace('source '+REMOTE+'/install/setup.bash',
        'source '+PREVIOUS+'/install/setup.bash\nsource '+COMPONENT+'/install/setup.bash\nsource '+REMOTE+'/install/setup.bash')
    if name=='installed_component.py':text=text.replace("'test_motion_intent_handoff_ros.MotionIntentHandoffTests'",
        "'test_motion_intent_handoff_ros.MotionIntentHandoffTests', 'test_controller_source_ros.ControllerSourceTests'")
    if name=='seal_field.py':text=text.replace("'ackermann_smac/src/planner.cpp',", "'ackermann_rpp/src/controller.cpp', 'nav2_adapter/test/test_controller_source_ros.py', 'ackermann_smac/src/planner.cpp',")
    (OUT/name).write_bytes(text.encode())
(OUT/'deploy_build.sh').write_bytes(('''set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source '''+PREVIOUS+'''/install/setup.bash
source '''+COMPONENT+'''/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=179
cd '''+REMOTE+'''
python3 -m zipfile -e official_wildos_beamng_bundle.zip candidate
python3 -m zipfile -e harness.zip .
cp official_wildos_beamng_bundle.manifest.json manifest.json
python3 - <<'PY'
import hashlib,json
from pathlib import Path
old=Path("'''+COMPONENT+'''")
for n in ('ackermann_rpp/src/controller.cpp','ackermann_rpp/CMakeLists.txt','ackermann_rpp/package.xml'):
 assert (Path('candidate')/n).read_bytes()==(old/'candidate'/n).read_bytes(),n
lib=old/'install/wildos_ackermann_rpp/lib/libwildos_ackermann_rpp.so'
assert hashlib.sha256(lib.read_bytes()).hexdigest()=='d2fd9cee5ecabb49e8be06a24079813eb46a9e77872ef6982d1ed547bb04d846'
PY
colcon --log-base build_log build --base-paths candidate/nav2_adapter --build-base build --install-base install >build.log 2>&1
source install/setup.bash
python3 installed_component.py >installed_component.log 2>&1
python3 field/verify_before.py >prelaunch_verification.json
''').encode())
with zipfile.ZipFile(OUT/'harness.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(field.iterdir()):z.write(p,'field/'+p.name)
    for name in (*names,'deploy_build.sh'):z.write(OUT/name,name)
summary=dict(candidate=digest,members=len(manifest['members']),remote=REMOTE,
    inherited_rpp_install=COMPONENT+'/install',
    changes_from_94=[n for n in sorted(manifest['members']) if manifest['members'][n]!=old['members'].get(n)],
    harness_sha256=hashlib.sha256((OUT/'harness.zip').read_bytes()).hexdigest(),field_acceptance=False)
(OUT/'freeze.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8')
print(json.dumps(summary,indent=2))
