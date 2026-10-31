"""Freeze this offline candidate and retain the exact tested source delta."""
import difflib
import ast
import hashlib
import json
from pathlib import Path
import shutil
import zipfile
import sys
from collections import Counter

out=Path(__file__).resolve().parent
root=out.parents[1]
baseline=root/'deploy/20261016_frontier_motion_candidate'
candidate=root/'deploy'/(sys.argv[1] if len(sys.argv)>1 else '20261017_source_control_candidate')
bundle='official_wildos_beamng_bundle.zip'
old=json.loads((baseline/'official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
new=json.loads((root/'official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
digest=lambda b:hashlib.sha256(b).hexdigest()
assert old['sha256']=='12023042037205a79a57f0e76bb1d8544e94309f8ff9b80a3fe28f554f521f2d'
assert digest((baseline/bundle).read_bytes())==old['sha256']
assert digest((root/bundle).read_bytes())==new['sha256']
with zipfile.ZipFile(baseline/bundle) as z:
    assert z.testzip() is None
    for name,sha in old['members'].items():assert digest(z.read(name))==sha,name
    changed={n:dict(before=old['members'][n],after=sha)
             for n,sha in new['members'].items() if n in old['members'] and sha!=old['members'][n]}
    added={n:sha for n,sha in new['members'].items() if n not in old['members']}
    removed={n:sha for n,sha in old['members'].items() if n not in new['members']}
    for name in changed:
        before=out/'before'/name;before.parent.mkdir(parents=True,exist_ok=True)
        if before.exists():assert before.read_bytes()==z.read(name),name
        else:before.write_bytes(z.read(name))
        after=root/name
        diff=''.join(difflib.unified_diff(z.read(name).decode('utf-8-sig').replace('\r\n','\n').splitlines(True),
            after.read_text(encoding='utf-8-sig').splitlines(True),fromfile='before/'+name,tofile='after/'+name))
        target=out/'diffs'/(name.replace('/','__')+'.diff')
        target.parent.mkdir(parents=True,exist_ok=True);target.write_text(diff,encoding='utf8')
with zipfile.ZipFile(root/bundle) as z:
    assert z.testzip() is None
    assert set(z.namelist())==set(new['members'])
    for name,sha in new['members'].items():
        assert digest(z.read(name))==digest((root/name).read_bytes())==sha,name
        if name.endswith('.py'):
            ast.parse(z.read(name).decode('utf-8-sig'),filename=name)
for name in changed|added:
    target=out/'after'/name;target.parent.mkdir(parents=True,exist_ok=True)
    target.write_bytes((root/name).read_bytes())
protected=[n for n in old['members'] if n.startswith(('ackermann_rpp/','ackermann_smac/',
    'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/',
    'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/'))
    or n in ('nav2_adapter/config/nav2_ackermann_control.yaml',
             'nav2_adapter/wildos_nav2_adapter/vehicle_control_lease.lua',
             'nav2_adapter/wildos_nav2_adapter/vehicle_control_lease.py',
             'nav2_adapter/wildos_nav2_adapter/drive_math.py')]
for name in protected:assert new['members'][name]==old['members'][name],name
candidate.mkdir(exist_ok=True)
for name in (bundle,'official_wildos_beamng_bundle.manifest.json','official_wildos_beamng_bundle.sha256'):
    target=candidate/name
    if target.exists():assert target.read_bytes()==(root/name).read_bytes(),name
    else:shutil.copyfile(root/name,target)
for name in ('report.json','report.md','callback_report.json','test_source_control.log',
             'test_runtime_contract.log','test_package_official_bundle.log','test_dds_profile.log'):
    shutil.copyfile(root/'test_artifacts/offline_navigation'/name,out/name)
callbacks=json.loads((out/'callback_report.json').read_text(encoding='utf8'))
global_report=json.loads((out/'report.json').read_text(encoding='utf8'))
summary=dict(baseline=old['sha256'],candidate=new['sha256'],members=len(new['members']),
    modified=changed,added_to_archive=added,removed_generated_members=removed,
    archive_crc_and_member_sha_verified=True,current_sources_match_archive=True,
    unchanged_protected_source_count=len(protected),
    callback_tests=callbacks['tests'],callback_passed=callbacks['passed'],
    global_checks=dict(Counter(x['status'] for x in global_report['checks'])),
    failed_checks=[x for x in global_report['checks'] if x['status']=='FAIL'],
    scope='Offline source/protocol and callback doubles only; no ROS/DDS or BeamNG execution',
    field_acceptance=False,server_deployed=False,git_commit_created=False,
    pending=['Windows sandbox DDS strict-path 4 errors',
             'Real ROS launch expansion, message delivery and old/new deployment rejection',
             'Measured command/RPC/physical braking latency and cusp execution',
             'Bounded missing-frontier re-association and overall exploration progress',
             'Full green-house search; 20-trial and multi-goal acceptance'])
(out/('server_candidate_summary.json' if len(sys.argv)>1 else 'summary.json')).write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:summary[k] for k in ('candidate','members','callback_tests','callback_passed','global_checks')},indent=2))
print('Modified:',len(changed),'Added to archive:',len(added),'Removed generated:',len(removed))
