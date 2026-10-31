"""Check source identity and component evidence; full vehicle acceptance stays false."""
from pathlib import Path,PurePosixPath
import hashlib,json,re,tarfile,xml.etree.ElementTree as E,zipfile
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
archive=OUT/'server_evidence.tgz'
digest=hashlib.sha256(archive.read_bytes()).hexdigest()
assert digest==(OUT/'server_evidence.sha256').read_text().split()[0]
assert digest=='43531230f6d446a07c927e28c7594c02c5787b6f1b1c87afc30f47f8a58d2a30'
destination=OUT/'server';assert not destination.exists();destination.mkdir()
seen=set()
with tarfile.open(archive) as package:
 for member in package.getmembers():
  relative=PurePosixPath(member.name)
  assert member.isfile() and not relative.is_absolute() and '..' not in relative.parts and '\\' not in member.name
  assert member.name not in seen;seen.add(member.name)
  p=destination.joinpath(*relative.parts);assert p.resolve().is_relative_to(destination.resolve())
  p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(package.extractfile(member).read())
manifest=json.loads((destination/'evidence_manifest.json').read_text())
assert seen==set(manifest)|{'evidence_manifest.json'}
for name,identity in manifest.items():
 p=destination/name
 assert p.stat().st_size==identity['bytes'] and hashlib.sha256(p.read_bytes()).hexdigest()==identity['sha256'],name
delta=json.loads((OUT/'candidate6/candidate_delta.json').read_text(encoding='utf8'))
assert json.loads((destination/'candidate6/candidate_delta.json').read_text())==delta
original=json.loads((ROOT/'official_wildos_beamng_bundle.manifest.json').read_text())
assert original['sha256']==delta['baseline_bundle']==hashlib.sha256((ROOT/'official_wildos_beamng_bundle.zip').read_bytes()).hexdigest()
expected=dict(original['members']);expected.update(delta['members'])
for name,want in expected.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==want,name
with zipfile.ZipFile(OUT/'candidate6/candidate_delta.zip') as package:
 assert package.testzip() is None
 for name,want in delta['members'].items():assert hashlib.sha256(package.read(name)).hexdigest()==want,name
def cpp(label,name,tests,failures):
 result=E.parse(destination/label/(name+'_results.xml')).getroot()
 assert int(result.attrib['tests'])==tests and int(result.attrib['failures'])==failures and int(result.attrib['errors'])==0
cpp('baseline','mapper',21,4);cpp('baseline','planner',38,2)
cpp('candidate2','mapper',21,2)
cpp('candidate3','mapper',21,0);cpp('candidate3','planner',38,0)
cpp('candidate5','planner',40,3)
cpp('candidate6','mapper',21,0);cpp('candidate6','planner',40,0)
def one(pattern):
 values=list(destination.glob(pattern));assert len(values)==1,(pattern,values)
 return json.loads(values[0].read_text())
baseline=one('baseline_dds_v2_*/result.json')
assert baseline['passed'] and all(s['state']['complete'] for s in baseline['stages'])
counter=one('candidate3_dds_v2_*/result.json')
assert not counter['passed'] and counter['stages'][0]['state']['complete'] is False
assert 'unknown_after_steep' in '\n'.join(counter['errors'])
chain=one('candidate6_dds_v2_*/result.json')
assert chain['passed'] and not chain['errors']
assert [s['state']['complete'] for s in chain['stages']]==[False,False,True,True,True,False,True]
held=chain['stages'][1]['edges']
assert any(e['observed_at']==[1,0] and e['properties'].get('physical_violation_observed')==[1.] for e in held)
assert any(e['observed_at']==[2,0] and e['confidence']==0. for e in held)
for pattern,count in [('candidate6/dds_*/result.json',15),('candidate6/decision_dds_*/result.json',5)]:
 d=one(pattern);assert d['passed'] and not d['errors'] and len(d['stages'])==count
for name,count in [('recall_tests.log',51),('ground_tests.log',19)]:
 body=(destination/'candidate6'/name).read_text()
 assert re.search(r'Ran '+str(count)+r' tests .*\n\nOK',body),name
ros=json.loads((destination/'candidate6/candidate/test_artifacts/goal_release_repair_20261019/real_ros.json').read_text())
assert ros['passed'] and ros['tests']==60 and ros['failures']==ros['errors']==0
replay=one('replay_*/result.json');assert replay['completed'] and not replay['errors'] and len(replay['snapshots'])==22
assert replay['source_database']['sha256']=='0a1234726e95c0369bf11446701e2bb1ac16e3a92c4e397feea54d3419177dbc'
assert json.loads((destination/'post_inventory.json').read_text())['project_processes']==[]
callbacks=json.loads((ROOT/'test_artifacts/offline_navigation/callback_report.json').read_text())
assert callbacks['passed'] and callbacks['tests']==570
summary=dict(evidence_consistent=True,source_members_verified=len(expected),server_payloads_verified=len(manifest),
    cpp_tests=61,physical_dds_stages=7,goal_release_dds_stages=15,recall_dds_stages=5,
    real_ros_callbacks=60,memory_tests=51,ground_tests=19,offline_callbacks=570,recorded_maps=22,
    counterexample_preserved=True,vehicle_acceptance=False,production_deployment=False,
    low_surface_filter_repaired=False,terrain_step_metric_repaired=False)
(OUT/'evidence_review.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8')
print(json.dumps(summary,indent=2))
