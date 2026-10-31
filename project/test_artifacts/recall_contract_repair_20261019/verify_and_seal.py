"""Verify frozen source and component evidence; never infer full vehicle success."""
from pathlib import Path,PurePosixPath
import hashlib,json,re,tarfile,zipfile
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
remote=OUT/'server_evidence.tgz'
assert hashlib.sha256(remote.read_bytes()).hexdigest()=='feca3489f5c323a6ec5842c50dd18fcf97eaf50c6da934a2c39f8ead6224abe3'
destination=OUT/'server';assert not destination.exists();destination.mkdir()
with tarfile.open(remote) as archive:
 for member in archive.getmembers():
  relative=PurePosixPath(member.name)
  assert not relative.is_absolute() and '..' not in relative.parts and member.isfile()
  target=destination.joinpath(*relative.parts);assert target.resolve().is_relative_to(destination.resolve())
  target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(archive.extractfile(member).read())
manifest=json.loads((OUT/'v2/candidate_delta.json').read_text())
original=json.loads((ROOT/'official_wildos_beamng_bundle.manifest.json').read_text())
assert hashlib.sha256((ROOT/'official_wildos_beamng_bundle.zip').read_bytes()).hexdigest()==manifest['baseline_bundle']==original['sha256']
expected=dict(original['members']);expected.update(manifest['members'])
for name,want in expected.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==want,name
with zipfile.ZipFile(OUT/'v2/candidate_delta.zip') as archive:
 assert archive.testzip() is None
 for name,want in manifest['members'].items():assert hashlib.sha256(archive.read(name)).hexdigest()==want,name
v2=destination/'v2'
assert json.loads((v2/'candidate_delta.json').read_text())==manifest
for name,count in (('recall_tests.log',51),('ground_tests.log',19)):
 body=(v2/name).read_text()
 assert re.search(r'Ran '+str(count)+r' tests .*\n\nOK',body),name
ros=json.loads((v2/'candidate/test_artifacts/goal_release_repair_20261019/real_ros.json').read_text())
assert ros['passed'] and ros['tests']==60 and ros['failures']==ros['errors']==0
dds_paths=list(v2.glob('decision_dds_*/result.json'));assert len(dds_paths)==1
dds=json.loads(dds_paths[0].read_text());assert dds['passed'] and not dds['errors']
assert not dds['vehicle_started'] and not dds['complete_navigation_passed']
assert [s['state']['goal'] for s in dds['stages']]==[[20,0],[-20,0],[20,0],[-20,0],[20,0]]
assert json.loads((v2/'post_inventory.json').read_text())['project_processes']==[]
old=list(destination.glob('decision_dds_*/result.json'));assert len(old)==1
assert not json.loads(old[0].read_text())['passed']
for row in json.loads((OUT/'before_manifest.json').read_text()):
 assert hashlib.sha256((OUT/'before'/row['path']).read_bytes()).hexdigest()==row['sha256']
additional=['wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/execution_scene_samples.py',
 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test/test_scoring_execution_contract.py',
 'tools/offline_navigation_check.py','nav2_adapter/test/test_pointcloud.py']
with zipfile.ZipFile(ROOT/'official_wildos_beamng_bundle.zip') as archive:
 for name in additional:
  p=OUT/'baseline_additional'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(archive.read(name))
callbacks=json.loads((ROOT/'test_artifacts/offline_navigation/callback_report.json').read_text())
assert callbacks['passed'] and callbacks['tests']==570
comparison=json.loads((ROOT/'test_artifacts/passability_contract_20261019/candidate_comparison.json').read_text())
assert len(comparison['snapshots'])==22 and comparison['clearing_unchanged'] and not comparison['low_surface_loss_fixed']
assert all(all(v==0 for v in s['baseline_recorded_cell_differences'].values()) for s in comparison['snapshots'])
review=dict(evidence_consistent=True,source_members_verified=len(expected),recall_tests=51,ground_tests=19,
 offline_callbacks=570,real_ros_callbacks=60,dds_decisions=5,recorded_map_replays=22,
 physical_stall_repaired=False,field_acceptance=False,production_deployment=False)
(OUT/'evidence_review.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf8')
print(json.dumps(review,indent=2))
