from pathlib import Path
import hashlib,json,zipfile,shutil
root=Path(__file__).resolve().parents[2];out=root/'test_artifacts/frontier_cost_repair_20261016'
checks=[]
for archive_name,metadata_name in (('field_evidence.zip','core_meta.json'),('component_evidence.zip','component_meta.json')):
 archive=out/archive_name;meta=json.loads((out/metadata_name).read_text())
 assert hashlib.sha256(archive.read_bytes()).hexdigest()==meta['sha256']
 with zipfile.ZipFile(archive) as z:
  assert z.testzip() is None
  manifest=json.loads(z.read('MANIFEST.json'))
  assert len(manifest)==meta['members']
  for name,sha in manifest.items():
   data=z.read(name);assert hashlib.sha256(data).hexdigest()==sha,name
   target=out/'server'/name
   assert target.resolve().is_relative_to(out.resolve())
   if name.endswith('/rviz_live.mp4'):
    local=root/'captures/20261016_frontier_cost_retest/rviz_live.mp4'
    assert hashlib.sha256(local.read_bytes()).hexdigest()==sha
   else:
    target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
 checks.append(dict(archive=archive_name,sha256=meta['sha256'],members=meta['members'],crc_ok=True,all_member_hashes_ok=True))
for name in ('cost_before.log','cost_global.log','cost_release_global.log','cost_epoch_before.log',
             'cost_epoch_after.log','cost_counterexample.py','inspect_recovery_closed.py'):
 shutil.copy2(Path(__file__).parent/name,out/name)
for name in ('report.json','report.md','callback_report.json'):
 shutil.copy2(root/'test_artifacts/offline_navigation'/name,out/name)
(out/'archive_validation.json').write_text(json.dumps(dict(archives=checks,video_matches_archive=True),indent=2))
print(json.dumps(checks,indent=2))
reviews=list((out/'server').rglob('closed_review.json'))
for path in reviews:
 report=json.loads(path.read_text())
 print(json.dumps({k:report[k] for k in ('status_counts','native_distance_m','max_departure_m','stationary_intervals_ge5s','retirement_reasons','vision_final')},indent=2))
