from pathlib import Path
import difflib,hashlib,json,shutil,zipfile
out=Path(__file__).resolve().parent;root=out.parents[1]
meta=json.loads((out/'evidence_meta.json').read_text(encoding='utf8'))
archive=out/'component_evidence.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==meta['sha256']
with zipfile.ZipFile(archive) as z:
 assert z.testzip() is None
 manifest=json.loads(z.read('evidence_manifest.json'))
 assert set(z.namelist())==set(manifest)|{'evidence_manifest.json'}
 for n,m in manifest.items():
  assert not Path(n).is_absolute() and '..' not in Path(n).parts
  b=z.read(n);assert len(b)==m['size'] and hashlib.sha256(b).hexdigest()==m['sha256']
 z.extractall(out/'server')
delta=json.loads((out/'delta.json').read_text(encoding='utf8'))
bundle=json.loads((root/'deploy/20261016_exploration_scope_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
assert delta['candidate']==meta['candidate']==bundle['sha256']
diffs=out/'diffs';diffs.mkdir(exist_ok=True)
for name in delta['changed']:
 assert hashlib.sha256((root/name).read_bytes()).hexdigest()==bundle['members'][name]
 old=(out/'before'/name).read_text(encoding='utf8')
 new=(root/name).read_text(encoding='utf8')
 (diffs/(name.replace('/','__')+'.diff')).write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='e6f28118/'+name,tofile='23675e28/'+name)),encoding='utf8')
offline=json.loads((root/'test_artifacts/offline_navigation/report.json').read_text(encoding='utf8'))
assert not any(c['status']=='FAIL' for c in offline['checks'])
callbacks=json.loads((root/'test_artifacts/offline_navigation/callback_report.json').read_text(encoding='utf8'))
assert callbacks['passed'] and callbacks['tests']==446
shutil.copyfile(root/'test_artifacts/offline_navigation/report.json',out/'offline_report.json')
shutil.copyfile(root/'test_artifacts/offline_navigation/callback_report.json',out/'callback_report.json')
result=dict(candidate=meta['candidate'],changed=delta['changed'],archive_sha256=meta['sha256'],verified_members=len(manifest),
 offline_counts={s:sum(x['status']==s for x in offline['checks']) for s in ('PASS','FAIL','WARN')},
 callback_tests=callbacks['tests'],server=json.loads((out/'server/validation_identity.json').read_text(encoding='utf8')),
 field_validation='Not run for this candidate; production remains e6f28118; no success count or commit',
 replay=json.loads((out/'server/replay_summary.json').read_text(encoding='utf8')))
(out/'verification.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k not in ('server','replay','changed')},indent=2))
