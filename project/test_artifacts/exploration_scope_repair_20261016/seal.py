from pathlib import Path
import hashlib,json,zipfile
r=Path('/home/ly/exploration_scope_repair_20261016')
required=['baseline.sh','baseline_configure.log','baseline_build.log','cpp_before.log',
 'delta.json','official_wildos_beamng_bundle.manifest.json','validate_candidate.sh',
 'build_validation.log','cpp_after.log','ros_after.log','dds_after.log','validation_identity.json',
 'test_scope_dds.py','scope_dds.log','validate_replay.py','replay.sh','replay_summary.json',
 'replay_after.log','replay_after_stderr.log','replay_configure.log','replay_build.log']
required+=['baseline/CMakeLists.txt','baseline/test_route_mechanism.cpp',
           'replay/CMakeLists.txt','replay/backoff_replay.cpp']
for name in required:assert (r/name).is_file(),name
assert '100% tests passed' in (r/'cpp_after.log').read_text()
assert 'Ran 50 tests' in (r/'ros_after.log').read_text() and (r/'ros_after.log').read_text().rstrip().endswith('OK')
assert 'Ran 9 tests' in (r/'dds_after.log').read_text() and (r/'dds_after.log').read_text().rstrip().endswith('OK')
assert 'Ran 2 tests' in (r/'scope_dds.log').read_text() and (r/'scope_dds.log').read_text().rstrip().endswith('OK')
manifest={name:dict(size=(r/name).stat().st_size,sha256=hashlib.sha256((r/name).read_bytes()).hexdigest()) for name in required}
(r/'evidence_manifest.json').write_text(json.dumps(manifest,indent=2))
with zipfile.ZipFile(r/'component_evidence.zip','w',zipfile.ZIP_DEFLATED) as z:
 for name in required+['evidence_manifest.json']:z.write(r/name,name)
meta=dict(sha256=hashlib.sha256((r/'component_evidence.zip').read_bytes()).hexdigest(),members=len(required)+1,
    candidate=json.loads((r/'validation_identity.json').read_text())['candidate'])
(r/'evidence_meta.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta,indent=2))
