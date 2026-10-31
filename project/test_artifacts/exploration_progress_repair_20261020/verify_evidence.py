"""Validate the sealed closed trials and frozen pre-repair source against this workspace."""
from pathlib import Path
import hashlib
import json
import zipfile

root=Path(__file__).resolve().parent;repo=root.parents[1]
archive=root/'sealed/exploration_evidence.zip';identity=json.loads((root/'sealed/archive.json').read_text())
assert archive.stat().st_size==identity['size']
assert hashlib.sha256(archive.read_bytes()).hexdigest()==identity['sha256']
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    manifest=json.loads(z.read('evidence_manifest.json'))
    for name,item in manifest['files'].items():
        data=z.read(name);assert len(data)==item['size'] and hashlib.sha256(data).hexdigest()==item['sha256'],name
    frozen=json.loads(z.read('full/frozen_source_manifest.json'))
    replacements={
        'wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper/src/mapper.cpp':repo/'test_artifacts/graph_start_attachment_repair_20261020/mapper.before.cpp',
        'wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper/test/test_mapper.cpp':repo/'test_artifacts/graph_start_attachment_repair_20261020/test_mapper.before.cpp',
    }
    for name,digest in frozen['members'].items():
        assert hashlib.sha256(replacements.get(name,repo/name).read_bytes()).hexdigest()==digest,name
    cpp=json.loads(z.read('full/captured_planner_result.json'))
    original=[r for r in cpp['summary'] if r['original']];assert len(original)==1 and original[0]['targets']==30 and original[0]['nonempty']==0
    assert any(not r['original'] and r['nonempty']==30 for r in cpp['summary'])
    graph=json.loads(z.read('full/graph_analysis_final.json'));assert len(graph['snapshots'])==101 and not graph['observer_errors']
    terrain=json.loads(z.read('full/terrain_start_review/result.json'))
    assert all(r['terrain_ns']==r['transform_ns']==r['physical_observed_ns'] for r in terrain['results'])
    report=dict(payloads=len(manifest['files']),source_members=len(frozen['members']),
                archive_sha256=identity['sha256'],cpp_original_start_targets=30,cpp_original_nonempty=0,
                complete_navigation_passed=False,verified=True)
(root/'local_evidence_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
