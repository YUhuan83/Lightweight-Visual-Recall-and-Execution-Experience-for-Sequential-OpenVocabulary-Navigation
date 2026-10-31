import ast,difflib,hashlib,json,zipfile
from pathlib import Path
root=Path(__file__).parent.parent/'拟真/test_artifacts/confirmed_progress_repair_20261025'
candidate=root/'candidate';base=json.loads((root/'base_hashes.json').read_text())
added=['nav2_adapter/test/test_confirmed_progress_ros.py','nav2_adapter/test/fixtures/standoff_first_snapshot.npz']
changed=[n for n,h in base.items() if hashlib.sha256((candidate/n).read_bytes()).hexdigest()!=h]+added
runtime=[n for n in changed if '/test/' not in n]
assert set(runtime)=={'official_adapter/beamng_wildos_adapter/goal_standoff.py','official_adapter/beamng_wildos_adapter/visible_approach.py'},runtime
reference=root.parent/'execution_handoff_repair_20261025/candidate'
diff=''.join(''.join(difflib.unified_diff((reference/n).read_text(encoding='utf8').splitlines(True),(candidate/n).read_text(encoding='utf8').splitlines(True),fromfile='v28/'+n,tofile='v31/'+n)) for n in runtime)
(root/'v28_to_v31.diff').write_text(diff,encoding='utf8')
for n in runtime:ast.parse((candidate/n).read_text(encoding='utf8'))
with zipfile.ZipFile(root/'confirmed_progress_delta.zip','w',zipfile.ZIP_DEFLATED) as archive:
    archive.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed if n in base},indent=2))
    for n in changed:archive.write(candidate/n,n)
report=dict(base='v28_intent_handoff',changed={n:hashlib.sha256((candidate/n).read_bytes()).hexdigest() for n in changed},runtime=runtime,
    preserved='v28 planning/controller/physics/localization/confirmation/arrival profiles; no v29/v30 runtime changes')
(root/'source_scope.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
