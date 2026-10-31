"""Seal this review without changing prior closed evidence archives."""
import difflib
import hashlib
import json
from pathlib import Path
import zipfile

root = Path(__file__).resolve().parents[2]
out = Path(__file__).parent
old = root/'test_artifacts/visible_pursuit_repair_20261017'
production = 'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
baseline = root/'deploy/20261017_visible_pursuit_candidate/official_wildos_beamng_bundle.zip'
with zipfile.ZipFile(baseline) as archive:
    assert archive.read(production) == (out/'before/graph_path_bridge.py').read_bytes()
diff = ''.join(difflib.unified_diff(
    (out/'before/graph_path_bridge.py').read_text(encoding='utf8').splitlines(True),
    (root/production).read_text(encoding='utf8').splitlines(True),
    fromfile='f9d1caa7/'+production, tofile='candidate/'+production))
(out/'graph_path_bridge.diff').write_text(diff, encoding='utf8')
for name in ('nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
             'nav2_adapter/test/test_execution_progress_ros.py',
             'nav2_adapter/test/test_tracking_handoff_ros.py',
             'tools/offline_callback_check.py'):
    target = out/'after'/name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes((root/name).read_bytes())
for name in ('audit_execution_loop.py', 'execution_loop_review.json',
             'reproduce_progress_budget.py', 'reproduce_progress_budget.json'):
    target = out/'closed_review'/name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes((old/name).read_bytes())
paths = [p for p in out.rglob('*') if p.is_file() and '__pycache__' not in p.parts
         and p.suffix != '.zip' and p.name not in ('supplement_validation.json',)]
paths += [root/'docs/EXECUTION_PROGRESS_REPAIR_20261017.md']
members = {}
archive_path = out/'execution_review_supplement.zip'
with zipfile.ZipFile(archive_path, 'w', zipfile.ZIP_DEFLATED) as archive:
    for p in sorted(paths):
        name = str(p.relative_to(root)).replace('\\', '/')
        data = p.read_bytes()
        archive.writestr(name, data)
        members[name] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
    archive.writestr('manifest.json', json.dumps(members, indent=2))
with zipfile.ZipFile(archive_path) as archive:
    assert archive.testzip() is None
    for name, entry in members.items():
        assert hashlib.sha256(archive.read(name)).hexdigest() == entry['sha256']
report = dict(baseline_source_equal=True, members=len(members), crc=True, member_sha256=True,
    sha256=hashlib.sha256(archive_path.read_bytes()).hexdigest(), bytes=archive_path.stat().st_size,
    changed_production_files=[production], field_retest=False, deployed=False, committed=False)
(out/'supplement_validation.json').write_text(json.dumps(report, indent=2), encoding='utf8')
print(json.dumps(report, indent=2))
