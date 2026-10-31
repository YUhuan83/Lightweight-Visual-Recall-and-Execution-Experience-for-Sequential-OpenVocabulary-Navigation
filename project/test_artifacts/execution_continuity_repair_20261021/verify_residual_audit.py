"""Check stop accounting and provenance; known failing handoff cases stay open."""
from pathlib import Path
import hashlib
import json
import zipfile

r=Path(__file__).resolve().parent
p=r/'residual_evidence.zip'
assert p.stat().st_size==21462
assert hashlib.sha256(p.read_bytes()).hexdigest()=='03435d2bf2eca3670e8600f4837db9995032fb699d66feaa07b3ea1f593dd957'
old=zipfile.ZipFile(r/'execution_evidence.zip')
references={v['path']:v['sha256'] for v in json.loads(old.read('evidence_manifest.json'))['bag_references']}
with zipfile.ZipFile(p) as z:
    assert z.testzip() is None
    manifest=json.loads(z.read('residual_manifest.json'))
    for name,digest in manifest.items():
        assert hashlib.sha256(z.read(name)).hexdigest()==digest,name
    results={}
    for version,trial in [('v3','084227'),('v4','085212')]:
        s=json.loads(z.read(f'residual_audit/{version}/residual_stops.json'))
        previous=json.loads(old.read(f'{version}/full/trial_20261021_{trial}/field_summary.json'))
        assert references[s['bag']['path']]==s['bag']['sha256']
        assert abs(s['stationary_s']-120*previous['stationary_fraction'])<1e-5
        assert abs(sum(s['stationary_by_control'].values())-s['stationary_s'])<1e-6
        assert abs(sum(s['stationary_by_plan'].values())-s['stationary_s'])<1e-6
        assert abs(sum(v['seconds'] for v in s['stationary_cross'])-s['stationary_s'])<1e-6
        assert all(v['duration_s']>=3 and v['net_m']<1. for v in s['stops_over_3s'])
        results[version]=dict(stationary_s=s['stationary_s'],by_control=s['stationary_by_control'])
    detail=z.read('selection_detail.log').decode()
    assert 'PATH 100.0557746887207' in detail and 'PATH 104.99516892433167' in detail
    selected=json.loads(z.read('residual_audit/v4/selection_wait.json'))
    race=next(v for v in selected if 100<v['choice_s']<101)
    assert race['ack'] is None
    # Keep a small readable copy; the sealed archive remains the source of truth.
    for version in ('v3','v4'):
        (r/f'residual_stops_{version}.json').write_bytes(z.read(f'residual_audit/{version}/residual_stops.json'))
counter=(r/'residual_owner_counterexamples.log').read_text(errors='replace')
assert '"tests": 2, "failures": 2, "errors": 0' in counter
report=dict(evidence_integrity_passed=True,results=results,
            new_counterexample_failures=2,production_changed=False,
            fixed=False,complete_navigation_passed=False)
(r/'residual_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
