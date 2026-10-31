"""Closed moving field comparison, not a claim that reliable/BE is causal."""
import collections,json,statistics
from pathlib import Path
out=Path('/home/ly/execution_wait_audit_20261018/startup_receipt')
root=Path('/home/ly/planning_phase_physical_20261018')
driver=root/'field_yaw_left90_search_20261018_151916'
assert (driver/'result.json').is_file()
d=json.loads((driver/'result.json').read_text());session=Path(d['session'])
assert (session/'session_result.json').is_file() and (session/'goal_001/result.json').is_file()
lo=next(c['wall'] for c in d['commands'] if c['text']=='green residential house')
hi=lo+json.loads((session/'goal_001/result.json').read_text())['elapsed_s']
p=next(out.glob('search_shadow_*.jsonl'));rows=[json.loads(l) for l in p.read_text().splitlines()]
def stats(a):
    a=sorted(a)
    return dict(n=len(a),min=a[0],median=statistics.median(a),p95=a[int(.95*len(a))],max=a[-1]) if a else None
def key(v):return tuple(v[k] for k in ('sender_id','generation','sequence'))
sources={key(r['data']):r['data'] for r in rows if r['kind']=='best1'}
native=[r['data'] for r in rows if r['kind']=='native' and lo<=r['wall_ns']/1e9<=hi]
source_readers={kind:{key(r['data']):r for r in rows if r['kind']==kind} for kind in ('reliable1','reliable10','best1')}
paired=[]
for v in native:
    if v['command_receipt_age'] is None:continue
    receipt=v.get('source_receipt')
    frame=sources.get(key(receipt)) if receipt else None
    inferred=False
    if frame is None:
        issued=v['wall_time']-v['command_age']
        frame=min(sources.values(),key=lambda f:abs(f['issued_wall_ns']/1e9-issued))
        if abs(frame['issued_wall_ns']/1e9-issued)>.001:continue
        inferred=True
    age=v['command_age']-v['command_receipt_age']
    item=dict(native_wall=v['wall_time'],identity=key(frame),callback_source_age=age,
              inferred_identity=inferred,reader_ages={})
    for kind,lookup in source_readers.items():
        observed=lookup.get(key(frame))
        if observed:item['reader_ages'][kind]=(observed['wall_ns']-frame['issued_wall_ns'])/1e9
    paired.append(item)
report=dict(scope=__doc__,session=str(session),counts=dict(collections.Counter(r['kind'] for r in rows)),
    reader_source_age={kind:stats([(r['wall_ns']-r['data']['issued_wall_ns'])/1e9 for r in lookup.values() if lo<=r['wall_ns']/1e9<=hi]) for kind,lookup in source_readers.items()},
    native_callback_source_age=stats([p['callback_source_age'] for p in paired]),
    watchdog_samples=sum(v['watchdog_stop'] for v in native),native_samples=len(native),
    missing_receipt=sum(v.get('source_receipt') is None for v in native),
    rejection_counts=dict(collections.Counter(str(v.get('source_rejection')) for v in native)),
    late_native_over_point2=[p for p in paired if p['callback_source_age']>.2],
    limits='Additional subscribers share one diagnostic participant; not the Native participant. No direct Native rmw receive timestamp recorded.')
(out/'search_shadow_review.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='late_native_over_point2'},indent=2))
print('Late Native samples',len(report['late_native_over_point2']))
