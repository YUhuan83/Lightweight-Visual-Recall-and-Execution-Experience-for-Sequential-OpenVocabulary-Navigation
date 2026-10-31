"""Read only bounded tails of active JSONL/log files, never active databases."""
import json,time,collections
from pathlib import Path
root=Path('/home/ly/source_control_repair_20261017')
trial=sorted(root.glob('field_scope_baseline_search_*'))[-1]
identity=[p for p in (root/'field_results').glob('*/console_identity.json')
    if json.loads(p.read_text())['pid']==int((trial/'driver_child.pid').read_text())]
def tail(path,size=3000):
    if not path.exists():return ''
    with path.open('rb') as f:
        f.seek(max(0,path.stat().st_size-size));return f.read().decode(errors='replace')
report=dict(trial=str(trial),closed=(trial/'result.json').exists(),console=tail(trial/'console.log',1200))
if identity:
    s=identity[0].parent;g=s/'goal_001';report['session']=str(s)
    report['last_logs']={name:tail(s/name,700) for name in ('scene.log','physical.log','vision.log','preflight.log','execution_observer.log') if (s/name).exists()}
    samples=[]
    for line in tail(g/'native_samples.jsonl',500_000).splitlines():
        try:samples.append(json.loads(line))
        except ValueError:pass
    if samples:
        recent=samples[-100:];last=recent[-1]
        report['native']=dict(samples=len(recent),source_age_max=max(x.get('command_age',0) for x in recent),
            receipt_age_max=max(x.get('command_receipt_age') or 0 for x in recent),
            source_receipt_valid=sum(bool((x.get('source_receipt') or {}).get('applied')) for x in recent),
            watchdog_stops=sum(bool(x.get('watchdog_stop')) for x in recent),
            speed=last.get('speed'),last_sent=last.get('sent'),
            telemetry_ok=last.get('telemetry_ok'),actuator_ok=last.get('actuator_ok'),
            source_rejection=last.get('source_rejection'),last_wall=last['wall_time'])
    report['probe']={k:v for k,v in json.loads((trial/'source_lifecycle.json').read_text()).items() if k!='rows'} if (trial/'source_lifecycle.json').exists() else None
    report['goal']=json.loads((g/'result.json').read_text()) if (g/'result.json').exists() else None
print(json.dumps(report,indent=2))
