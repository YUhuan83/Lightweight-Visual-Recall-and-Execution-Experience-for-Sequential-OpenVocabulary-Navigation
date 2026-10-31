"""Seal a closed, brake-only diagnostic; never label it navigation acceptance."""
from pathlib import Path
import hashlib
import json
import math
import sys
import zipfile

run = Path(sys.argv[1])
driver = json.loads((run/'result.json').read_text())
session = Path(driver['session'])
result = json.loads((session/'session_result.json').read_text())
assert driver['component_closed'] and driver.get('brake_only_idle')
assert result['goals'] == 0 and result['errors'] == []
native = []; control = []; sources = []
for line in (session/'execution_observer/messages.jsonl').open():
    row = json.loads(line)
    if row['topic'] == '/wildos/native_control_status': native.append(json.loads(row['message']['data']))
    elif row['topic'] == '/wildos/control_status': control.append(json.loads(row['message']['data']))
    elif row['topic'] == '/wildos/native_control_command': sources.append(json.loads(row['message']['data']))
trace = [json.loads(line) for line in (session/'native_timing.jsonl').open()]
trace = [r for r in trace if 'issued_wall_ns' in r]
age = sorted((r['callback_wall_ns']-r['issued_wall_ns'])/1e6 for r in trace)
positive = sum(v['sent']['throttle'] > 0 and v['sent']['brake'] < .9 for v in native)
moving_targets = sum(abs(v.get('target_speed', 0.)) > 1e-6 for v in control)
assert native and control and sources and positive == 0 and moving_targets == 0
assert not any(v['actuating'] for v in control)
report = dict(scope=__doc__, source_driver=str(run), session=str(session),
    idle_wait_sec=60, observed_callback_span_sec=(trace[-1]['callback_mono_ns']-trace[0]['callback_mono_ns'])/1e9,
    records=len(trace), source_to_callback_ms=dict(p95=age[int(.95*len(age))], maximum=age[-1]),
    over_200ms=sum(v > 200 for v in age), native_samples=len(native),
    maximum_observed_speed=max(v['speed'] for v in native),
    positive_drive_commands=positive, nonzero_speed_targets=moving_targets,
    watchdog_samples=sum(v.get('watchdog_stop', False) for v in native),
    controller_reasons=sorted({v['reason'] for v in control}),
    configured_native_qos='RELIABLE depth 1; diagnostic BEST_EFFORT unset',
    goals=0, navigation_success=False, historical_latency_fixed=False,
    inherited_driver_scope_note='Driver scope text says 420s search; --idle actually sent zero queries and waited 60 seconds after readiness.',
    errors=result['errors'])
(session/'idle_connectivity_review.json').write_text(json.dumps(report, indent=2)+'\n')
selected = ['native_timing.jsonl', 'native_udp.jsonl', 'native_samples.jsonl',
            'udp_timing_review.json', 'idle_connectivity_review.json', 'stop.json',
            'session_result.json', 'verified_sources.json', 'owned_processes.json',
            'execution_observer/messages.jsonl', 'stack.log', 'preflight.log']
payloads = {str(Path('session')/name): session/name for name in selected}
payloads.update({'driver/'+name: run/name for name in ('result.json', 'config.json', 'console.log', 'prestart.log')})
assert all(path.is_file() for path in payloads.values())
manifest = {name: dict(bytes=path.stat().st_size, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            for name, path in payloads.items()}
archive = run/'idle_connectivity_evidence.zip'
with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as zf:
    for name, path in payloads.items(): zf.write(path, name)
    zf.writestr('manifest.json', json.dumps(manifest, indent=2)+'\n')
identity = dict(archive=str(archive), bytes=archive.stat().st_size,
                sha256=hashlib.sha256(archive.read_bytes()).hexdigest(), payloads=len(payloads), review=report)
(run/'idle_connectivity_identity.json').write_text(json.dumps(identity, indent=2)+'\n')
print(json.dumps(identity, indent=2))
