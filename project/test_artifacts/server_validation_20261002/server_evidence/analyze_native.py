"""Measure actual turning and stopped-command episodes from native telemetry."""
from pathlib import Path
import json
import math
import statistics
import sys
root=Path('/home/ly/full_town_memory_20261002')/sys.argv[1]
samples=[]
for line in (root/'native_samples.jsonl').read_text().splitlines():
    try: samples.append(json.loads(line))
    except json.JSONDecodeError: pass
def heading(s): return math.atan2(s['direction'][1],s['direction'][0])
curves=[]
for i,a in enumerate(samples):
    steer=a.get('actual',{}).get('steering_input')
    if steer is None or abs(steer)<.59 or a['speed']<.3: continue
    for b in samples[i+1:]:
        current=b.get('actual',{}).get('steering_input')
        if (current is None or abs(current-steer)>.015 or b['speed']<.3
                or b['sent']['gear']!=a['sent']['gear']): break
        gap=math.dist(a['position'][:2],b['position'][:2])
        if gap>=1:
            angle=abs(math.atan2(math.sin(heading(b)-heading(a)),math.cos(heading(b)-heading(a))))
            if angle>.01:
                dt=b['wall_time']-a['wall_time']
                signed_angle=math.atan2(math.sin(heading(b)-heading(a)),math.cos(heading(b)-heading(a)))
                lateral=[-s['velocity'][0]*math.sin(heading(s))+s['velocity'][1]*math.cos(heading(s)) for s in (a,b)]
                curves.append({'time':a['wall_time'],'phase':a['phase'],'gear':a['sent']['gear'],
                               'radius':gap/(2*math.sin(angle/2)),'distance':gap,'turn':angle,
                               'estimated_reference_forward_offset':sum(lateral)/2/(signed_angle/dt)})
            break
stalls=[s for s in samples if s['speed']<.02 and s['sent']['throttle']>0 and not s['watchdog_stop'] and s['sent']['brake']==0]
report={'attempt':sys.argv[1],'samples':len(samples),'saturated_steering_windows':len(curves),
        'turning_radius_median':statistics.median(c['radius'] for c in curves) if curves else None,
        'turning_radius_range':[min(c['radius'] for c in curves),max(c['radius'] for c in curves)] if curves else None,
        'estimated_reference_forward_offset_median':statistics.median(c['estimated_reference_forward_offset'] for c in curves) if curves else None,
        'stopped_under_throttle_samples':len(stalls),
        'stopped_under_throttle_example':stalls[len(stalls)//2] if stalls else None,
        'turn_windows':curves}
(root/'native_analysis.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='turn_windows'},indent=2))
