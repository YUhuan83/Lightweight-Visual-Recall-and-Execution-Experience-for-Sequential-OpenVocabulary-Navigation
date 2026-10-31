"""Read only progress snapshots; provisional file-window motion statistics."""
from pathlib import Path
from collections import Counter
import json
import math
import sys

root = Path(sys.argv[1] if len(sys.argv)>1 else '/home/ly/execution_continuity_review_20261022/v1/full')
trials = sorted(root.glob('trial_*'))
if not trials:
    print(json.dumps({'state': 'not_started'})); raise SystemExit()
trial = trials[-1]
goals = sorted(trial.glob('sessions/*/goal_*'))
out = {'trial': str(trial), 'goal_started': bool(goals)}
if (trial / 'result.json').exists():
    out['closed_result'] = json.loads((trial / 'result.json').read_text())
if goals:
    goal = goals[-1]
    def read_rows(name):
        rows = []
        p = goal / name
        if p.exists():
            for line in p.read_text().splitlines():
                try: rows.append(json.loads(line))
                except ValueError: pass
        return rows
    events = read_rows('events.jsonl')
    native = read_rows('native_samples.jsonl')
    plans = [e for e in events if e.get('kind') == 'plan_status']
    out['last_status'] = [{k: e[k] for k in ('elapsed','status','reason','exploration_id') if k in e}
                          for e in plans[-6:]]
    counts = Counter(e.get('status') for e in plans)
    out['state_counts'] = {k: counts[k] for k in (
        'retired_tail_candidate_revalidated','retired_tail_candidate_rejected',
        'exploration_segment_finished','planning','no_vehicle_path','object_found')}
    if native:
        covered = stopped = 0.
        for a,b in zip(native,native[1:]):
            dt = b['wall_time']-a['wall_time']
            if 0. < dt <= .5 and a.get('telemetry_ok'):
                covered += dt
                if abs(a['speed']) < .1: stopped += dt
        out['provisional_native'] = {
            'file_window_s': native[-1]['wall_time']-native[0]['wall_time'],
            'covered_s': covered, 'stopped_s': stopped,
            'near_stop_pct': 100*stopped/covered if covered else None,
            'net_displacement_m': math.dist(native[0]['position'][:2],native[-1]['position'][:2]),
            'position': native[-1]['position'], 'speed': native[-1]['speed']}
    out['kind_counts'] = dict(Counter(e.get('kind') for e in events))
print(json.dumps(out, indent=2))
