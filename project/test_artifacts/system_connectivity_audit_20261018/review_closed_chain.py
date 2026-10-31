"""Join closed observer records without equating public status to CPU/lock time."""
from pathlib import Path
import bisect
import collections
import hashlib
import json
import math

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
SESSION = ROOT/'test_artifacts/smac_full_field_20261018/closed_evidence/session'
goal = json.loads((SESSION/'goal_001/result.json').read_text())
canonical = json.loads((SESSION/'full_query_review.json').read_text())
assert (SESSION/'session_result.json').is_file(), 'Closed evidence only'
lo = goal['started']; hi = lo+goal['elapsed_s']
native = []; plans = []; sources = {}; controls = {}; stamped_ages = []
def key(value):
    return tuple(value.get(k) for k in ('sender_id', 'generation', 'sequence'))
for line in (SESSION/'execution_observer/messages.jsonl').open(encoding='utf8'):
    row = json.loads(line); topic = row['topic']
    if topic not in ('/wildos/native_control_command', '/wildos/control_status',
                     '/wildos/native_control_status', '/wildos/plan_status',
                     '/wildos/cmd_vel_stamped'):
        continue
    t = row['received_monotonic_ns']/1e9
    if topic == '/wildos/cmd_vel_stamped':
        if lo <= t <= hi:
            stamp = row['message']['header']['stamp']
            stamped_ages.append((row['received_ros_ns']-stamp['sec']*10**9-stamp['nanosec'])/1e9)
        continue
    value = json.loads(row['message']['data'])
    if topic == '/wildos/native_control_command': sources[key(value)] = value
    elif topic == '/wildos/control_status': controls[key(value.get('source_control') or {})] = value
    elif topic == '/wildos/plan_status': plans.append((t, value))
    elif lo <= t <= hi: native.append((t, value))
pt = [t for t, _ in plans]
fault = next(t for t, p in plans if lo <= t <= hi and
             p['status'] == 'graph_reset_failed' and p.get('reason') == 'response_budget_exhausted')
phases = {name: dict(covered_sec=0., stopped_sec=0., distance_m=0.,
                    stopped_by_source=collections.defaultdict(float),
                    stopped_by_public_status=collections.defaultdict(float),
                    plan_status_counts=collections.Counter())
          for name in ('before_reset_budget_failure', 'after_reset_budget_failure')}
for (t, v), (end, following) in zip(native, native[1:]):
    dt = min(.2, max(0., end-t))
    name = 'before_reset_budget_failure' if t < fault else 'after_reset_budget_failure'
    phase = phases[name]; phase['covered_sec'] += dt
    phase['distance_m'] += math.dist(v['position'][:2], following['position'][:2])
    if v['speed'] >= .05: continue
    phase['stopped_sec'] += dt
    receipt = v.get('source_receipt') or {}
    frame, control = sources.get(key(receipt)), controls.get(key(receipt))
    matched = (frame and control and receipt.get('applied') is True and
               all(receipt.get(k) == frame.get(k) for k in
                   ('issued_wall_ns', 'expires_wall_ns', 'control')) and
               receipt['control'] == v['sent'])
    phase['stopped_by_source'][control['reason'] if matched else 'unmatched_or_not_applied'] += dt
    index = bisect.bisect_right(pt, t)-1
    phase['stopped_by_public_status'][plans[index][1]['status'] if index >= 0 else 'no_status'] += dt
for t, p in plans:
    if lo <= t <= hi:
        name = 'before_reset_budget_failure' if t < fault else 'after_reset_budget_failure'
        phases[name]['plan_status_counts'][p['status']] += 1
for phase in phases.values():
    assert abs(sum(phase['stopped_by_source'].values())-phase['stopped_sec']) < 1e-6
    assert abs(sum(phase['stopped_by_public_status'].values())-phase['stopped_sec']) < 1e-6
assert abs(sum(p['stopped_sec'] for p in phases.values())-canonical['stopped_sec']) < 1e-6
assert abs(sum(p['covered_sec'] for p in phases.values())-canonical['covered_sec']) < 1e-6
assert abs(sum(p['distance_m'] for p in phases.values())-canonical['distance_m']) < 1e-6
important = {'visible_target_approach', 'visible_target_route_timeout', 'follow_failed',
             'visible_target_reobservation_timeout', 'graph_reset_failed',
             'exploration_handoff_stopped', 'replanning_after_maneuver'}
events = [dict(goal_elapsed=t-lo, **v) for t, v in plans if
          lo <= t <= hi and v['status'] in important]
ages = sorted(stamped_ages)
vision = []
wall_lo = canonical['goal_started_wall']
for line in (SESSION/'vision_participation.jsonl').open(encoding='utf8'):
    value = json.loads(line)
    if (value.get('kind') == 'scores' and value.get('query') == goal['query']
            and wall_lo <= value['wall'] <= wall_lo+goal['elapsed_s']):
        vision.append(value)
vision_report = dict(
    target_query_score_frames=len(vision),
    frames_with_nondefault_nodes=sum(v.get('nondefault_nodes', 0) > 0 for v in vision),
    maximum_memory_utility_delta=max((v.get('max_memory_utility_delta', 0.) for v in vision), default=None),
    maximum_retrieve_calls=max((v.get('state', {}).get('retrieve_calls', 0) for v in vision), default=None),
    score_errors=sum(len(v.get('errors', [])) for v in vision),
    scope='Nondefault outputs demonstrate scorer activity, not a causal ablation or memory-increment acceptance')
report = dict(scope=__doc__, source_session=str(SESSION),
    reset_budget_failure_goal_elapsed=fault-lo, phases=phases,
    vision_participation=vision_report,
    stamped_to_observer_sec=dict(count=len(ages), p95=ages[int(.95*len(ages))],
                                 maximum=ages[-1]) if ages else None,
    causal_boundary_events=events,
    accounting_checks='Phase totals conserve the independently sealed canonical totals',
    conclusions=dict(
        reset_bug_is_not_sufficient_to_explain_all_stopping=True,
        stale_command_reason_does_not_prove_network_delay=True,
        public_status_is_not_private_planning_cpu_or_lock_time=True,
        no_new_field_test=True, production_changed=False,
        physical_contact_cause_confirmed=False))
(OUT/'closed_chain_review.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf8')
print(json.dumps({k: v for k, v in report.items() if k != 'causal_boundary_events'}, indent=2))
